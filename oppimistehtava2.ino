#include <Arduino.h>
#include <esp_task_wdt.h>
#include <esp_system.h>  // esp_reset_reason()

// --- Tuning knobs ---
static constexpr int      kWdtTimeoutSeconds     = 5;
static constexpr uint32_t kFeedIntervalMs        = 1000;
static constexpr uint32_t kDisableWindowMs       = 30000;
static constexpr uint32_t kStatusPrintIntervalMs = 5000;
static constexpr uint32_t kMinFeedCount          = 6;

// Stretch test tuning
static constexpr uint32_t kStretchStartMs        = 1000;
static constexpr uint32_t kStretchHoldMs         = 8000;
static constexpr uint32_t kCoarseStepMs          = 500;
static constexpr uint32_t kFineStepMs            = 100;

// Store state in RTC memory without auto-initialization on boot.
struct RtcState {
  uint32_t magic;
  uint32_t bootCount;
  uint32_t phase;

  // Coarse search
  uint32_t coarseIntervalMs;
  uint32_t coarseLastGoodMs;
  uint32_t coarseFailMs;

  // Fine search
  uint32_t fineIntervalMs;
  uint32_t fineLastGoodMs;
  uint32_t fineFailMs;

  // Final result
  uint32_t finalLastGoodMs;
  uint32_t finalFailMs;
};

static constexpr uint32_t kRtcMagic = 0xC0FFEE42;
RTC_NOINIT_ATTR static RtcState gRtc;

// Runtime counters (normal RAM)
static uint32_t gLastFeedMs = 0;
static uint32_t gFeedCount  = 0;

enum class Phase : uint32_t {
  FeedWarmup    = 0,
  ResetTest1    = 1,
  ResetTest2    = 2,
  Disabled30s   = 3,
  FeedAfterOn   = 4,
  StretchCoarse = 5,
  StretchFine   = 6,
  Result        = 7
};

static const char* resetReasonToString(esp_reset_reason_t reason)
{
  switch (reason) {
    case ESP_RST_POWERON:   return "POWERON";
    case ESP_RST_EXT:       return "EXT (external reset pin)";
    case ESP_RST_SW:        return "SW (software reset)";
    case ESP_RST_PANIC:     return "PANIC (abort/panic)";
    case ESP_RST_INT_WDT:   return "INT_WDT (interrupt watchdog)";
    case ESP_RST_TASK_WDT:  return "TASK_WDT (task watchdog)";
    case ESP_RST_WDT:       return "WDT (other watchdog)";
    case ESP_RST_DEEPSLEEP: return "DEEPSLEEP";
    case ESP_RST_BROWNOUT:  return "BROWNOUT";
    case ESP_RST_SDIO:      return "SDIO";
    default:                return "UNKNOWN";
  }
}

static void rtcInitIfNeeded()
{
  if (gRtc.magic != kRtcMagic) {
    gRtc.magic = kRtcMagic;
    gRtc.bootCount = 0;
    gRtc.phase = static_cast<uint32_t>(Phase::FeedWarmup);

    gRtc.coarseIntervalMs = kStretchStartMs;
    gRtc.coarseLastGoodMs = 0;
    gRtc.coarseFailMs     = 0;

    gRtc.fineIntervalMs   = 0;
    gRtc.fineLastGoodMs   = 0;
    gRtc.fineFailMs       = 0;

    gRtc.finalLastGoodMs  = 0;
    gRtc.finalFailMs      = 0;
  }
}

static void printBootHeader()
{
  const esp_reset_reason_t reason = esp_reset_reason();

  Serial.println();
  Serial.println("======================================");
  Serial.print("BOOT #"); Serial.println(gRtc.bootCount);
  Serial.print("Reset reason: "); Serial.println(resetReasonToString(reason));
  Serial.print("Phase: "); Serial.println(gRtc.phase);

  if (static_cast<Phase>(gRtc.phase) == Phase::Result) {
    Serial.println("--- Stored result ---");
    Serial.print("Coarse lastGood="); Serial.print(gRtc.coarseLastGoodMs); Serial.print(" ms, ");
    Serial.print("coarse fail=");     Serial.print(gRtc.coarseFailMs);     Serial.println(" ms");

    Serial.print("Fine lastGood=");   Serial.print(gRtc.fineLastGoodMs);   Serial.print(" ms, ");
    Serial.print("fine fail=");       Serial.print(gRtc.fineFailMs);       Serial.println(" ms");

    Serial.print("Final lastGood=");  Serial.print(gRtc.finalLastGoodMs);  Serial.print(" ms, ");
    Serial.print("final fail=");      Serial.print(gRtc.finalFailMs);      Serial.println(" ms");
  }
}

static bool wdtEnable()
{
  esp_err_t err = esp_task_wdt_init(kWdtTimeoutSeconds, true); // reset on timeout
  if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
    Serial.print("WDT init failed: ");
    Serial.println(esp_err_to_name(err));
    return false;
  }

  err = esp_task_wdt_add(nullptr); // watch current loop task
  if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
    Serial.print("WDT add failed: ");
    Serial.println(esp_err_to_name(err));
    return false;
  }

  Serial.print("WDT enabled (timeout ");
  Serial.print(kWdtTimeoutSeconds);
  Serial.println(" s)");
  return true;
}

static void wdtDisable()
{
  // These may return INVALID_STATE depending on internal lifecycle; that's fine.
  esp_err_t err = esp_task_wdt_delete(nullptr);
  if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
    Serial.print("WDT delete failed: ");
    Serial.println(esp_err_to_name(err));
  }

  err = esp_task_wdt_deinit();
  if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
    Serial.print("WDT deinit failed: ");
    Serial.println(esp_err_to_name(err));
  }

  Serial.println("WDT disabled");
}

static void wdtFeedIfDue(uint32_t intervalMs)
{
  const uint32_t now = millis();
  if (now - gLastFeedMs < intervalMs) return;

  const esp_err_t err = esp_task_wdt_reset();
  if (err == ESP_OK) {
    gFeedCount++;
    Serial.print("Fed WDT (");
    Serial.print(gFeedCount);
    Serial.print(") interval=");
    Serial.print(intervalMs);
    Serial.println(" ms");
  } else {
    Serial.print("WDT feed failed: ");
    Serial.println(esp_err_to_name(err));
  }

  gLastFeedMs = now;
}

static void resetCountersForPhase()
{
  gFeedCount  = 0;
  gLastFeedMs = millis();
}

static void advancePhaseOnWdtReset()
{
  if (esp_reset_reason() != ESP_RST_TASK_WDT) return;

  const Phase p = static_cast<Phase>(gRtc.phase);

  if (p == Phase::ResetTest1) {
    gRtc.phase = static_cast<uint32_t>(Phase::ResetTest2);
    return;
  }

  if (p == Phase::ResetTest2) {
    gRtc.phase = static_cast<uint32_t>(Phase::Disabled30s);
    return;
  }

  if (p == Phase::StretchCoarse) {
    // Coarse failure occurred at coarseIntervalMs.
    gRtc.coarseFailMs = gRtc.coarseIntervalMs;
    gRtc.coarseLastGoodMs = (gRtc.coarseIntervalMs > kCoarseStepMs)
                              ? (gRtc.coarseIntervalMs - kCoarseStepMs)
                              : 0;

    // Prepare fine search starting from last good (coarse).
    gRtc.fineIntervalMs = gRtc.coarseLastGoodMs;
    gRtc.fineLastGoodMs = 0;
    gRtc.fineFailMs     = 0;

    gRtc.phase = static_cast<uint32_t>(Phase::StretchFine);
    return;
  }

  if (p == Phase::StretchFine) {
    // Fine failure occurred at fineIntervalMs.
    gRtc.fineFailMs = gRtc.fineIntervalMs;
    gRtc.fineLastGoodMs = (gRtc.fineIntervalMs > kFineStepMs)
                            ? (gRtc.fineIntervalMs - kFineStepMs)
                            : 0;

    // Final results
    gRtc.finalLastGoodMs = gRtc.fineLastGoodMs;
    gRtc.finalFailMs     = gRtc.fineFailMs;

    gRtc.phase = static_cast<uint32_t>(Phase::Result);
    return;
  }
}

static void hangUntilReset(const char* title)
{
  Serial.println();
  Serial.println(title);
  Serial.println("Hanging without feeding the watchdog...");
  Serial.flush();

  while (true) {
    // Intentionally no esp_task_wdt_reset() here.
  }
}

static void runDisableWindow()
{
  Serial.println();
  Serial.println("Disabling WDT for a while...");
  wdtDisable();

  const uint32_t start = millis();
  uint32_t lastPrint = 0;

  while (millis() - start < kDisableWindowMs) {
    const uint32_t elapsed = millis() - start;

    if (elapsed - lastPrint >= kStatusPrintIntervalMs) {
      Serial.print("WDT disabled, elapsed ");
      Serial.print(elapsed / 1000);
      Serial.println(" s");
      lastPrint = elapsed;
    }

    delay(10);
  }

  Serial.println("Re-enabling WDT...");
  wdtEnable();
}

static void stretchStepLogic(uint32_t& intervalMs, uint32_t stepMs, const char* label)
{
  wdtFeedIfDue(intervalMs);

  // Hold each interval, then increase.
  static uint32_t intervalStartMs = 0;
  static uint32_t lastPrintedInterval = 0;

  if (intervalStartMs == 0) {
    intervalStartMs = millis();
  }

  if (intervalMs != lastPrintedInterval) {
    Serial.print(label);
    Serial.print(" interval=");
    Serial.print(intervalMs);
    Serial.println(" ms");
    lastPrintedInterval = intervalMs;
  }

  const uint32_t heldMs = millis() - intervalStartMs;
  if (heldMs >= kStretchHoldMs) {
    intervalMs += stepMs;   // no cap / target
    intervalStartMs = 0;
  }

  // Periodic status print
  static uint32_t lastHeartbeatMs = 0;
  if (millis() - lastHeartbeatMs >= kStatusPrintIntervalMs) {
    Serial.print(label);
    Serial.print(" running: interval=");
    Serial.print(intervalMs);
    Serial.println(" ms");
    lastHeartbeatMs = millis();
  }
}

void setup()
{
  Serial.begin(115200);
  delay(250);

  rtcInitIfNeeded();
  gRtc.bootCount++;

  // If last boot ended due to TASK_WDT, move forward.
  advancePhaseOnWdtReset();

  printBootHeader();

  // Enable watchdog by default (it is explicitly disabled only in Disabled30s).
  wdtEnable();
  resetCountersForPhase();
}

void loop()
{
  const Phase phase = static_cast<Phase>(gRtc.phase);

  switch (phase) {
    case Phase::FeedWarmup: {
      wdtFeedIfDue(kFeedIntervalMs);

      if (gFeedCount >= kMinFeedCount) {
        Serial.println("Warmup feeding done -> going to reset test #1");
        gRtc.phase = static_cast<uint32_t>(Phase::ResetTest1);
        resetCountersForPhase();
        delay(200);
      }
      break;
    }

    case Phase::ResetTest1:
      hangUntilReset("RESET TEST #1");
      break;

    case Phase::ResetTest2:
      hangUntilReset("RESET TEST #2");
      break;

    case Phase::Disabled30s: {
      runDisableWindow();
      Serial.println("WDT is back on -> feeding again");
      gRtc.phase = static_cast<uint32_t>(Phase::FeedAfterOn);
      resetCountersForPhase();
      break;
    }

    case Phase::FeedAfterOn: {
      wdtFeedIfDue(kFeedIntervalMs);

      if (gFeedCount >= kMinFeedCount) {
        Serial.println("Post-enable feeding done -> entering stretch (coarse)");
        gRtc.phase = static_cast<uint32_t>(Phase::StretchCoarse);

        // Initialize coarse search state
        gRtc.coarseIntervalMs = kStretchStartMs;
        gRtc.coarseLastGoodMs = 0;
        gRtc.coarseFailMs     = 0;

        // Clear fine/final
        gRtc.fineIntervalMs   = 0;
        gRtc.fineLastGoodMs   = 0;
        gRtc.fineFailMs       = 0;
        gRtc.finalLastGoodMs  = 0;
        gRtc.finalFailMs      = 0;

        resetCountersForPhase();
        delay(200);
      }
      break;
    }

    case Phase::StretchCoarse: {
      // Coarse: +500ms steps until TASK_WDT reset happens.
      stretchStepLogic(gRtc.coarseIntervalMs, kCoarseStepMs, "COARSE");
      break;
    }

    case Phase::StretchFine: {
      // Fine: start from coarse last good and increase +100ms until reset.
      if (gRtc.fineIntervalMs == 0) {
        gRtc.fineIntervalMs = gRtc.coarseLastGoodMs;
      }
      stretchStepLogic(gRtc.fineIntervalMs, kFineStepMs, "FINE");
      break;
    }

    case Phase::Result: {
      // Print once, then disable watchdog and stop output so Serial Monitor stops scrolling.
      static bool done = false;
      if (!done) {
        Serial.println();
        Serial.println("=== RESULT ===");
        Serial.print("Coarse fail: "); Serial.print(gRtc.coarseFailMs); Serial.println(" ms");
        Serial.print("Coarse last good: "); Serial.print(gRtc.coarseLastGoodMs); Serial.println(" ms");
        Serial.print("Fine fail: "); Serial.print(gRtc.fineFailMs); Serial.println(" ms");
        Serial.print("Fine last good: "); Serial.print(gRtc.fineLastGoodMs); Serial.println(" ms");
        Serial.print("Final last good (best estimate): "); Serial.print(gRtc.finalLastGoodMs); Serial.println(" ms");
        Serial.print("Final fail: "); Serial.print(gRtc.finalFailMs); Serial.println(" ms");

        Serial.println();
        Serial.println("Disabling WDT and stopping output...");
        Serial.flush();

        wdtDisable();

        done = true;
      }

      // Idle without prints.
      delay(1000);
      break;
    }

    default:
      gRtc.phase = static_cast<uint32_t>(Phase::FeedWarmup);
      resetCountersForPhase();
      break;
  }
}