/*
  Tehtävä 1: ESP32 ADC + analogiset lämpötila-anturit LM35 & TMP36
  - Mittaa molemmat anturit (LM35, TMP36)
  - Tulostaa lämpötilat celsiusasteina Serial Monitoriin

  Kytkennät (tehtävämonisteen mukaan):
    TMP36 OUT -> GPIO34 (ADC1)
    LM35  OUT -> GPIO35 (ADC1)

  Huom:
  - ESP32 ADC:n tarkkuus ja Vref vaihtelevat sirujen välillä.
  - analogReadMilliVolts() antaa Arduino-ESP32 -ytimessä usein suoraan mV-arvion
    (sis. kalibrointia toteutuksesta riippuen), ja on siksi parempi kuin pelkkä raaka-arvo.
*/

#include <Arduino.h>

static constexpr int PIN_TMP36 = 34; // ADC1
static constexpr int PIN_LM35  = 35; // ADC1

// Näytteenottoasetukset
static constexpr uint16_t SAMPLES_PER_READING = 32;   // keskiarvoistettavat näytteet / mittaus
static constexpr uint32_t SAMPLE_DELAY_US     = 200;  // pieni viive näytteiden välillä (µs)

// IIR (exponential moving average) -suodatus: 0..1 (pienempi = enemmän suodatusta)
static constexpr float EMA_ALPHA = 0.20f;

// Tulostus
static constexpr uint32_t PRINT_INTERVAL_MS = 500;

struct SensorState {
  float emaMilliVolts = NAN;
};

static SensorState tmp36State;
static SensorState lm35State;

static uint32_t lastPrintMs = 0;

static float readMilliVoltsAveraged(int pin) {
  uint32_t acc = 0;
  for (uint16_t i = 0; i < SAMPLES_PER_READING; i++) {
#if defined(ARDUINO_ARCH_ESP32)
    // Arduino-ESP32: palauttaa mV (toteutus voi hyödyntää kalibrointia)
    acc += (uint32_t)analogReadMilliVolts(pin);
#else
    // Fallback (yleinen Arduino): muunna analogRead -> mV (oletus 3.3V)
    const int raw = analogRead(pin);
    acc += (uint32_t)((raw * 3300UL) / 4095UL);
#endif
    delayMicroseconds(SAMPLE_DELAY_US);
  }
  return (float)acc / (float)SAMPLES_PER_READING;
}

static float emaUpdate(SensorState &state, float newMilliVolts) {
  if (isnan(state.emaMilliVolts)) {
    state.emaMilliVolts = newMilliVolts; // init
  } else {
    state.emaMilliVolts = EMA_ALPHA * newMilliVolts + (1.0f - EMA_ALPHA) * state.emaMilliVolts;
  }
  return state.emaMilliVolts;
}

// LM35: 10 mV / °C, 0°C -> ~0 mV
static float lm35MilliVoltsToC(float mv) {
  return mv / 10.0f;
}

// TMP36: 10 mV / °C, 0°C -> 500 mV (offset), 25°C -> 750 mV
static float tmp36MilliVoltsToC(float mv) {
  return (mv - 500.0f) / 10.0f;
}

static void printReading(float mvTmp36, float mvLm35) {
  const float tTmp36 = tmp36MilliVoltsToC(mvTmp36);
  const float tLm35  = lm35MilliVoltsToC(mvLm35);
  const float diff   = tTmp36 - tLm35;

  Serial.println();
  Serial.println(F("=== Lämpötilamittaus (ESP32 ADC) ==="));
  Serial.print(F("TMP36: "));
  Serial.print(mvTmp36, 1);
  Serial.print(F(" mV -> "));
  Serial.print(tTmp36, 2);
  Serial.println(F(" °C"));

  Serial.print(F("LM35 : "));
  Serial.print(mvLm35, 1);
  Serial.print(F(" mV -> "));
  Serial.print(tLm35, 2);
  Serial.println(F(" °C"));

  Serial.print(F("Erotus (TMP36 - LM35): "));
  Serial.print(diff, 2);
  Serial.println(F(" °C"));
}

void setup() {
  Serial.begin(115200);
  delay(200);

#if defined(ARDUINO_ARCH_ESP32)
  // ESP32 ADC-asetukset (parantaa käytännössä toistettavuutta)
  analogReadResolution(12); // 0..4095
  // 11 dB attenuation -> mittausalue tyypillisesti ~0..3.3V (riittää näille antureille)
  analogSetPinAttenuation(PIN_TMP36, ADC_11db);
  analogSetPinAttenuation(PIN_LM35,  ADC_11db);
#endif

  Serial.println(F("Käynnistetty. Tulostetaan mittaukset Serial Monitoriin..."));
  Serial.println(F("Vinkki: anna anturin tasaantua hetki, vältä koskettamasta anturia sormilla mittauksen aikana."));
}

void loop() {
  // Lue ja suodata mV-arvot
  const float mvTmp36Raw = readMilliVoltsAveraged(PIN_TMP36);
  const float mvLm35Raw  = readMilliVoltsAveraged(PIN_LM35);

  const float mvTmp36 = emaUpdate(tmp36State, mvTmp36Raw);
  const float mvLm35  = emaUpdate(lm35State, mvLm35Raw);

  // Tulosta määräajoin
  const uint32_t now = millis();
  if (now - lastPrintMs >= PRINT_INTERVAL_MS) {
    lastPrintMs = now;
    printReading(mvTmp36, mvLm35);
  }
}
