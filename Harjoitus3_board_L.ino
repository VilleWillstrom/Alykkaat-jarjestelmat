#include <HardwareSerial.h>

// UART instances
HardwareSerial UART_from_M(1);  // UART1 receiving
HardwareSerial UART_to_R(2);    // UART2 sending

// Defining pins
#define RX_FROM_M 16
#define TX_TO_R   14

void setup()
{
    // USB Serial monitor
    Serial.begin(115200);
    Serial.println("ESP32_L valmis");

    // Receive from ESP32_M:ltä
    UART_from_M.begin(
        9600,
        SERIAL_8N1,
        RX_FROM_M,
        -1
    );

    // Send to ESP32_R:lle
    UART_to_R.begin(
        38400,
        SERIAL_8E2,
        -1,
        TX_TO_R
    );
}

void loop()
{
    if (UART_from_M.available())
    {
        byte received = UART_from_M.read();

        // Show in serial monitor
        Serial.print("Vastaanotettu: ");
        Serial.println(received);

        // Send forward
        UART_to_R.write(received);
    }
}