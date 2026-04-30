#include <HardwareSerial.h>

// UART instance
HardwareSerial UART_from_L(1);

// Define pin
#define RX_FROM_L 17

void setup()
{
    // USB Serial monitor
    Serial.begin(115200);
    Serial.println("ESP32_R valmis");

    // Receive from ESP32_L
    UART_from_L.begin(
        38400,
        SERIAL_8E2,
        RX_FROM_L,
        -1
    );
}

void loop()
{
    if (UART_from_L.available())
    {
        byte received = UART_from_L.read();

        // Show in serial monitor
        Serial.print("Saapui: ");
        Serial.println(received);
    }
}