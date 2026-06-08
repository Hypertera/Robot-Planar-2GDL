#include <Wire.h>

void setup()
{
    Serial.begin(115200); // baudrate

    Wire.begin();

    Wire.setWireTimeout(1000, true);

    Serial.println("I2C listo");

    Serial.println("Escaneando...");

    for(uint8_t addr = 1; addr < 127; addr++)
    {
        Wire.beginTransmission(addr);

        uint8_t error = Wire.endTransmission();

        if(error == 0)
        {
            Serial.print("Dispositivo encontrado en dirección ");
            Serial.println(addr, HEX);
        }
    }
}

void loop(){}