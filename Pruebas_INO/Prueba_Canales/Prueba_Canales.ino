#include <Wire.h>

int MUX = 0x70;
void pcasel(uint8_t i) {
  Wire.beginTransmission(0x70);
  Wire.write(1 << i);
  Wire.endTransmission();
}

void setup() {
  Wire.begin();
  Serial.begin(115200); // Baudrate
  delay(1000);
  Serial.println("\n--- Escaneando canales del Multiplexor ---");

  for (uint8_t canal = 0; canal < 8; canal++) {
    pcasel(canal);
    Serial.print("Canal #"); Serial.print(canal); Serial.print(": ");
    
    bool encontrado = false;
    for (uint8_t addr = 1; addr < 127; addr++) {
      if (addr == MUX) continue;
      
      Wire.beginTransmission(addr);
      if (Wire.endTransmission() == 0) {
        Serial.print("ID encontrada en [0x");
        if (addr < 16) Serial.print("0");
        Serial.print(addr, HEX);
        Serial.print("] ");
        encontrado = true;
      }
    }
    if (!encontrado) Serial.print("Vacío");
    Serial.println();
  }
}

void loop() {}
