#include <Wire.h>

const int PCA_ADDR = 0x70; // direccion del MUX
const int MT6701_ADDR = 0x06; // direccion del sensor

// inicializar mux
void pcasel(uint8_t bus) {
  if (bus > 7) return;
  Wire.beginTransmission(PCA_ADDR);
  Wire.write(1 << bus);
  Wire.endTransmission();
}

// inicializar conexion
void setup() {
  Wire.begin();
  Serial.begin(115200);
  //Serial.println("Lectura de MT6701 Iniciada...");
}

float obtenerAngulo(int canal) {
  pcasel(canal); // Seleccionar canal del multiplexor
  
  Wire.beginTransmission(MT6701_ADDR);
  Wire.write(0x03); 
  if (Wire.endTransmission() != 0) return -1.0; // Error de comunicación

  Wire.requestFrom(MT6701_ADDR, 2);

  if (Wire.available() == 2) {
    uint8_t msb = Wire.read();
    uint8_t lsb = Wire.read();
    uint16_t rawData = (msb << 6) | (lsb >> 2);
    //  obtener angulo en grados y corregir
    return (rawData * 360.0) / 16384.0;
  }
  return -1.0;
}

void loop() {
  // Solo actúa si Python envió algo
  if (Serial.available() > 0) {
    char comando = Serial.read(); // Leer la señal de Python

    if (comando == 'r') {
      float angulo0 = obtenerAngulo(0);
      float angulo1 = obtenerAngulo(1);

      // Enviamos la respuesta
      Serial.print(angulo0);
      Serial.print(","); 
      Serial.println(angulo1);
    }
  }
}