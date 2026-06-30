#include <AccelStepper.h>

// Pines CNC Shield v3
const int stepX = 2; const int dirX = 5;
const int stepY = 3; const int dirY = 6;
const int enablePin = 8;

AccelStepper motorX(1, stepX, dirX);
AccelStepper motorY(1, stepY, dirY);

void setup() {
  Serial.begin(115200); // baudrate
  pinMode(enablePin, OUTPUT);
  digitalWrite(enablePin, LOW); // Habilitar motores

  motorX.setMaxSpeed(4000); // Límite máximo de pulsos/seg
  motorY.setMaxSpeed(4000);
}

void loop() {
  if (Serial.available() > 0) {
    char c = Serial.read();
    if (c == 'v') {
      long p_x = Serial.parseInt();
      long p_y = Serial.parseInt();
      
      motorX.setSpeed(p_x);
      motorY.setSpeed(p_y);
    }
  }
  
  // Estas funciones deben ejecutarse lo más rápido posible
  motorX.runSpeed();
  motorY.runSpeed();
}
