#include <AccelStepper.h>

const int senNum = 2; // Número de sensores

enum Comandos{
    CMD_VEL = 4,
    CMD_MOVE = 5,
    CMD_PARO = 6
};

// Datos para recibir y mandar comandos
struct __attribute__((packed)) VelPacket
{
    int16_t vel[senNum];
}; VelPacket velData;

struct __attribute__((packed)) MovePacket
{
    int16_t pasos[senNum];
}; MovePacket moveData;

struct __attribute__((packed)) AfirmPacket
{
    uint8_t encabezado = 0xAA;
    uint8_t confirm = 0x55;
}; AfirmPacket afirm;

// Pines CNC Shield
const int stepX = 2; const int dirX = 5;
const int stepY = 3; const int dirY = 6;
const int enablePin = 8;

AccelStepper motorX(1, stepX, dirX);
AccelStepper motorY(1, stepY, dirY);

void setup() {
  Serial.begin(250000);
  
  pinMode(enablePin, OUTPUT);
  digitalWrite(enablePin, LOW); 

  motorX.setMaxSpeed(4074);
  motorY.setMaxSpeed(4074);

  motorX.setAcceleration(250);
  motorY.setAcceleration(250);

  Serial.write(0xAA); // Encabezado
  Serial.write(0x03); // Mensaje

  while (true){
    if (Serial.available() >= 2){
      uint8_t h = Serial.read();
      uint8_t c = Serial.read();

      if (h == 0xAA && c == 0x04){
        break;
      }

    }

  }

}

void comandos(){
  if (Serial.available() >= 2){

    if (Serial.read() == 0xAA){

      uint8_t cmd = Serial.read();

      if (cmd == CMD_VEL){

        Serial.readBytes((char*)&velData, sizeof(velData));

        int v1 = velData.vel[0];
        int v2 = velData.vel[1];

        motorX.setSpeed(v1);
        motorY.setSpeed(v2);

      }else if (cmd == CMD_MOVE){

        Serial.readBytes((char*)&moveData, sizeof(moveData));

        int p1 = moveData.pasos[0];
        int p2 = moveData.pasos[1];

        motorX.move(p1);
        motorY.move(p2);

        bool band = true;
        while (band){

          motorX.run();
          motorY.run();

          if (motorX.distanceToGo() == 0 && motorY.distanceToGo() == 0){
            Serial.write((uint8_t*)&afirm, sizeof(afirm));
            band = false;
          }

        }

      }else if (cmd == CMD_PARO){
        motorX.stop();
        motorY.stop();
        motorX.setSpeed(0);
        motorY.setSpeed(0);
      }

    }

  }

}

void loop(){
  motorX.runSpeed();
  motorY.runSpeed();
  comandos();
}