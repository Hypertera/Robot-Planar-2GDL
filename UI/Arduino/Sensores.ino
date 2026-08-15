#include <Wire.h>

const int PCA_ADDR = 0x70; // Dirección del multiplexor
const int MT6701_ADDR = 0x06; // Dirección del MT6701
const int senNum = 2; // Número de sensores 

enum Comandos{
    CMD_POS = 1,
    CMD_CAL = 2,
    CMD_RESET_TURNS = 3,
};

// Datos del sensor y calibración
struct MT6701{
  float angulo = 0.0, desfase = 0.0;
  bool dir = false;
  int vueltas = 0, cuadranteActl = 0, cuadranteAnt = 0;
}; MT6701 Sensor[senNum];

// Datos para recibir y mandar comandos
struct __attribute__((packed)) AngPacket
{
    uint8_t encabezado = 0xAA;
    float ang[senNum];
    bool est[senNum];
}; AngPacket angData;

struct __attribute__((packed)) CalPacket
{
    int16_t ID;
    float desfase;
    bool dir;
}; CalPacket cal;

struct __attribute__((packed)) EstPacket
{
    uint8_t encabezado = 0xAA;
    bool est[senNum];
}; EstPacket estData;

struct __attribute__((packed)) AfirmPacket
{
    uint8_t encabezado = 0xAA;
    uint8_t val;
}; AfirmPacket afirm;

void setup() {
  Wire.begin();
  Wire.setClock(400000);
  Serial.begin(250000); // baudrate
  //delay(2000);
  
  Serial.write(0xAA);
  Serial.write(0x01);

  while (true){
    if (Serial.available() >= 2){
      uint8_t h = Serial.read();
      uint8_t c = Serial.read();

      if (h == 0xAA && c == 0x02){
        break;
      }

    }

  }
  
}

// Configurar canales del multiplexor
void pcasel(uint8_t bus) {
  if (bus > 7) return;
  Wire.beginTransmission(PCA_ADDR);
  Wire.write(1 << bus);
  Wire.endTransmission();
}

// Funciones para calibrar y calcular sensores
float obtenerAngulo(int canal) {
  pcasel(canal);
  Wire.beginTransmission(MT6701_ADDR);
  Wire.write(0x03); 
  if (Wire.endTransmission() != 0) return 0xFF;

  Wire.requestFrom(MT6701_ADDR, 2);
  if (Wire.available() == 2) {
    uint16_t rawData = (Wire.read() << 6) | (Wire.read() >> 2);
    return (rawData * 360.0) / 16384.0;
  }
  return 0xFF;
}

void angCorrec(float anguloCrudo, int N){

  // correccion del angulo de inicio
  anguloCrudo = anguloCrudo - Sensor[N].desfase;
  if (anguloCrudo<0){
      anguloCrudo = anguloCrudo + 360;
    }

    // cambiar direccion de giro
    if (Sensor[N].dir && anguloCrudo !=0){
      Sensor[N].angulo =  360.0 - anguloCrudo;
    }
    else{
      Sensor[N].angulo = anguloCrudo;
    }    
}

void angTotal(int N){

  // determinar cuadrante
  if (Sensor[N].angulo>=0 && Sensor[N].angulo<=90){
    Sensor[N].cuadranteActl = 1;
  }
  else if (Sensor[N].angulo>90 && Sensor[N].angulo<=180){
    Sensor[N].cuadranteActl = 2;
  }
  else if (Sensor[N].angulo>180 && Sensor[N].angulo<=270){
    Sensor[N].cuadranteActl = 3;
  }
  else if (Sensor[N].angulo>270 && Sensor[N].angulo<360){
    Sensor[N].cuadranteActl = 4;
  }

  // determinar el cambio de cuadrante y aumentar num de vueltas
  if (Sensor[N].cuadranteActl != Sensor[N].cuadranteAnt){
    if(Sensor[N].cuadranteActl == 1 && Sensor[N].cuadranteAnt == 4){
        Sensor[N].vueltas++;
    }
    if(Sensor[N].cuadranteActl == 4 && Sensor[N].cuadranteAnt == 1){
        Sensor[N].vueltas--;
    }
      Sensor[N].cuadranteAnt = Sensor[N].cuadranteActl;
  }

  // calcular angulo total
  Sensor[N].angulo = (Sensor[N].vueltas*360) + Sensor[N].angulo;
}

void comandos(){
  if (Serial.available() >= 2){

    if (Serial.read() == 0xAA){

      uint8_t cmd = Serial.read();

      if (cmd == CMD_POS){

        for (int i=0; i<senNum; i++){
          float angCrudo = obtenerAngulo(i);
          angCorrec(angCrudo, i);
          angTotal(i);
          angData.ang[i] = Sensor[i].angulo;
        }

        //angData.ang1 = Sensor[0].angulo;
        //angData.ang2 = Sensor[1].angulo;

        Serial.write((char*)&angData, sizeof(angData));

      }else if (cmd == CMD_CAL){
        
        size_t n = Serial.readBytes((char*)&cal, sizeof(cal));

        int a;
        if (n == sizeof(cal)){
          if (cal.ID >= 0 && cal.ID < senNum){
            Sensor[cal.ID].desfase = cal.desfase;
            Sensor[cal.ID].dir = cal.dir;
            Sensor[cal.ID].vueltas = 0; 
            Sensor[cal.ID].cuadranteActl = 0;
            Sensor[cal.ID].cuadranteAnt = 0;
            a = 0x55;
          }else{
            a = 0xEE;
          }

        }else{
          a = 0xFF;
        }

        afirm.val = a;
        Serial.write((uint8_t*)&afirm, sizeof(afirm));

      }else if (cmd == CMD_RESET_TURNS){
        for (int i=0; i<senNum; i++){
          Sensor[i].vueltas = 0;
        }

        Serial.write((uint8_t*)&estData, sizeof(estData));
      } 

    }

  }

}

void loop() {
  comandos();
}