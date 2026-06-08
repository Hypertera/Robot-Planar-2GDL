import serial
import time
import math
PI = math.pi

# Configuración del motor
PASOS_POR_REV = 200   # Pasos nativos del NEMA 17
MICROSTEPPING = 4     # Ajusta según tus jumpers (1, 2, 4, 8, 16, 32)
RELACION_PASOS_RAD = (PASOS_POR_REV * MICROSTEPPING) / (2*PI)

# Configurar puerto
mot = serial.Serial(port='COM7', baudrate=115200, timeout=.1)
mot.reset_input_buffer()
time.sleep(2)
sen = serial.Serial(port='COM3', baudrate=115200, timeout=.1)
sen.reset_input_buffer()
sen.reset_output_buffer()
time.sleep(2)

def enviar_velocidad_rad(rad_x, rad_y):
    # Convertir radianes/s a pasos/s
    pasos_x = int(rad_x * RELACION_PASOS_RAD)
    pasos_y = int(rad_y * RELACION_PASOS_RAD)
    
    comando = f"v{pasos_x},{pasos_y}\n"
    mot.write(comando.encode())
    print(f"Enviando Pasos/s -> X: {pasos_x} | Y: {pasos_y}\n")

def leer_sen():
    # Leer sensores
    sen.write(b'r')
    cadena = sen.readline().decode('utf-8').strip()
    
    vector = [float(x) for x in cadena.split(',')]

    print(f"Sensores: {vector}\n")


enviar_velocidad_rad(2, 2) 

time.sleep(5)

enviar_velocidad_rad(0, 0)

try:
    while True:
        leer_sen()
        time.sleep(0.5)
except:
    sen.close()
    mot.close()
