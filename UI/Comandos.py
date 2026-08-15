import serial
import serial.tools.list_ports
import struct
import time
import math

class NANO:
    def __init__(self):
        
        self.arduino = None
        
        # Encabezado del mensaje
        self.Encabezado = b'\xAA' 
        
        # Comandos
        self.CMD_POS = b'\x01' # Consultar posición
        self.CMD_CAL = b'\x02' # Enviar calibración
        self.CMD_RESET_TURNS = b'\x03' # Consultar estado de los sensores
        
        # Formato de datos 
        self.SenFormat = '<2f' # float, float
        self.SenSize = struct.calcsize(self.SenFormat) # Calcular tamaño del mensaje
        self.RVFormat = '<2?' # bool, bool
        self.RVSize = struct.calcsize(self.RVFormat) # Calcular tamaño del mensaje
        self.CalFormat = '<hf?' # int16, float, bool
        
        self.ultima_pos = [0]*2
    
    def conectarArd(self):
        for puerto in serial.tools.list_ports.comports():
            try:
                ser = serial.Serial(
                    puerto.device,
                    baudrate=250000,
                    timeout=2
                )
    
                dato = ser.read(2)
    
                if dato == b'\xAA\x01':
                    ser.write(b'\xAA\x02')
                    print("Arduino encontrado en", puerto.device)
                    self.arduino = ser
                    return True
    
                ser.close()
    
            except Exception:
                pass
    
        return False
    
    def obtenerPosicion(self):
        
        # Enviar comando
        packet = self.Encabezado + self.CMD_POS
        self.arduino.write(packet)
        
        while self.arduino.in_waiting <= (self.SenSize + 1):
            byte = self.arduino.read(1)
            
        if byte == self.Encabezado:
            data = self.arduino.read(self.SenSize)
            ang = struct.unpack(self.SenFormat, data)
            self.ultima_pos = ang
            return ang
        else:
            return self.ultima_pos
    
    def enviarCal(self, ID, desfase, invertido):
        
        # Enviar comando
        packet = self.Encabezado + self.CMD_CAL
        packet += struct.pack(self.CalFormat, ID, desfase, invertido)
        self.arduino.write(packet)
        
        while True:
            byte = self.arduino.read(1)
    
            if byte == self.Encabezado:
                break

        data = self.arduino.read()
            
        if data == b'\x55':
            print("\n Sensor",ID,"Calibrado\n")
        elif data == b'\xEE':
            print("\n ID",ID,"Invalido\n")
        elif data == b'\xFF':
            print("\n Paquete incomleto, intente de nuevo\n")
                
    def reiniciarVueltas(self):
        # Enviar comando
        packet = self.Encabezado
        packet += self.CMD_RESET_TURNS
        self.arduino.write(packet)
        
    def cerrarSerial(self):
        self.arduino.close()
        time.sleep(0.5)
        print("\nConexión cerrada")

class UNO:
    def __init__(self):
        
        self.arduino = None
        
        # Encabezado del mensaje
        self.Encabezado = b'\xAA' 
        
        # Comandos
        self.CMD_VEL = b'\x04' # Enviar velocidades
        self.CMD_MOVE = b'\x05' # Enviar pose articular 
        self.CMD_PARO = b'\x06' # Parar motores
        
        # Formatp de datos
        self.VelFormat = '<2h' # int16, int16
        self.MoveFormat = '<2h' # int16, int16
        
        # Configuración de motor
        PASOS_POR_REV = 200   # Pasos nativos del NEMA 17
        MICROSTEPPING = 4     # MicrosPasos
        self.RELACION_PASOS_RAD = (PASOS_POR_REV * MICROSTEPPING) / (2 * math.pi)
        self.angRad = [0]*2
        self.pasos = [0]*2
        
    def conectarArd(self):
        for puerto in serial.tools.list_ports.comports():
            try:
                ser = serial.Serial(
                    puerto.device,
                    baudrate=250000,
                    timeout=2
                )
    
                dato = ser.read(2)
    
                if dato == b'\xAA\x03':
                    ser.write(b'\xAA\x04')
                    print("Arduino encontrado en", puerto.device)
                    self.arduino = ser
                    return True
    
                ser.close()
    
            except Exception:
                pass
    
        return False
    
    def enviarVel(self, v):
        
        # Convertir radianes/s a pasos/s (enteros)
        for i in range(len(v)):
            self.pasos[i] = round(v[i] * self.RELACION_PASOS_RAD)
            
        # Enviar comando
        packet = self.Encabezado + self.CMD_VEL
        packet += struct.pack(self.VelFormat, self.pasos[0], self.pasos[1])
        
        self.arduino.write(packet)
    
    def enviarPosArt(self, ang):
        
        # Convertir radianes a pasos (enteros)
        for i in range(len(ang)):
            self.angRad[i] = round(ang[i] * self.RELACION_PASOS_RAD)
            
        # Enviar comando
        packet = self.Encabezado + self.CMD_MOVE
        packet += struct.pack(self.MoveFormat, self.angRad[0]*4, self.angRad[1]*4)
        
        self.arduino.write(packet)
        
        while True:
            byte = self.arduino.read(1)
    
            if byte == self.Encabezado:
                break
        
        if self.arduino.read() == b'\x55':
            return True
        else:
            return False
        
    def detenerMot(self):
        self.arduino.write(b'\xAA\x06')
    
    def cerrarSerial(self):
        self.arduino.close()
        time.sleep(0.5)
        print("\nConexión cerrada")