import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial
import time

START_BYTE = 0xAA
END_BYTE   = 0x55
ID_ENCODERS = 0x02
ID_CMD_MOTOR_SERVO = 0x10

DIR_BACKWARD = 0
DIR_FORWARD  = 1
DIR_RELEASE  = 2

class ArduinoBridge(Node):
    def __init__(self):
        super().__init__('nodo_arduino_bridge')
        
        self.sub_cmd = self.create_subscription(String, '/robot_command', self.cmd_callback, 10)
        self.pub_encoders = self.create_publisher(String, '/encoders_raw', 10)

        self.dir_izq = DIR_RELEASE
        self.pwm_izq = 0
        self.dir_der = DIR_RELEASE
        self.pwm_der = 0
        
        # Ahora tenemos dos servos
        self.angulo_camara = 90
        self.angulo_lidar = 90
        self.paso_camara = 10
        self.velocidad_base = 100 # Ajustado a tu código (90)

        self.rx_state = 'WAIT_START'
        self.rx_id = 0
        self.rx_len = 0
        self.rx_checksum = 0
        self.rx_buffer = []
            
        # Configure Serial connection
        # IMPORTANT: Check if your Arduino is /dev/ttyUSB0 or /dev/ttyACM0
        try:
            self.ser = serial.Serial('/dev/ttyACM0', 115200, timeout=0.1)
            self.ser.reset_input_buffer()
            self.get_logger().info('Conexión Serial Establecida con Arduino')
        except Exception as e:
            self.get_logger().error(f'No se pudo conectar al Arduino: {e}')
            self.ser = None
            
        self.timer_leer = self.create_timer(0.01, self.leer_serial)
        self.timer_enviar = self.create_timer(0.1, self.enviar_paquete)

    def cmd_callback(self, msg):
        comando = msg.data.strip().lower()

        # MOVIMIENTO MOTORES
        if comando == 'w':
            self.dir_izq = DIR_FORWARD;  self.pwm_izq = self.velocidad_base
            self.dir_der = DIR_FORWARD;  self.pwm_der = self.velocidad_base
        elif comando == 's':
            self.dir_izq = DIR_BACKWARD; self.pwm_izq = self.velocidad_base
            self.dir_der = DIR_BACKWARD; self.pwm_der = self.velocidad_base
        elif comando == 'a':
            self.dir_izq = DIR_BACKWARD; self.pwm_izq = self.velocidad_base
            self.dir_der = DIR_FORWARD;  self.pwm_der = self.velocidad_base
        elif comando == 'd':
            self.dir_izq = DIR_FORWARD;  self.pwm_izq = self.velocidad_base
            self.dir_der = DIR_BACKWARD; self.pwm_der = self.velocidad_base
        elif comando == 'x' or comando == ' ':
            self.dir_izq = DIR_RELEASE;  self.pwm_izq = 0
            self.dir_der = DIR_RELEASE;  self.pwm_der = 0
            
        # SERVO CÁMARA
        elif comando == 'q':
            self.angulo_camara += self.paso_camara
            if self.angulo_camara > 180: self.angulo_camara = 180
        elif comando == 'e':
            self.angulo_camara -= self.paso_camara
            if self.angulo_camara < 0: self.angulo_camara = 0
            
        # SERVO LIDAR
        elif comando.startswith('p') and len(comando) > 1:
            try:
                nuevo_angulo = int(comando[1:])
                if 0 <= nuevo_angulo <= 180:
                    self.angulo_lidar = nuevo_angulo
            except:
                pass

    def enviar_paquete(self):
        if not self.ser: return

        # AHORA SON 6 DATOS (Añadido angulo_lidar al final)
        data = [
            self.dir_izq, self.pwm_izq, 
            self.dir_der, self.pwm_der, 
            self.angulo_camara, self.angulo_lidar
        ]
        
        length = len(data)
        checksum = ID_CMD_MOTOR_SERVO ^ length
        for b in data:
            checksum ^= b
            
        paquete = bytes([START_BYTE, ID_CMD_MOTOR_SERVO, length] + data + [checksum, END_BYTE])
        self.ser.write(paquete)

    def leer_serial(self):
        if not self.ser: return
        
        while self.ser.in_waiting > 0:
            byte_in = self.ser.read(1)[0]
            
            if self.rx_state == 'WAIT_START':
                if byte_in == START_BYTE: self.rx_state = 'READ_ID'
            elif self.rx_state == 'READ_ID':
                self.rx_id = byte_in; self.rx_checksum = byte_in; self.rx_state = 'READ_LEN'
            elif self.rx_state == 'READ_LEN':
                self.rx_len = byte_in; self.rx_checksum ^= byte_in; self.rx_buffer = []
                if self.rx_len > 20: self.rx_state = 'WAIT_START'
                else: self.rx_state = 'READ_DATA'
            elif self.rx_state == 'READ_DATA':
                self.rx_buffer.append(byte_in); self.rx_checksum ^= byte_in
                if len(self.rx_buffer) >= self.rx_len: self.rx_state = 'READ_CHK'
            elif self.rx_state == 'READ_CHK':
                if byte_in == self.rx_checksum: self.rx_state = 'WAIT_END'
                else: self.rx_state = 'WAIT_START'
            elif self.rx_state == 'WAIT_END':
                if byte_in == END_BYTE: self.procesar_paquete(self.rx_id, self.rx_buffer)
                self.rx_state = 'WAIT_START'

    def procesar_paquete(self, id_paquete, data):
        if id_paquete == ID_ENCODERS and len(data) == 4:
            fi, fd, ti, td = data[0], data[1], data[2], data[3]
            msg = String()
            msg.data = f"FI:{fi} FD:{fd} TI:{ti} TD:{td}"
            self.pub_encoders.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    nodo = ArduinoBridge()
    rclpy.spin(nodo)
    nodo.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
