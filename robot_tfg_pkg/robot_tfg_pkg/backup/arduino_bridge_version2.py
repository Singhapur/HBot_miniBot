import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
import serial
import math
from std_msgs.msg import Int32

# --- CONSTANTES PROTOCOLO BINARIO ---
START_BYTE = 0xAA
END_BYTE   = 0x55
ID_ENCODERS = 0x02
ID_CMD_MOTOR_SERVO = 0x10

DIR_BACKWARD = 0
DIR_FORWARD  = 1
DIR_RELEASE  = 2

class ArduinoBridge(Node):
    def __init__(self):
        super().__init__('arduino_bridge')
        
        # 1. SUSCRIPTOR ESTÁNDAR DE ROS (/cmd_vel)
        self.sub_cmd_vel = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        
        # 2. PUBLICADOR ESTÁNDAR DE ROS (/joint_states)
        self.pub_joints = self.create_publisher(JointState, '/joint_states', 10)
        
        # 3. SUSCRIPTORES PARA LOS SERVOS
        self.sub_camara = self.create_subscription(Int32, '/set_camera_angle', self.camara_callback, 10)
        self.sub_lidar = self.create_subscription(Int32, '/set_lidar_angle', self.lidar_callback, 10)
        
        # Variables de estado enviadas al Arduino
        self.dir_izq = DIR_RELEASE
        self.pwm_izq = 0
        self.dir_der = DIR_RELEASE
        self.pwm_der = 0
        self.angulo_camara = 90
        self.angulo_lidar = 90

        # Parámetros físicos del robot para la cinemática
        self.distancia_ejes = 0.145  # Metros (L)
        self.max_velocidad_ms = 0.5  # Asumimos que a PWM 255 va a 0.5 m/s (ajústalo)
        
        # Variables comunicación serial
        self.rx_state = 'WAIT_START'
        self.rx_id = 0
        self.rx_len = 0
        self.rx_checksum = 0
        self.rx_buffer = []

        try:
            self.ser = serial.Serial('/dev/ttyACM0', 115200, timeout=0.01)
            self.get_logger().info('Arduino Bridge Estándar (Twist/JointState) Iniciado')
        except Exception as e:
            self.get_logger().error(f'Error de conexión: {e}')
            self.ser = None

        self.timer_leer = self.create_timer(0.01, self.leer_serial)
        self.timer_enviar = self.create_timer(0.1, self.enviar_paquete)

    # ========================================================
    # CINEMÁTICA INVERSA (De Twist a PWM)
    # ========================================================
    def cmd_vel_callback(self, msg):
        v = msg.linear.x  # Velocidad hacia adelante/atrás (m/s)
        w = msg.angular.z # Velocidad de giro (rad/s)

        # 1. Amplificador de giro (Si ves que le cuesta girar, sube este 2.0 a 3.0)
        factor_giro = 2.0 
        
        # 2. Calcular velocidad teórica de cada lado (m/s)
        v_izq = v - (w * factor_giro * self.distancia_ejes / 2.0)
        v_der = v + (w * factor_giro * self.distancia_ejes / 2.0)

        # 3. Convertir velocidad (m/s) a PWM (0-255)
        pwm_i = int(abs(v_izq) * (255.0 / self.max_velocidad_ms))
        pwm_d = int(abs(v_der) * (255.0 / self.max_velocidad_ms))

        # --- NUEVO: COMPENSACIÓN DE ZONA MUERTA (DEADBAND) ---
        # Si el cálculo pide moverse pero la potencia es muy baja, 
        # le damos un "empujón" mínimo de 90 PWM para vencer la fricción.
        MIN_PWM = 95
        if pwm_i > 0 and pwm_i < MIN_PWM:
            pwm_i = MIN_PWM
        if pwm_d > 0 and pwm_d < MIN_PWM:
            pwm_d = MIN_PWM

        # Limitar a 255 máximo (por seguridad)
        self.pwm_izq = min(255, pwm_i)
        self.pwm_der = min(255, pwm_d)

        # 4. Determinar dirección física de los pines
        if v_izq > 0.01:
            self.dir_izq = DIR_FORWARD
        elif v_izq < -0.01:
            self.dir_izq = DIR_BACKWARD
        else:
            self.dir_izq = DIR_RELEASE
            self.pwm_izq = 0 # Apagar del todo

        if v_der > 0.01:
            self.dir_der = DIR_FORWARD
        elif v_der < -0.01:
            self.dir_der = DIR_BACKWARD
        else:
            self.dir_der = DIR_RELEASE
            self.pwm_der = 0 # Apagar del todo

    # ========================================================
    # ENVÍO AL ARDUINO (Protocolo Binario)
    # ========================================================
    def enviar_paquete(self):
        if not self.ser: return

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

    # ========================================================
    # RECEPCIÓN DEL ARDUINO Y PUBLICACIÓN DE JOINTSTATE
    # ========================================================
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
            
            # Crear mensaje estándar de ROS para las juntas (ruedas)
            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.name = ['front_left_wheel', 'front_right_wheel', 'rear_left_wheel', 'rear_right_wheel']
            
            # Por ahora, mapearemos los "ticks crudos" al array de velocity.
            # Más adelante en la odometría, convertiremos esto a radianes/segundo reales.
            msg.velocity = [float(fi), float(fd), float(ti), float(td)]
            
            self.pub_joints.publish(msg)
            
    def camara_callback(self, msg):
        angulo = max(0, min(180, msg.data)) # Limitar entre 0 y 180
        self.angulo_camara = angulo

    def lidar_callback(self, msg):
        angulo = max(0, min(180, msg.data))
        self.angulo_lidar = angulo

def main(args=None):
    rclpy.init(args=args)
    nodo = ArduinoBridge()
    rclpy.spin(nodo)
    nodo.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
