import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from std_msgs.msg import Int16MultiArray

class ControllerNode(Node):
    def __init__(self):
        super().__init__('controller_node')
        
        # 1. SUSCRIPTOR AL TECLADO
        self.sub_cmd_vel = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        
        # 2. SUSCRIPTOR A LOS ENCODERS (Para el PID)
        self.sub_joints = self.create_subscription(JointState, '/joint_states', self.joint_callback, 10)
        
        # 3. PUBLICADOR HACIA EL ARDUINO BRIDGE
        self.pub_pwm = self.create_publisher(Int16MultiArray, '/pwm_setpoints', 10)
        
        # Parámetros físicos
        self.distancia_ejes = 0.145
        self.max_velocidad_ms = 0.5
        
        # Variables de estado
        self.base_pwm_izq = 0
        self.base_pwm_der = 0
        self.dir_izq = 2 # 2=Release, 1=Forward, 0=Backward
        self.dir_der = 2
        self.modo_linea_recta = False
        self.ultima_pwm_izq = 0
        self.ultima_pwm_der = 0
        
        # Memoria para el término Integral
        self.error_acumulado = 0.0
        
        # Timer para enviar comandos de PWM constantemente
        self.timer = self.create_timer(0.1, self.publicar_pwm)
        
        self.get_logger().info('Controlador PID de Trayectoria Iniciado')

    def cmd_vel_callback(self, msg):
        v = msg.linear.x
        w = msg.angular.z

        # ¿Queremos ir en línea recta?
        if abs(v) > 0.01 and abs(w) < 0.01:
            self.modo_linea_recta = True
        else:
            self.modo_linea_recta = False
            self.error_acumulado = 0.0 # Borramos la memoria al girar

        factor_giro = 1.5 
        v_izq = v - (w * factor_giro * self.distancia_ejes / 2.0)
        v_der = v + (w * factor_giro * self.distancia_ejes / 2.0)

        pwm_i = int(abs(v_izq) * (255.0 / self.max_velocidad_ms))
        pwm_d = int(abs(v_der) * (255.0 / self.max_velocidad_ms))

        # Compensación de zona muerta
        MIN_PWM = 95
        if pwm_i > 0 and pwm_i < MIN_PWM: pwm_i = MIN_PWM
        if pwm_d > 0 and pwm_d < MIN_PWM: pwm_d = MIN_PWM

        # Guardamos la BASE del cálculo cinemático
        self.base_pwm_izq = min(255, pwm_i)
        self.base_pwm_der = min(255, pwm_d)

        # Direcciones
        if v_izq > 0.01: self.dir_izq = 1
        elif v_izq < -0.01: self.dir_izq = 0
        else: self.dir_izq = 2

        if v_der > 0.01: self.dir_der = 1
        elif v_der < -0.01: self.dir_der = 0
        else: self.dir_der = 2

    def joint_callback(self, msg):
        # Recibimos los ticks y aplicamos tu PID si vamos en línea recta
        if self.modo_linea_recta:
            ti = msg.velocity[2]
            td = msg.velocity[3]
            
            error = float(ti) - float(td)
            # ACUMULAMOS EL ERROR (Término Integral) ---
            self.error_acumulado += error
            
            # Limitamos la memoria para que no crezca hasta el infinito (Anti-Windup)
            self.error_acumulado = max(-50.0, min(50.0, self.error_acumulado))
            
            kp = 4.5
            ki = 1.0
            # La corrección ahora es: Presente (P) + Pasado (I)
            correccion = int((error * kp) + (self.error_acumulado * ki))
            
            # Aplicamos la corrección sobre la base
            if self.ultima_pwm_izq != 0 and self.ultima_pwm_der != 0:
                self.base_pwm_izq = self.ultima_pwm_izq - correccion
                self.base_pwm_der = self.ultima_pwm_der + correccion
            else:
                self.base_pwm_izq -= correccion
                self.base_pwm_der += correccion
            
            self.base_pwm_izq = max(95, min(255, self.base_pwm_izq))
            self.ultima_pwm_izq = self.base_pwm_izq       
            self.base_pwm_der = max(95, min(255, self.base_pwm_der))
            self.ultima_pwm_der = self.base_pwm_der

    def publicar_pwm(self):
        # Convertimos absoluto a valores firmados (positivos/negativos)
        val_izq = self.base_pwm_izq if self.dir_izq == 1 else -self.base_pwm_izq
        if self.dir_izq == 2: val_izq = 0
        
        val_der = self.base_pwm_der if self.dir_der == 1 else -self.base_pwm_der
        if self.dir_der == 2: val_der = 0
        
        msg_pwm = Int16MultiArray()
        msg_pwm.data = [val_izq, val_der]
        self.pub_pwm.publish(msg_pwm)

def main(args=None):
    rclpy.init(args=args)
    nodo = ControllerNode()
    rclpy.spin(nodo)
    nodo.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
