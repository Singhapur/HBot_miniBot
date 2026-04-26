import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Int16MultiArray
import numpy as np

# ========================================================
# CLASE PID (Sustituye a mini_bot.utils.pid)
# ========================================================
class PID:
    def __init__(self, kp, ki, kd, integral_max):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral_max = integral_max
        
        self.integral = 0.0
        self.prev_error = 0.0

    def update(self, error, dt):
        if dt <= 0.0:
            return 0.0
            
        # Proporcional
        p_term = self.kp * error
        
        # Integral con anti-windup (Límite de saturación)
        self.integral += error * dt
        self.integral = max(-self.integral_max, min(self.integral_max, self.integral))
        i_term = self.ki * self.integral
        
        # Derivativo
        derivative = (error - self.prev_error) / dt
        d_term = self.kd * derivative
        
        self.prev_error = error
        return p_term + i_term + d_term

# ========================================================
# NODO PRINCIPAL DEL CONTROLADOR
# ========================================================
class ControllerNode(Node):
    def __init__(self):
        super().__init__('controller_node')
        
        # Parámetros físicos (¡Asegúrate de que coinciden con los de odometría_node!)
        self.declare_parameter('wheel_radius', 0.033) # 66mm diametro / 2 = 0.033
        self.declare_parameter('wheel_base', 0.145)   # Distancia entre ejes L
        self.declare_parameter('max_delta_pwm', 25.0)
        
        # Ganancias PID (Tendrás que afinarlas probando el robot)
        self.declare_parameter('v_pid.kp', 5.0)
        self.declare_parameter('v_pid.ki', 0.1)
        self.declare_parameter('v_pid.kd', 0.05)
        self.declare_parameter('v_pid.integral_max', 0.2)
        
        self.declare_parameter('w_pid.kp', 5.0)
        self.declare_parameter('w_pid.ki', 0.1)
        self.declare_parameter('w_pid.kd', 0.05)
        self.declare_parameter('w_pid.integral_max', 0.2)
        
        self.declare_parameter('dt', 0.05) # 50 ms loop
        
        # Look-Up Table (La tabla mágica del profesor para saltarse la zona muerta)
        self.declare_parameter('velocity_to_pwm_lut_left', 
            [-6.0, -255.0, -3.0, -225.0, -2.0, -200.0, -1.0, -100.0, 0.0, 0.0, 1.0, 100.0, 2.0, 200.0, 3.5, 225.0, 6.5, 255.0])
        self.declare_parameter('velocity_to_pwm_lut_right', 
            [-6.0, -255.0, -3.0, -225.0, -2.0, -200.0, -1.0, -100.0, 0.0, 0.0, 1.0, 100.0, 2.0, 200.0, 3.5, 225.0, 6.5, 255.0])

        # Cargar variables
        self.wheel_radius = self.get_parameter('wheel_radius').get_parameter_value().double_value
        self.wheel_base = self.get_parameter('wheel_base').get_parameter_value().double_value
        self.dt = self.get_parameter('dt').get_parameter_value().double_value
        
        lut_param_left = self.get_parameter('velocity_to_pwm_lut_left').get_parameter_value().double_array_value
        self.velocity_to_pwm_lut_left = np.array(lut_param_left).reshape(-1, 2)
        lut_param_right = self.get_parameter('velocity_to_pwm_lut_right').get_parameter_value().double_array_value
        self.velocity_to_pwm_lut_right = np.array(lut_param_right).reshape(-1, 2)

        # Estados de velocidad actual (Feedback de Odometría)
        self.current_v = 0.0
        self.current_w = 0.0
        self.last_velocity_time = self.get_clock().now()

        # Instrucciones de velocidad deseadas (Setpoint de /cmd_vel)
        self.desired_v = 0.0
        self.desired_w = 0.0
        self.desired_last_time = self.get_clock().now()

        # Instanciar Controladores PID
        self.v_pid = PID(
            self.get_parameter('v_pid.kp').value, self.get_parameter('v_pid.ki').value,
            self.get_parameter('v_pid.kd').value, self.get_parameter('v_pid.integral_max').value)
        self.w_pid = PID(
            self.get_parameter('w_pid.kp').value, self.get_parameter('w_pid.ki').value,
            self.get_parameter('w_pid.kd').value, self.get_parameter('w_pid.integral_max').value)
            
        self.last_time = self.get_clock().now()

        # Publicadores y Suscriptores
        self.pwm_pub = self.create_publisher(Int16MultiArray, '/pwm_setpoints', 10)
        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        
        # Bucle de Control Principal a 20Hz (0.05s)
        self.create_timer(self.dt, self.apply_velocity)
        
        self.get_logger().info("Controlador de Navegación PID Activo")

    def cmd_vel_callback(self, msg: Twist):
        self.desired_v = msg.linear.x
        self.desired_w = msg.angular.z
        self.desired_last_time = self.get_clock().now()

    def odom_callback(self, msg: Odometry):
        # El feedback real: a qué velocidad se está moviendo realmente el robot
        self.current_v = msg.twist.twist.linear.x
        self.current_w = msg.twist.twist.angular.z
        self.last_velocity_time = self.get_clock().now()

    def apply_velocity(self):
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9
        self.last_time = now

        # PERRO GUARDÍAN (Watchdog): Si hace 1 segundo que no recibo comandos, freno por seguridad
        if (now - self.desired_last_time).nanoseconds / 1e9 > 1.0:
            self.desired_v = 0.0
            self.desired_w = 0.0
        
        # Feed-forward base
        v_feed_forward = self.desired_v
        w_feed_forward = self.desired_w

        # APLICAR PID: Comparar lo deseado con lo real
        if (now - self.last_velocity_time).nanoseconds / 1e9 > 0.2:
            # Si perdemos la odometría, no usamos el PID porque sería peligroso
            v_correction = 0.0
            w_correction = 0.0
        else:
            # Calcular el error
            v_error = v_feed_forward - self.current_v
            w_error = w_feed_forward - self.current_w

            # Obtener corrección matemática
            v_correction = self.v_pid.update(v_error, dt)
            w_correction = self.w_pid.update(w_error, dt)

        # Velocidad compensada (Lo que queremos + lo que el PID dice para compensar el derrape)
        v = v_feed_forward + v_correction
        w = w_feed_forward + w_correction

        # Cinemática Diferencial a radianes/segundo
        vr_rads = ((2 * v) + (w * self.wheel_base)) / (2 * self.wheel_radius)
        vl_rads = ((2 * v) - (w * self.wheel_base)) / (2 * self.wheel_radius)

        # Tabla de Búsqueda (Magia para motores TT): Mapear Rad/s a PWM
        left_pwm = np.interp(vl_rads, self.velocity_to_pwm_lut_left[:, 0], self.velocity_to_pwm_lut_left[:, 1])
        right_pwm = np.interp(vr_rads, self.velocity_to_pwm_lut_right[:, 0], self.velocity_to_pwm_lut_right[:, 1])

        # Asegurar freno total si no se pulsa nada
        if self.desired_v == 0.0 and self.desired_w == 0.0:
            left_pwm = 0.0
            right_pwm = 0.0

        self.send_pwm_command(int(left_pwm), int(right_pwm))

    def send_pwm_command(self, left_pwm, right_pwm):
        pwm_msg = Int16MultiArray()
        pwm_msg.data = [int(left_pwm), int(right_pwm)]
        self.pwm_pub.publish(pwm_msg)

def main(args=None):
    rclpy.init(args=args)
    node = ControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
