import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
import math

class HighLevelController(Node):
    def __init__(self):
        super().__init__('high_level_controller')
        
        # 1. SUSCRIPCIONES
        self.sub_odom = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.sub_goal = self.create_subscription(PoseStamped, '/goal_pose', self.goal_callback, 10)
        
        # 2. PUBLICADOR
        self.pub_cmd_vel = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # 3. VARIABLES DE ESTADO
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        
        self.goal_x = 0.0
        self.goal_y = 0.0
        self.goal_yaw = 0.0
        self.has_goal = False
        
        # 4. PARÁMETROS DEL CONTROLADOR (Ajustables)
        self.kp_linear = 0.5       # Velocidad base para avanzar
        self.kp_angular = 1.0      # Fuerza de giro
        self.max_linear_vel = 0.3  # m/s máximos
        self.max_angular_vel = 0.5 # rad/s máximos
        
        self.distance_tolerance = 0.10 # Se detiene a 10 cm del objetivo
        self.yaw_tolerance = 0.15      # Tolerancia de alineación final (radianes)
        
        # Bucle de control a 10 Hz
        self.timer = self.create_timer(0.1, self.control_loop)
        self.get_logger().info('High-Level Navigator Controller Started. Waiting for Goal in RViz2...')

    def quaternion_to_yaw(self, x, y, z, w):
        # Fórmula matemática para extraer el ángulo Z (Yaw) de un cuaternión
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)

    def normalize_angle(self, angle):
        # Truco matemático para mantener el ángulo siempre entre -Pi y Pi
        return math.atan2(math.sin(angle), math.cos(angle))

    def odom_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self.current_yaw = self.quaternion_to_yaw(q.x, q.y, q.z, q.w)

    def goal_callback(self, msg):
        self.goal_x = msg.pose.position.x
        self.goal_y = msg.pose.position.y
        q = msg.pose.orientation
        self.goal_yaw = self.quaternion_to_yaw(q.x, q.y, q.z, q.w)
        
        self.has_goal = True
        self.get_logger().info(f'New Goal Received! -> X: {self.goal_x:.2f}, Y: {self.goal_y:.2f}')

    def control_loop(self):
        if not self.has_goal:
            return

        cmd = Twist()
        
        # 1. Calcular errores (Distancia y Ángulo)
        dx = self.goal_x - self.current_x
        dy = self.goal_y - self.current_y
        
        distance_to_goal = math.sqrt(dx**2 + dy**2)
        angle_to_goal = math.atan2(dy, dx)
        
        # Error de ángulo hacia el punto objetivo
        heading_error = self.normalize_angle(angle_to_goal - self.current_yaw)

        # 2. LÓGICA DE NAVEGACIÓN (Máquina de Estados)
        if distance_to_goal > self.distance_tolerance:
            # ESTADO A: Ir hacia el punto
            
            if abs(heading_error) > 0.3:
                # Sub-estado 1: El robot está mirando a Cuenca. Primero rotar sobre sí mismo.
                cmd.linear.x = 0.0
                cmd.angular.z = self.kp_angular * heading_error
            else:
                # Sub-estado 2: Ya mira al objetivo, avanzar y corregir suavemente
                cmd.linear.x = self.kp_linear * distance_to_goal
                cmd.angular.z = self.kp_angular * heading_error
                
        else:
            # ESTADO B: Ya hemos llegado al punto X,Y. Ahora hay que alinear la orientación final.
            final_heading_error = self.normalize_angle(self.goal_yaw - self.current_yaw)
            
            if abs(final_heading_error) > self.yaw_tolerance:
                # Rotar sobre sí mismo para calcar la flecha de RViz
                cmd.linear.x = 0.0
                cmd.angular.z = self.kp_angular * final_heading_error
            else:
                # ¡OBJETIVO ALCANZADO!
                cmd.linear.x = 0.0
                cmd.angular.z = 0.0
                self.has_goal = False
                self.get_logger().info('Goal Reached Successfully!')

        # 3. LIMITAR VELOCIDADES (Seguridad)
        cmd.linear.x = max(-self.max_linear_vel, min(self.max_linear_vel, cmd.linear.x))
        cmd.angular.z = max(-self.max_angular_vel, min(self.max_angular_vel, cmd.angular.z))

        # 4. Enviar comandos al controller_node
        self.pub_cmd_vel.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = HighLevelController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
