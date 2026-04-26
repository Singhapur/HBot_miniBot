import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import math

class OdometriaNode(Node):
    def __init__(self):
        super().__init__('nodo_odometria')

        # --- PARÁMETROS FÍSICOS ---
        self.rueda_diametro = 0.066  
        self.distancia_ejes = 0.145  
        self.ticks_por_vuelta = 20.0
        
        self.circunferencia = math.pi * self.rueda_diametro
        self.metros_por_tick = self.circunferencia / self.ticks_por_vuelta

        # Estado del robot
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        self.sentido = 0.0 # 1.0 = adelante, -1.0 = atrás, 0.0 = parado

        # Suscriptor a los encoders
        self.sub_encoders = self.create_subscription(String, '/encoders_raw', self.enc_callback, 10)
        
        # NUEVO: Suscriptor al comando para saber la dirección
        self.sub_comando = self.create_subscription(String, '/robot_command', self.cmd_callback, 10)
        
        # Publicadores
        self.pub_odom = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.get_logger().info('Odometría con detección de sentido (W/S) iniciada')

    def cmd_callback(self, msg):
        comando = msg.data.lower()
        if comando == 'w':
            self.sentido = 1.0
        elif comando == 's':
            self.sentido = -1.0
        elif comando in ['x', ' ']:
            self.sentido = 0.0
        # Si es un giro (a/d), el centro del robot no suele avanzar mucho, 
        # pero para simplificar podemos dejar el sentido que tuviera o ponerlo a 1.
        elif comando in ['a', 'd']:
            self.sentido = 1.0 

    def euler_a_cuaternion(self, roll, pitch, yaw):
        qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
        qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
        qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        return qx, qy, qz, qw

    def enc_callback(self, msg):
        try:
            partes = msg.data.split(' ')
            fi = int(partes[0].split(':')[1])
            fd = int(partes[1].split(':')[1])
            ti = int(partes[2].split(':')[1])
            td = int(partes[3].split(':')[1])

            # Convertir ticks a distancia y APLICAR EL SENTIDO
            dist_izq = ((ti) / 2.0) * self.metros_por_tick * self.sentido
            dist_der = ((td) / 2.0) * self.metros_por_tick * self.sentido

            # Cinemática diferencial
            dist_centro = (dist_der + dist_izq) / 2.0
            
            # El giro (delta_th) no depende del sentido adelante/atrás, 
            # sino de la diferencia entre ruedas.
            delta_th = (dist_der - dist_izq) / self.distancia_ejes

            # Actualizar posición
            self.x += dist_centro * math.cos(self.th)
            self.y += dist_centro * math.sin(self.th)
            self.th += delta_th

            self.publicar_odometria()

        except Exception as e:
            pass

    def publicar_odometria(self):
        current_time = self.get_clock().now().to_msg()
        qx, qy, qz, qw = self.euler_a_cuaternion(0, 0, self.th)

        t = TransformStamped()
        t.header.stamp = current_time
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.rotation.x = qx
        t.transform.rotation.y = qy
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw
        self.tf_broadcaster.sendTransform(t)

        odom = Odometry()
        odom.header.stamp = current_time
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        self.pub_odom.publish(odom)

def main(args=None):
    rclpy.init(args=args)
    nodo = OdometriaNode()
    rclpy.spin(nodo)
    nodo.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
