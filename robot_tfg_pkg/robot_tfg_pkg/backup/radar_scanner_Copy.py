import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range, LaserScan
from std_msgs.msg import String
import math

class RadarScanner(Node):
    def __init__(self):
        super().__init__('nodo_radar_scanner')
        
        # Subscription
        self.sub_distancia = self.create_subscription(Range, '/sensor_distancia', self.dist_callback, 10)
        
        # Subscription to keyboard to read the 'L' command
        self.sub_teclado = self.create_subscription(String, '/robot_command', self.cmd_callback, 10)
        
        # Publishers
        self.pub_cmd = self.create_publisher(String, '/robot_command', 10)
        self.pub_scan = self.create_publisher(LaserScan, '/scan', 10)
        
        # Configuration variables
        self.paso_grados = 2       # 2 degrees for a good balance between speed and detail
        self.tiempo_paso = 0.1     # 100ms per step
        self.ultima_distancia = float('inf')
        self.num_lecturas = int(180 / self.paso_grados) + 1
        self.rangos = [float('inf')] * self.num_lecturas
        
        # RViz memory
        self.ultimo_scan_msg = None
        
        # State machine: 'IDLE' (Rest), 'PREPARING' (Moving to 0), 'SCANNING' (Sweeping)
        self.estado = 'IDLE'
        self.angulo_actual = 90
        self.contador_espera = 0
        
        # Node general timer
        self.timer = self.create_timer(self.tiempo_paso, self.timer_callback)

    def dist_callback(self, msg):
        self.ultima_distancia = msg.range

    def cmd_callback(self, msg):
        # If 'L' is pressed and the radar is idle, start the sequence
        if msg.data.lower() == 'l' and self.estado == 'IDLE':
            self.get_logger().info('¡Iniciando Escaneo de Entorno!')
            self.estado = 'PREPARANDO'
            
            # Send the servo to 0 degrees immediately
            self.angulo_actual = 0
            msg_cmd = String()
            msg_cmd.data = "P0"
            self.pub_cmd.publish(msg_cmd)
            
            # Calculate how long to wait for the servo to reach 0 before measuring
            # (1 second is enough to go from 90 to 0)
            self.contador_espera = int(1.0 / self.tiempo_paso)

    def timer_callback(self):
        if self.estado == 'IDLE':
            # KEEP POINTS IN RVIZ: Continuously re-publish the last map
            if self.ultimo_scan_msg:
                # Only update the timestamp so RViz does not delete it as "old"
                self.ultimo_scan_msg.header.stamp = self.get_clock().now().to_msg()
                self.pub_scan.publish(self.ultimo_scan_msg)
                
        elif self.estado == 'PREPARANDO':
            # Wait for the servo to mechanically reach position 0
            self.contador_espera -= 1
            if self.contador_espera <= 0:
                self.estado = 'ESCANEANDO'
                self.rangos = [float('inf')] * self.num_lecturas # Clear previous map
                
                
        elif self.estado == 'ESCANEANDO':
            # Store the distance
            indice = int(self.angulo_actual / self.paso_grados)
            if 0 <= indice < len(self.rangos):
                self.rangos[indice] = self.ultima_distancia
                
            # Move to the next angle
            self.angulo_actual += self.paso_grados
            
            # Have we finished the scan
            if self.angulo_actual > 180:
                self.generar_y_guardar_scan()
                self.get_logger().info('Escaneo finalizado. Volviendo al centro.')
                
                # Return to center
                msg_cmd = String()
                msg_cmd.data = "P90"
                self.pub_cmd.publish(msg_cmd)
                
                self.estado = 'IDLE' # Volvemos a reposo
            else:
                # Otherwise, keep sending the next angle to Arduino
                msg_cmd = String()
                msg_cmd.data = f"P{self.angulo_actual}"
                self.pub_cmd.publish(msg_cmd)

    def generar_y_guardar_scan(self):
        scan = LaserScan()
        scan.header.stamp = self.get_clock().now().to_msg()
        scan.header.frame_id = "laser_frame"
        
        scan.angle_min = 0.0
        scan.angle_max = math.pi 
        scan.angle_increment = math.radians(self.paso_grados)
        scan.time_increment = self.tiempo_paso
        scan.scan_time = self.tiempo_paso * self.num_lecturas
        scan.range_min = 0.0 # 0 cm
        scan.range_max = 8.0 # 8 metros
        scan.ranges = self.rangos
        
        # Store it in memory and publish it for the first time
        self.ultimo_scan_msg = scan
        self.pub_scan.publish(scan)

def main(args=None):
    rclpy.init(args=args)
    nodo = RadarScanner()
    rclpy.spin(nodo)
    nodo.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
