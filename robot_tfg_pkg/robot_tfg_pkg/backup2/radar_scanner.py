import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from std_msgs.msg import Int32, Header
from sensor_msgs.msg import Range, LaserScan, PointCloud2
from sensor_msgs_py import point_cloud2
import math

class RadarScanner(Node):
    def __init__(self):
        super().__init__('smart_radar')
        
        # 1. SERVICE: When someone calls /get_scan, self.start_scan_callback will execute
        self.srv = self.create_service(Trigger, '/get_scan', self.start_scan_callback)
        
        # 2. PUBLISHER TO ARDUINO (To move the servo)
        self.pub_servo = self.create_publisher(Int32, '/set_lidar_angle', 10)
        
        # 3. SUBSCRIBER TO ESP32 (To read the laser)
        self.sub_lidar = self.create_subscription(Range, '/sensor_distancia', self.distance_callback, 10)
        
        # 4. PUBLISHER FOR THE FINAL MAP
        self.pub_scan = self.create_publisher(LaserScan, '/scan', 10)
        
        # 5. PUBLISHER FOR CLOUD POINT
        self.pub_pc2 = self.create_publisher(PointCloud2, '/cloud_in', 10)

        # Scanning state variables
        self.is_scanning = False
        self.current_angle = 0
        self.step = 2  # Degrees to advance in each step (Adjust to 5 or 10)
        self.measured_distances = []
        self.last_distance = 0.0

        # Timer that runs the radar state machine
        self.timer = self.create_timer(0.05, self.scan_loop)

        self.get_logger().info('Smart Radar ready. Waiting for /get_scan service call.')

    def distance_callback(self, msg):
        # Save the latest distance read from the ESP32
        self.last_distance = msg.range

    def start_scan_callback(self, request, response):
        if not self.is_scanning:
            self.get_logger().info('Starting radar sweep from 0 to 180...')
            self.is_scanning = True
            self.current_angle = 0
            self.measured_distances = []
            
            # --- Save the actual start time ---
            self.scan_start_time = self.get_clock().now().to_msg()
            
            response.success = True
            response.message = "Sweep started successfully."
        else:
            response.success = False
            response.message = "Radar is already scanning, please wait."
            
        return response

    def scan_loop(self):
        # If we are not scanning, do nothing
        if not self.is_scanning:
            return

        # 1. Move the servo to the current angle
        msg_servo = Int32()
        msg_servo.data = self.current_angle
        self.pub_servo.publish(msg_servo)

        # 2. Save the distance read at this angle
        self.measured_distances.append(self.last_distance)

        # 3. Advance to the next angle
        self.current_angle += self.step

        # 4. Check if we have finished the 180-degree sweep
        if self.current_angle > 180:
            self.publish_maps()
            self.is_scanning = False
            
            # Return the servo to the center position (90)
            msg_servo.data = 90
            self.pub_servo.publish(msg_servo)
            self.get_logger().info('Sweep finished. Map published to /scan')

    def publish_maps(self):
        scan = LaserScan()
        # --- Use the start time, not the current time ---
        scan.header.stamp = self.scan_start_time 
        scan.header.frame_id = 'tf_luna_link'
        
        # --- GEOMETRIC CORRECTION ---
        # Center the arc in front of the robot (-90 degrees to +90 degrees)
        scan.angle_min = -math.pi / 2.0                      
        scan.angle_max = math.pi / 2.0                  
        scan.angle_increment = math.radians(self.step)
        
        # --- RESTORE REAL TIME INCREMENT ---
        scan.time_increment = 0.05                
        
        scan.range_min = 0.0
        scan.range_max = 8.0
        
        # Equalize the array length to match expected measurements
        expected_measurements = int((math.pi / scan.angle_increment)) + 1
        measured = self.measured_distances
        if len(measured) > expected_measurements:
            measured = measured[:expected_measurements]
        elif len(measured) < expected_measurements:
            measured.extend([float('inf')] * (expected_measurements - len(measured)))
            
        scan.ranges = measured
        self.pub_scan.publish(scan)
        
        # Create PointCloud
        points = []
        current_angle_rad = scan.angle_min
        
        for r in scan.ranges:
            # Solo añadimos puntos que sean válidos (ni infinitos ni ceros raros)
            if r != float('inf') and scan.range_min < r < scan.range_max:
                # Trigonometría para pasar de coordenadas polares (distancia, ángulo) a cartesianas (X, Y)
                x = r * math.cos(current_angle_rad)
                y = r * math.sin(current_angle_rad)
                z = 0.0 # Como el radar barre en horizontal, la altura Z es 0 respecto al sensor
                
                points.append([x, y, z])
            
            current_angle_rad += scan.angle_increment
            
        # Creamos la cabecera (debe coincidir con el tiempo del escaneo y el frame del lidar)
        header = Header()
        header.stamp = self.scan_start_time
        header.frame_id = 'tf_luna_link'
        
        # Usamos la librería oficial de ROS 2 para empaquetar la lista de puntos
        pc2_msg = point_cloud2.create_cloud_xyz32(header, points)
        
        # Publicamos hacia OctoMap
        self.pub_pc2.publish(pc2_msg)

def main(args=None):
    rclpy.init(args=args)
    node = RadarScanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
