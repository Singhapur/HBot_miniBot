import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from std_msgs.msg import Int32, Header
from sensor_msgs.msg import Range, PointCloud2
from sensor_msgs_py import point_cloud2
from rclpy.duration import Duration
import math

class RadarScanner(Node):
    def __init__(self):
        super().__init__('radar_scanner')
        
        # 1. SERVICE: Triggers a single high-precision scan
        self.srv = self.create_service(Trigger, '/trigger_scan', self.trigger_scan_callback)
        
        # 2. PUBLISHER TO ARDUINO (Move servo)
        self.pub_servo = self.create_publisher(Int32, '/set_lidar_angle', 10)
        
        # 3. SUBSCRIBER TO ESP32 (Read laser)
        self.sub_lidar = self.create_subscription(Range, '/sensor_distancia', self.distance_callback, 10)
        
        # 4. POINT CLOUD PUBLISHER
        self.pub_pc2 = self.create_publisher(PointCloud2, '/cloud_in', 10)

        # State variables
        self.is_scanning = False
        self.current_angle = 0
        self.step = 1  # 1 degree step for MAXIMUM resolution
        self.last_distance = -1.0
        self.points_buffer = []

        # Radar loop at 20Hz (0.05 seconds).
        self.timer = self.create_timer(0.05, self.scan_step)

        # Center the servo on startup
        self.center_servo()
        self.get_logger().info('Live Precision Radar ready. Call /trigger_scan to start.')

    def center_servo(self):
        msg = Int32()
        msg.data = 90
        self.pub_servo.publish(msg)

    def distance_callback(self, msg):
        self.last_distance = msg.range

    def trigger_scan_callback(self, request, response):
        if self.is_scanning:
            response.success = False
            response.message = "Radar is already scanning. Please wait for it to finish."
            return response

        # Start the photographic sweep
        self.is_scanning = True
        self.current_angle = 0
        self.points_buffer = []
        self.get_logger().info('Starting live precision sweep (0 to 180 degrees)...')
        
        response.success = True
        response.message = "Precision scan started."
        return response

    def scan_step(self):
        # If we haven't been asked to scan, do nothing
        if not self.is_scanning:
            return

        # 1. Move the servo to the current angle
        msg_servo = Int32()
        msg_servo.data = self.current_angle
        self.pub_servo.publish(msg_servo)
        
        # 2. Read the distance
        r = self.last_distance
        self.last_distance = -1.0 # Consume the data

        # 3. Save the point (Maximum 4 meters)
        if r != float('inf') and 0.0 < r <= 4.0:
            ros_angle_rad = math.radians(self.current_angle - 90)
            x = r * math.cos(ros_angle_rad)
            y = r * math.sin(ros_angle_rad)
            z = 0.0 
            self.points_buffer.append([x, y, z])

        # --- THE UPGRADE: CHUNKED SENDING ---
        # If we have accumulated 5 points, publish them immediately to see them in RViz
        if len(self.points_buffer) >= 5:
            self.publish_pointcloud(self.points_buffer)
            self.points_buffer = []  # Empty the buffer for the next 5 points

        # 4. Advance the angle
        self.current_angle += self.step

        # 5. Have we finished the 180-degree sweep?
        if self.current_angle > 180:
            self.is_scanning = False
            
            # Final cleanup: If there are leftover points in the buffer (e.g., only 2 or 3 points left)
            if len(self.points_buffer) > 0:
                self.publish_pointcloud(self.points_buffer)
                self.points_buffer = []
            
            # Stow the laser again
            self.center_servo()
            self.get_logger().info('Scan finished. Waiting for new command.')

    def publish_pointcloud(self, points):
        # If the list is empty, publish nothing
        if not points:
            return
            
        safe_time = (self.get_clock().now() - Duration(seconds=0.1)).to_msg()
        header = Header()
        header.stamp = safe_time
        header.frame_id = 'tf_luna_link'
        
        pc2_msg = point_cloud2.create_cloud_xyz32(header, points)
        self.pub_pc2.publish(pc2_msg)

def main(args=None):
    rclpy.init(args=args)
    node = RadarScanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
