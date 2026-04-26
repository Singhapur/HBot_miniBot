import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from std_msgs.msg import Int32, Header
from sensor_msgs.msg import Range, LaserScan, PointCloud2
from sensor_msgs_py import point_cloud2
from rclpy.duration import Duration
import math

class RadarScanner(Node):
    def __init__(self):
        super().__init__('smart_radar')
        
        # 1. SERVICE: When called, self.start_scan_callback will execute
        self.srv = self.create_service(Trigger, '/get_scan', self.toggle_scan_callback)
        
        # 2. PUBLISHER TO ARDUINO (To move the servo)
        self.pub_servo = self.create_publisher(Int32, '/set_lidar_angle', 10)
        
        # 3. SUBSCRIBER TO ESP32 (To read the laser)
        self.sub_lidar = self.create_subscription(Range, '/sensor_distancia', self.distance_callback, 10)
        
        # 4. PUBLISHER FOR POINT CLOUD
        self.pub_pc2 = self.create_publisher(PointCloud2, '/cloud_in', 10)

        # Scanning state variables
        self.is_scanning = False
        self.current_angle = 0
        self.direction = 1       # 1 for forward (0->180), -1 for backward (180->0)
        self.step = 2  # Degrees to advance in each step
        self.last_distance = 0.0
        
        # Buffer to store points and publish every 5 measurements
        self.points_buffer = []

        # Timer that runs the radar state machine
        self.timer = self.create_timer(0.05, self.scan_loop)

        self.get_logger().info('Continuous Radar ready. Call /get_scan to TOGGLE ON/OFF.')

    def distance_callback(self, msg):
        # Save the latest distance read from the ESP32
        self.last_distance = msg.range

    def toggle_scan_callback(self, request, response):
        # Toggle the scanning state (Turn ON if OFF, Turn OFF if ON)
        self.is_scanning = not self.is_scanning
        if self.is_scanning:
            self.get_logger().info('Radar STARTED: Continuous scanning mode.')
            response.message = "Radar Started."
        else:
            self.get_logger().info('Radar STOPPED.')
            response.message = "Radar Stopped."
            
            # Return the servo to the center position when stopped
            msg_servo = Int32()
            msg_servo.data = 90
            self.pub_servo.publish(msg_servo)
            
            # Flush any remaining points in the buffer
            if self.points_buffer:
                self.publish_pointcloud(self.points_buffer)
                self.points_buffer = []

        response.success = True
        return response

    def scan_loop(self):
        # If we are not scanning, do nothing
        if not self.is_scanning:
            return

        # 1. Move the servo to the current angle
        msg_servo = Int32()
        msg_servo.data = self.current_angle
        self.pub_servo.publish(msg_servo)
        
        # 2. Convert Servo Angle (0 to 180) to ROS Angle (-90 to +90 degrees)
        ros_angle_rad = math.radians(self.current_angle - 90)

        # 3. Read distance and convert polar to Cartesian (X, Y)
        r = self.last_distance
        # Solution for the ghost measurements 
        self.last_distance = -1.0

        # Only process valid measurements
        if r != float('inf') and 0.0 < r < 8.0:
            x = r * math.cos(ros_angle_rad)
            y = r * math.sin(ros_angle_rad)
            z = 0.0 
            self.points_buffer.append([x, y, z])

        # 4. PUBLISH CHUNK: If we have 5 points, send them to RViz/OctoMap
        if len(self.points_buffer) >= 5:
            self.publish_pointcloud(self.points_buffer)
            self.points_buffer = []  # Reset buffer

        # 5. Advance to the next angle
        self.current_angle += (self.step * self.direction)

        # 6. PING-PONG LOGIC: Bounce at the edges
        if self.current_angle >= 180:
            self.current_angle = 180
            self.direction = -1  # Reverse direction
            
        elif self.current_angle <= 0:
            self.current_angle = 0
            self.direction = 1   # Forward direction

    def publish_pointcloud(self, points):
        # --- TIME TRICK (Avoid extrapolation into the future) ---
        safe_time = (self.get_clock().now() - Duration(seconds=0.1)).to_msg()
        
        # Create header matching scan time and lidar frame
        header = Header()
        header.stamp = safe_time
        header.frame_id = 'tf_luna_link'
        
        # Package point cloud using official ROS 2 library
        pc2_msg = point_cloud2.create_cloud_xyz32(header, points)
        
        # Publish to OctoMap
        self.pub_pc2.publish(pc2_msg)

def main(args=None):
    rclpy.init(args=args)
    node = RadarScanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
