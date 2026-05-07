import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from sensor_msgs.msg import JointState, Imu
from geometry_msgs.msg import Twist, TransformStamped

class OdometryNode(Node):
    def __init__(self):
        super().__init__('odometry_node')

        # --- PHYSICAL PARAMETERS ---
        self.wheel_radius = 0.066 / 2.0  
        self.wheel_base = 0.145     
        
        # --- CALIBRATION CONSTANTS ---
        self.linear_correction = 1.85 
        self.angular_correction = 0.85 

        # --- SENSOR FUSION ---
        self.use_imu = True  # Set to False to use only wheel encoders
        self.imu_yaw_rate = 0.0

        # Global robot position
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        self.v_linear = 0.0
        self.w_angular = 0.0

        # Timer for integral calculation (dt)
        self.last_time = self.get_clock().now()

        # Direction signs (based on /cmd_vel)
        self.left_sign = 1.0
        self.right_sign = 1.0

        # Subscribers and Publishers
        self.sub_cmd = self.create_subscription(Twist, '/cmd_vel', self.cmd_callback, 10)
        self.sub_joints = self.create_subscription(JointState, '/joint_states', self.joint_callback, 10)
        self.sub_imu = self.create_subscription(Imu, '/imu/data_raw', self.imu_callback, 10)
        
        self.pub_odom = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.get_logger().info('Odometry Node with IMU Fusion started')

    def cmd_callback(self, msg):
        v = msg.linear.x
        w = msg.angular.z

        # Logic to determine physical turning direction
        if v > 0.01:      
            self.left_sign = 1.0
            self.right_sign = 1.0
        elif v < -0.01:   
            self.left_sign = -1.0
            self.right_sign = -1.0
        elif w > 0.01:    
            self.left_sign = -1.0
            self.right_sign = 1.0
        elif w < -0.01:   
            self.left_sign = 1.0
            self.right_sign = -1.0
        else:             
            self.left_sign = 0.0
            self.right_sign = 0.0

    def imu_callback(self, msg):
        yaw_velocity = msg.angular_velocity.z
        
        # Noise Filter (Deadband): Cheap IMUs have some drift.
        # If the turning speed is less than 0.02 rad/s, we assume the robot is stationary
        # to prevent the map from slowly rotating when stopped.
        if abs(yaw_velocity) < 0.03:
            self.imu_yaw_rate = 0.0
        else:
            self.imu_yaw_rate = yaw_velocity

    def euler_to_quaternion(self, roll, pitch, yaw):
        qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
        qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
        qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        return qx, qy, qz, qw

    def joint_callback(self, msg):
        current_time = self.get_clock().now()
        dt = (current_time.nanoseconds - self.last_time.nanoseconds) / 1e9
        self.last_time = current_time

        if dt == 0 or dt > 0.5:
            return

        try:
            # 1. Get real rad/s from Arduino bridge, wheel order: [left, right]
            omega_left = abs(msg.velocity[2]) * self.left_sign
            omega_right = abs(msg.velocity[3]) * self.right_sign
            
            vl = (msg.velocity[2] * (self.wheel_radius)) 
            vr = (msg.velocity[3] * (self.wheel_radius))
            
            # Wheel velocity to robot linear and angular velocity 
            self.v_linear = (vl + vr)/2
            self.w_angular = (vr - vl)/(self.wheel_base)

            # 2. Distance traveled by each wheel
            dist_left = omega_left * self.wheel_radius * dt
            dist_right = omega_right * self.wheel_radius * dt

            # 3. Differential Kinematics (Odometry)
            # Straight forward movement is always provided by the wheels
            dist_center = ((dist_right + dist_left) / 2.0) * self.linear_correction
            
            # Add filter for Wheel Slip
            delta_th_imu = self.imu_yaw_rate * dt
            delta_th_enc = ((dist_right - dist_left) / self.wheel_base) * self.angular_correction
            
            if abs(delta_th_enc - delta_th_imu) > 0.05:
                dist_center = 0.0
            
            # --- SENSOR FUSION: Who calculates the turn? ---
            if self.use_imu:
                # Real turn (delta theta) is IMU velocity multiplied by time
                delta_th = self.imu_yaw_rate * dt
                # Note: Adjust sign to -self.imu_yaw_rate if RViz rotates in the opposite direction
            else:
                # Classic method (wheels only)
                delta_th = ((dist_right - dist_left) / self.wheel_base) * self.angular_correction

            # 4. Position Integration
            self.x += dist_center * math.cos(self.th + delta_th / 2.0)
            self.y += dist_center * math.sin(self.th + delta_th / 2.0)
            self.th += delta_th

            self.publish_odometry(current_time.to_msg())

        except Exception as e:
            self.get_logger().error(f'Odometry error: {e}')

    def publish_odometry(self, current_time):
        qx, qy, qz, qw = self.euler_to_quaternion(0, 0, self.th)

        # Publish TF
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

        # Publish Odometry Message
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
        # Set twist
        odom.twist.twist.linear.x = self.v_linear
        odom.twist.twist.angular.z = self.w_angular
        
        self.pub_odom.publish(odom)

def main(args=None):
    rclpy.init(args=args)
    node = OdometryNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
