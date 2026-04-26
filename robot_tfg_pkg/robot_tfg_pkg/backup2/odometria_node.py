import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
import math

class OdometryNode(Node):
    def __init__(self):
        super().__init__('odometry_node')

        # --- PHYSICAL PARAMETERS ---
        self.wheel_diameter = 0.066  # 66mm
        self.wheelbase = 0.240       # 240mm (L)
        self.ticks_per_rev = 20.0
        
        self.turn_correction = 0.85
        self.linear_correction = 0.40
        
        self.meters_per_tick = (math.pi * self.wheel_diameter) / self.ticks_per_rev

        # Global robot position
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0

        # Direction signs (based on /cmd_vel)
        self.sign_left = 1.0
        self.sign_right = 1.0

        # 1. SUBSCRIBER TO COMMANDS (To know direction of each wheel)
        self.sub_cmd = self.create_subscription(Twist, '/cmd_vel', self.cmd_callback, 10)
        
        # 2. SUBSCRIBER TO OFFICIAL ENCODERS
        self.sub_joints = self.create_subscription(JointState, '/joint_states', self.joint_callback, 10)
        
        # PUBLISHERS
        self.pub_odom = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.get_logger().info('Standard Odometry (JointState + Twist) started')

    def cmd_callback(self, msg):
        v = msg.linear.x
        w = msg.angular.z

        # Logic to determine the rotation direction of each side
        if v > 0.01:      # Moving forward
            self.sign_left = 1.0
            self.sign_right = 1.0
        elif v < -0.01:   # Moving backward
            self.sign_left = -1.0
            self.sign_right = -1.0
        elif w > 0.01:    # Turning left on the spot
            self.sign_left = -1.0
            self.sign_right = 1.0
        elif w < -0.01:   # Turning right on the spot
            self.sign_left = 1.0
            self.sign_right = -1.0
        else:             # Stopped
            self.sign_left = 0.0
            self.sign_right = 0.0

    def euler_to_quaternion(self, roll, pitch, yaw):
        qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
        qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
        qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        return qx, qy, qz, qw

    def joint_callback(self, msg):
        try:
            # In our bridge we pack: [fl, fr, rl, rr]
            # We use only the rear ones (indices 2 and 3) to avoid slipping errors
            rl_ticks = msg.velocity[2]
            rr_ticks = msg.velocity[3]

            # 1. Distance traveled by each wheel with its real sign
            dist_left = rl_ticks * self.meters_per_tick * self.sign_left
            dist_right = rr_ticks * self.meters_per_tick * self.sign_right

            # 2. Differential Kinematics (Odometry)
            dist_center = (dist_right + dist_left) / 2.0 * self.linear_correction
            delta_th = (dist_right - dist_left) / self.wheelbase

            # 3. Position Integration
            # We use self.th + delta_th/2 for better precision in curves (Runge-Kutta 2)
            self.x += dist_center * math.cos(self.th + delta_th / 2.0)
            self.y += dist_center * math.sin(self.th + delta_th / 2.0)
            self.th += delta_th

            self.publish_odometry()

        except Exception as e:
            pass

    def publish_odometry(self):
        current_time = self.get_clock().now().to_msg()
        qx, qy, qz, qw = self.euler_to_quaternion(0, 0, self.th)

        # TF Tree
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

        # /odom Message
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
    node = OdometryNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
