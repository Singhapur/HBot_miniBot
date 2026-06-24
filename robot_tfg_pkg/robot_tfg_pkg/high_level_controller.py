import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, PoseStamped

class HighLevelController(Node):
    def __init__(self):
        super().__init__('high_level_controller')
        
        # 1. SUBSCRIBERS
        self.sub_odom = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.sub_goal = self.create_subscription(PoseStamped, '/goal_pose', self.goal_callback, 10)
        self.pub_status = self.create_publisher(Bool, '/goal_reached', 10)
        
        # 2. PUBLISHER
        self.pub_cmd_vel = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # 3. STATE VARIABLES
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        
        self.goal_x = 0.0
        self.goal_y = 0.0
        self.goal_yaw = 0.0
        
        self.has_goal = False
        self.is_aligning = False
        
        # 4. CONTROLLER PARAMETERS (Adjustable)
        self.kp_linear = 0.5       # Base speed for moving forward
        self.kp_angular = 1.0      # Turning strength
        self.max_linear_vel = 0.3  # Max m/s
        self.max_angular_vel = 1.5 # Max rad/s
        
        self.distance_tolerance = 0.10 # Stops 10 cm from the goal
        self.yaw_tolerance = 0.15      # Final alignment tolerance (radians)
        
        # Control loop at 10 Hz
        self.timer = self.create_timer(0.1, self.control_loop)
        self.get_logger().info('High-Level Navigator Controller Started. Waiting for Goal in RViz2...')

    def quaternion_to_yaw(self, x, y, z, w):
        # Mathematical formula to extract the Z angle (Yaw) from a quaternion
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)

    def normalize_angle(self, angle):
        # Mathematical trick to keep the angle always between -Pi and Pi
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
        self.is_aligning = True
        
        self.get_logger().info(f'New Goal Received! -> X: {self.goal_x:.2f}, Y: {self.goal_y:.2f}')

    def control_loop(self):
        if not self.has_goal:
            return

        cmd = Twist()
        
        # 1. Calculate errors (Distance and Angle)
        dx = self.goal_x - self.current_x
        dy = self.goal_y - self.current_y
        
        distance_to_goal = math.sqrt(dx**2 + dy**2)
        angle_to_goal = math.atan2(dy, dx)
        
        # Heading error towards the goal point
        heading_error = self.normalize_angle(angle_to_goal - self.current_yaw)

        # 2. NAVIGATION LOGIC (State Machine)
        if distance_to_goal > self.distance_tolerance:
            # STATE A: Move towards the point
            
            if self.is_aligning:
                # Sub-state 1: PURE ALIGNMENT (Rotate in place)
                cmd.linear.x = 0.0
                cmd.angular.z = self.kp_angular * heading_error
                
                # If the error is less than 0.15 rad (~8.5 degrees), we are facing the goal. Move forward!
                if abs(heading_error) < 0.15:
                    self.is_aligning = False
                    
            else:
                # Sub-state 2: FORWARD MOTION (With gentle correction)
                cmd.linear.x = self.kp_linear * distance_to_goal
                cmd.angular.z = self.kp_angular * heading_error
                
                # HYSTERESIS (Anti Zig-Zag): Only stop and realign if the deviation is very large (> 22 degrees)
                if abs(heading_error) > 0.4:
                    self.is_aligning = True

        else:
            # STATE B: We reached point X,Y. Now align the final orientation.
            final_heading_error = self.normalize_angle(self.goal_yaw - self.current_yaw)
            
            if abs(final_heading_error) > self.yaw_tolerance:
                # Rotate on its axis to match the RViz arrow
                cmd.linear.x = 0.0
                cmd.angular.z = self.kp_angular * final_heading_error
            else:
                # GOAL REACHED!
                cmd.linear.x = 0.0
                cmd.angular.z = 0.0
                self.has_goal = False
                self.get_logger().info('Goal Reached Successfully!')
                
                # Publish a status message to indicate goal completion
                msg_status = Bool()
                msg_status.data = True
                self.pub_status.publish(msg_status)

        # 3. LIMIT VELOCITIES (Safety)
        cmd.linear.x = max(-self.max_linear_vel, min(self.max_linear_vel, cmd.linear.x))
        cmd.angular.z = max(-self.max_angular_vel, min(self.max_angular_vel, cmd.angular.z))

        # 4. Send commands to controller_node
        self.pub_cmd_vel.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = HighLevelController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
