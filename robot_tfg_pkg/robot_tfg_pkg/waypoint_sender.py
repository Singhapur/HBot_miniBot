import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool

class WaypointSender(Node):
    def __init__(self):
        super().__init__('waypoint_sender')

        # Goal publisher (simulates the "2D Goal Pose" button in RViz)
        self.pub_goal = self.create_publisher(PoseStamped, '/goal_pose', 10)

        # Subscriber to the goal reached event
        self.sub_status = self.create_subscription(Bool, '/goal_reached', self.status_callback, 10)

        # Format: (X in meters, Y in meters, Final heading angle in DEGREES)
        self.waypoints = [
            (2.0, 0.0, 90.0),    # Waypoint 1
            (0.0, 1.0, 90.0),    # Waypoint 2
            (-2.0, 0.0, 0.0)     # Waypoint 3
        ]

        self.current_wp_index = 0
        self.first_sent = False

        self.get_logger().info('Waypoint Sender Ready! Waiting 2 seconds to start the route...')

    def euler_to_quaternion(self, roll, pitch, yaw):
        qx = math.sin(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) - \
             math.cos(roll / 2) * math.sin(pitch / 2) * math.sin(yaw / 2)

        qy = math.cos(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2) + \
             math.sin(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2)

        qz = math.cos(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2) - \
             math.sin(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2)

        qw = math.cos(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) + \
             math.sin(roll / 2) * math.sin(pitch / 2) * math.sin(yaw / 2)

        return qx, qy, qz, qw

    def send_first_goal(self):
        if not self.first_sent:
            self.send_current_goal()
            self.first_sent = True

    def status_callback(self, msg):
        if msg.data:
            self.get_logger().info(
                f'Waypoint {self.current_wp_index} REACHED!'
            )

            # Move to the next waypoint
            self.current_wp_index += 1

            if self.current_wp_index < len(self.waypoints):
                self.get_logger().info('Sending next waypoint...')
                self.send_current_goal()
            else:
                self.get_logger().info('ALL WAYPOINTS COMPLETED! The route is finished.')


    def send_current_goal(self):
        # Extract data from the current waypoint
        x, y, theta_deg = self.waypoints[self.current_wp_index]

        # Convert degrees to radians and then to a quaternion (RViz2 format)
        theta_rad = math.radians(theta_deg)
        qx, qy, qz, qw = self.euler_to_quaternion(0.0, 0.0, theta_rad)

        # Build the message as if it were a click in RViz2
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'

        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)

        msg.pose.orientation.x = qx
        msg.pose.orientation.y = qy
        msg.pose.orientation.z = qz
        msg.pose.orientation.w = qw

        self.get_logger().info(f'Dispatching Waypoint {self.current_wp_index}: 'f'(X: {x}, Y: {y}, Yaw: {theta_deg}°)')

        self.pub_goal.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = WaypointSender()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()