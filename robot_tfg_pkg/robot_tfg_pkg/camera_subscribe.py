import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class CameraSubscriber(Node):
    def __init__(self):
        super().__init__('camera_subscriber_node')
        
        # 1. Publishers: Send results to the rest of the robot
        self.subscription = self.create_subscription(Image, '/image_publish_raw', self.listener_callback, 20)
        
        self.bridge = CvBridge()
        
    def listener_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg)
        
        cv2.imshow("Camera", frame)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = CameraSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
