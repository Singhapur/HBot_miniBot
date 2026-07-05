import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
import numpy as np


class CameraSubscriber(Node):
    def __init__(self):
        super().__init__('camera_subscriber_node')
        
        # 1. Publishers: Send results to the rest of the robot
        self.subscription = self.create_subscription(CompressedImage, '/image_publish_compressed', self.listener_callback, 10)
        
    def listener_callback(self, msg):
        np_arr = np.frombuffer(msg.data, np.uint8)
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        
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
