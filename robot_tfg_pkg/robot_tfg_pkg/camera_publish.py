import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class CameraPublisher(Node):
    def __init__(self):
        super().__init__('camera_publisher_node')
        
        # 1. Declare the camera params
        self.cameraId = 0 # Id or Number of the camera
        self.camera = cv2.VideoCapture(self.cameraId)
        
        # 2. Publishers: Send results to the rest of the robot
        self.publisher_img = self.create_publisher(Image, '/image_publish_raw', 20)
        
        self.bridge = CvBridge()
        
        # Communication period
        self.periodCommunication = 0.01
        
        # Very fast read timer to prevent losing bytes
        self.timer = self.create_timer(self.periodCommunication, self.timer_callbackFunction)
        
    def timer_callbackFunction(self):
        ret, frame = self.camera.read()
        target_width = 256
        target_height = 256
        frame = cv2.resize(frame, (target_width, target_height))

        if ret:
            msg = self.bridge.cv2_to_imgmsg(frame)
            self.publisher_img.publish(msg)
        else:
            self.get_logger().warning("Failed to capture image")

def main(args=None):
    rclpy.init(args=args)
    node = CameraPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
