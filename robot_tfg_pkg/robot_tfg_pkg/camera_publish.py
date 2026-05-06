import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
import numpy as np


class CameraPublisher(Node):
    def __init__(self):
        super().__init__('camera_publisher_node')
        
        # 1. Declare the camera params
        self.cameraId = 0 # Id or Number of the camera
        self.camera = cv2.VideoCapture(self.cameraId)
        
        # 2. Publishers: Send results to the rest of the robot
        self.publisher_img = self.create_publisher(CompressedImage, '/image_publish_compressed', 10)
        
        # Communication period
        self.periodCommunication = 0.05
        
        # Very fast read timer to prevent losing bytes
        self.timer = self.create_timer(self.periodCommunication, self.timer_callbackFunction)
        
    def timer_callbackFunction(self):
        ret, frame = self.camera.read()
        if not ret:
            self.get_logger().warning("Failed to capture image")
            return
        target_width = 256
        target_height = 256
        frame = cv2.resize(frame, (target_width, target_height))
        
        encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 50]
        result, encimg = cv2.imencode('.jpg', frame, encode_param)

        if result:
            msg = CompressedImage()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.format = "jpeg"
            msg.data = np.array(encimg).tobytes()
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
