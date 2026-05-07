import cv2
import rclpy
import numpy as np
import mediapipe as mp
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from mediapipe.tasks import python
from sensor_msgs.msg import CompressedImage


class CameraReader(Node):
    def __init__(self):
        super().__init__('vision_processing_node')
        
        # 1. Subscriber: Receives the camera image
        self.subscription = self.create_subscription(CompressedImage, '/image_compressed_raw', self.listener_callback, 10)
        
        # 2. Publishers: Send results to the rest of the robot
        self.publisher_img = self.create_publisher(CompressedImage, '/image_compressed_processed', 10)
        
        # 3. MediaPipe configuration
        models_path = "/home/hp/ros2_tfg/models/"
        
        BaseOptions = python.BaseOptions(model_asset_path=models_path + "pose_landmarker_lite.task")
        VisionRunningMode = mp.tasks.vision.RunningMode
        
        # Initialize Pose
        PoseLandmarker = mp.tasks.vision.PoseLandmarker
        PoseLandmarkerOptions = mp.tasks.vision.PoseLandmarkerOptions
        pose_options = PoseLandmarkerOptions(
            base_options=BaseOptions,
            running_mode=VisionRunningMode.VIDEO,
            num_poses=5
        )
        self.pose_landmarker = PoseLandmarker.create_from_options(pose_options)

    # --- MAIN LOOP (Callback) ---
    def listener_callback(self, msg):
        try:
            # 1. Convert ROS Compress Image to OpenCV
            np_arr = np.frombuffer(msg.data, np.uint8)
            frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            #frame = cv2.flip(frame, 1)

            # 2. Prepare image for MediaPipe
            rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb)
            
            timestamp_ms = int(self.get_clock().now().nanoseconds / 1000000)

            # 3. Detect Pose
            pose_result = self.pose_landmarker.detect_for_video(mp_image, timestamp_ms)
            
            if pose_result.pose_landmarks:
                for landmarks in pose_result.pose_landmarks:
                    xs = [int(lm.x * frame.shape[1]) for lm in landmarks]
                    ys = [int(lm.y * frame.shape[0]) for lm in landmarks]

                    x_min, x_max = min(xs), max(xs)
                    y_min, y_max = min(ys), max(ys)

                    # --- MARGIN CORRECTION ---
                    margin = 20
                    y_min = max(0, y_min - margin)
                    y_max = min(frame.shape[0], y_max + margin)

                    cv2.rectangle(frame, (x_min, y_min), (x_max, y_max), (0, 0, 255), 2)

            # 4. Show result
            cv2.imshow("Vision Processing", frame)
            cv2.waitKey(1)
            
            # Publish processed image to view it on another PC if it's necessary 
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

        except Exception as e:
            self.get_logger().error(f'Error processing image: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = CameraReader()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
