import cv2
import rclpy
import numpy as np
import mediapipe as mp
from rclpy.node import Node
from cv_bridge import CvBridge
from std_msgs.msg import Int32
from std_srvs.srv import Trigger
from sensor_msgs.msg import Image
from mediapipe.tasks import python
from geometry_msgs.msg import Twist
from sensor_msgs.msg import CompressedImage

class CameraReader(Node):
    def __init__(self):
        super().__init__('vision_processing_node')
        
        # 1. Subscriber: Receives the camera image
        self.subscription = self.create_subscription(CompressedImage, '/image_compressed_raw', self.listener_callback, 10)
        
        # 2. Publishers: Send results to the rest of the robot
        self.publisher_img = self.create_publisher(CompressedImage, '/image_compressed_processed', 15)
        self.publisher_servo = self.create_publisher(Int32, '/set_camera_angle', 10)
        self.pub_cmd_vel = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # 3. MediaPipe configuration
        models_path = "/home/hp/ros2_tfg/models/"

        # 4. Create service for follow a person
        self.srv = self.create_service(Trigger, '/trigger_follow', self.trigger_scan_callback)
        self.follow = False

        # Servo state
        self.current_servo_angle = 90
        self.last_servo_send_time = self.get_clock().now()
        self.move_step = 2
        self.error_margin = 15
        self.kp_speed = 0.015
        
        BaseOptions = python.BaseOptions(model_asset_path=models_path + "pose_landmarker_full.task")
        VisionRunningMode = mp.tasks.vision.RunningMode
        
        # Initialize Pose
        PoseLandmarker = mp.tasks.vision.PoseLandmarker
        PoseLandmarkerOptions = mp.tasks.vision.PoseLandmarkerOptions
        pose_options = PoseLandmarkerOptions(
            base_options=BaseOptions,
            running_mode=VisionRunningMode.VIDEO,
            num_poses=1
        )
        self.pose_landmarker = PoseLandmarker.create_from_options(pose_options)

    def trigger_scan_callback(self, request, response):
        if self.follow:
            self.follow = False
            response.success = False
            response.message = "Follow a person service has been stoped"
            self.pub_cmd_vel.publish(Twist()) # Stop robot for safty 
            return response
        else:
            self.follow = True
            response.success = True
            response.message = "Follow a person service has been started"
            return response

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

            step = self.move_step # Grados a mover por cada frame
            cmd = Twist() # Create basic speed msg from zero
            person_detected = False
            
            if pose_result.pose_landmarks:
                for landmarks in pose_result.pose_landmarks:
                    xs = [int(lm.x * frame.shape[1]) for lm in landmarks]
                    ys = [int(lm.y * frame.shape[0]) for lm in landmarks]

                    x_min, x_max = min(xs), max(xs)
                    y_min, y_max = min(ys), max(ys)

                    person_detected = True # We have detected a person

                    # Follow logic
                    if self.follow:
                        # Folow a person
                        person_center_x = (x_min + x_max) / 2
                        image_center_x = 128 # Half of 256
                        error_x = image_center_x - person_center_x

                        if abs(error_x) > self.error_margin:
                            # If the error is positive, the person is on the left -> we increase angle
                            # If the error is negative, it’s on the right -> we decrease angle
                            
                            if error_x > 0:
                                self.current_servo_angle += step
                            else:
                                self.current_servo_angle -= step

                            self.current_servo_angle = max(0, min(180, self.current_servo_angle))
                            # Publish new angle
                            servo_msg = Int32()
                            servo_msg.data = int(self.current_servo_angle)
                            self.publisher_servo.publish(servo_msg)

                        error_servo = self.current_servo_angle - 90
                        if abs(error_servo) > self.error_margin:
                            # Proportional constant (Kp) for rotation.
                            cmd.angular.z = error_servo * self.kp_speed
                            
                        # Move towards the person: We use the person's height as a distance proxy
                        bbox_height = y_max - y_min
                        
                        # If the person occupies less than 180 pixels in height (far away), move forward
                        if bbox_height < 180:
                            cmd.linear.x = 0.15 # Linear approach velocity
                        else:
                            cmd.linear.x = 0.0 # Close enough

                    # --- MARGIN CORRECTION ---
                    margin = 20
                    y_min = max(0, y_min - margin)
                    y_max = min(frame.shape[0], y_max + margin)

                    cv2.rectangle(frame, (x_min, y_min), (x_max, y_max), (0, 0, 255), 2)

            else:
                # Publish home angle
                if self.current_servo_angle != 90:
                    if self.current_servo_angle > 90:
                        self.current_servo_angle -= step
                        if self.current_servo_angle < 90: 
                            self.current_servo_angle = 90
                    elif self.current_servo_angle < 90:
                        self.current_servo_angle += step
                        if self.current_servo_angle > 90:
                            self.current_servo_angle = 90
                servo_msg = Int32()
                servo_msg.data = int(self.current_servo_angle)
                self.publisher_servo.publish(servo_msg)

            # Publish vel comando     
            if self.follow:
                if not person_detected:
                    self.pub_cmd_vel.publish(Twist())
                else:
                    self.pub_cmd_vel.publish(cmd)



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
