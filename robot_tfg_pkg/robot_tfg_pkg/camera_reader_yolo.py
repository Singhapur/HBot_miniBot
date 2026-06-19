import cv2
import rclpy
import numpy as np
from rclpy.node import Node
from ultralytics import YOLO
from std_msgs.msg import Int32
from std_srvs.srv import Trigger
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
        
        # 3. Create service for follow a person
        self.srv = self.create_service(Trigger, '/trigger_follow', self.trigger_scan_callback)
        self.follow = False

        # Servo and Control state
        self.current_servo_angle = 90
        self.last_servo_send_time = self.get_clock().now()
        
        self.move_step = 2      
        self.search_step = 2    
        
        self.error_margin = 15
        self.kp_speed = 0.015
        self.search_direction = 1
        
        # 4. Initialize YOLO11 Pose Model
        self.yolo_model = YOLO("yolo11n-pose.pt")
        self.get_logger().info('YOLO11 Pose Model Loaded successfully!')

    def trigger_scan_callback(self, request, response):
        if self.follow:
            self.follow = False
            response.success = False
            response.message = "Follow a person service has been stopped"
            self.pub_cmd_vel.publish(Twist()) # Stop robot for safety 
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

            # 2. Detect Pose with YOLO11
            results = self.yolo_model.predict(frame, verbose=False)

            cmd = Twist() 
            person_detected = False
            
            # Check if YOLO has detected at least one person
            if len(results) > 0 and len(results[0].boxes) > 0:
                person_detected = True 
                
                # YOLO gives the bounding box directly [x_min, y_min, x_max, y_max]
                # Take the first detected person (index 0)
                box = results[0].boxes[0].xyxy[0].cpu().numpy()
                x_min, y_min, x_max, y_max = int(box[0]), int(box[1]), int(box[2]), int(box[3])

                # Follow logic
                if self.follow:
                    # Person center using the box directly
                    person_center_x = (x_min + x_max) / 2
                    image_center_x = frame.shape[1] / 2 # Dynamic image center
                    error_x = image_center_x - person_center_x

                    if abs(error_x) > self.error_margin:
                        if error_x > 0:
                            self.current_servo_angle += self.move_step
                        else:
                            self.current_servo_angle -= self.move_step

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
                    
                    if bbox_height < 180:
                        cmd.linear.x = 0.15 # Linear approach velocity
                    else:
                        cmd.linear.x = 0.0 # Close enough

                # Draw the tracking box on the chassis in red
                cv2.rectangle(frame, (x_min, y_min), (x_max, y_max), (0, 0, 255), 2)
            
            else:
                if self.follow:
                    if not person_detected:
                        # --- Searching (Patrolling Mode) ---
                        cmd = Twist() 
                        
                        # The camera sweeps slowly from 0 to 180 degrees using search_step
                        self.current_servo_angle += self.search_direction * self.search_step
                        
                        if self.current_servo_angle >= 180:
                            self.current_servo_angle = 180
                            self.search_direction = -1 # Change direction to left
                        elif self.current_servo_angle <= 0:
                            self.current_servo_angle = 0
                            self.search_direction = 1  # Change direction to right

                        # Publish servo command for searching
                        servo_msg = Int32()
                        servo_msg.data = int(self.current_servo_angle)
                        self.publisher_servo.publish(servo_msg)
                        
                else:
                    # --- Idle Mode (Return to Center) ---
                    if self.current_servo_angle != 90:
                        if self.current_servo_angle > 90:
                            self.current_servo_angle -= self.move_step
                            if self.current_servo_angle < 90: 
                                self.current_servo_angle = 90
                        elif self.current_servo_angle < 90:
                            self.current_servo_angle += self.move_step
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
            cv2.imshow("Vision Processing YOLO11", frame)
            cv2.waitKey(1)
            
            # Publish processed image
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