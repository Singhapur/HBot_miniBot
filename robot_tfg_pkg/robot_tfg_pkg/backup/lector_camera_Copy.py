import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import mediapipe as mp

# --- MAIN ROS 2 NODE ---
class LectorCamara(Node):
    def __init__(self):
        super().__init__('nodo_procesamiento_vision')
        
        # 1. Subscriber: Receives the camera image
        self.subscription = self.create_subscription(
            Image, '/image_raw', self.listener_callback, 5)
        
         # 2. Publishers: Send results to the rest of the robot
        self.publisher_img = self.create_publisher(Image, '/image_processed', 5)
        
        self.bridge = CvBridge()

        # 3. MediaPipe configuration (ABSOLUTE PATHS)
        # IMPORTANT: Change 'hp' to your actual username if different
        ruta_modelos = "/home/hp/ros2_tfg/models/"
        
        BaseOptions = mp.tasks.BaseOptions
        VisionRunningMode = mp.tasks.vision.RunningMode
        
        # Initialize Pose
        PoseLandmarker = mp.tasks.vision.PoseLandmarker
        PoseLandmarkerOptions = mp.tasks.vision.PoseLandmarkerOptions
        pose_options = PoseLandmarkerOptions(
            base_options=BaseOptions(model_asset_path=ruta_modelos + "pose_landmarker_lite.task"),
            running_mode=VisionRunningMode.IMAGE,
            num_poses=5
        )
        self.pose_landmarker = PoseLandmarker.create_from_options(pose_options)

    # --- MAIN LOOP (Callback) ---
    def listener_callback(self, msg):
        try:
            # 1. Convert ROS Image -> OpenCV
            frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            
            # Mirror (Flip) - Warning: this inverts left/right
            frame = cv2.flip(frame, 1)

            # 2. Prepare image for MediaPipe
            rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb)

            # 3. Detect Pose
            pose_result = self.pose_landmarker.detect(mp_image)
            
            if pose_result.pose_landmarks:
                for landmarks in pose_result.pose_landmarks:
                    xs = [int(lm.x * frame.shape[1]) for lm in landmarks]
                    ys = [int(lm.y * frame.shape[0]) for lm in landmarks]

                    # Calcular esquinas del rectángulo
                    x_min, x_max = min(xs), max(xs)
                    y_min, y_max = min(ys), max(ys)

                    # Añadir margen (por ejemplo, 20 píxeles)
                    margin = 150
                    y_min = max(0, y_min - margin)
                    y_max = min(frame.shape[0], y_max + margin)

                    # Dibujar rectangle
                    cv2.rectangle(frame, (x_min, y_min), (x_max, y_max), (0, 0, 255), 2)
                    # Draw pose (Simplified for this example)
                    #for lm in landmarks:
                    #    x = int(lm.x * frame.shape[1])
                    #    y = int(lm.y * frame.shape[0])
                    #    cv2.circle(frame, (x, y), 3, (0, 255, 0), -1)

            # 4. Show result and publish processed image
            cv2.imshow("Vision TFG", frame)
            cv2.waitKey(1)
            
            # (Optional) Publish processed image to view it on another PC
            # out_msg = self.bridge.cv2_to_imgmsg(frame, 'bgr8')
            # self.publisher_img.publish(out_msg)

        except Exception as e:
            self.get_logger().error(f'Error procesando imagen: {e}')

def main(args=None):
    rclpy.init(args=args)
    nodo = LectorCamara()
    rclpy.spin(nodo)
    nodo.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
