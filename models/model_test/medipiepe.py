import cv2
import time
import numpy as np
import mediapipe as mp

from mediapipe.tasks import python

def main():
    print("Initializing models... Please wait.")

    # 2. MediaPipe Pose Configuration and Initialization (Tasks API)
    # Using the exact path specified in your ROS2 code
    model_path = "/home/hp/ros2_tfg/models/pose_landmarker_full.task"
    
    base_options = python.BaseOptions(model_asset_path=model_path)
    PoseLandmarker = mp.tasks.vision.PoseLandmarker
    PoseLandmarkerOptions = mp.tasks.vision.PoseLandmarkerOptions
    VisionRunningMode = mp.tasks.vision.RunningMode
    pose_options = PoseLandmarkerOptions(
        base_options=base_options,
        running_mode=VisionRunningMode.VIDEO, # Optimized mode for camera streams
        num_poses=1
    )
    mediapipe_landmarker = PoseLandmarker.create_from_options(pose_options)

    # 3. Initialize the local webcam
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("Error: Could not open webcam.")
        return

    print("\nModels ready! Starting live comparison...")
    print("Press 'q' in the video window to exit.")

    while cap.isOpened():
        ret, frame = cap.read()
        if not ret:
            break

        # --- IMAGE PREPARATION ---
        # MediaPipe Tasks requires its own mp.Image object in RGB format
        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=frame_rgb)
        
        # Generate the timestamp in milliseconds required by RunningMode.VIDEO
        timestamp_ms = int(time.time() * 1000)

        # --- MEDIAPIPE EVALUATION (Tasks API) ---
        start_mp = time.time()
        pose_result_mp = mediapipe_landmarker.detect_for_video(mp_image, timestamp_ms)
        end_mp = time.time()
        tiempo_mp_ms = (end_mp - start_mp) * 1000

        # --- KEYPOINT DRAWING (SKELETONS) ---
        # 1. Draw MediaPipe points (Green)
        if pose_result_mp.pose_landmarks:
            for landmarks in pose_result_mp.pose_landmarks:
                for lm in landmarks:
                    # MediaPipe provides normalized coordinates (0 to 1), convert to pixels
                    cx = int(lm.x * frame.shape[1])
                    cy = int(lm.y * frame.shape[0])
                    cv2.circle(frame, (cx, cy), 4, (0, 255, 0), -1)

        # --- SIDE DATA PANEL ---
        # Create a black background on the right side to display metrics cleanly
        panel = np.zeros((frame.shape[0], 360, 3), dtype=np.uint8)

        # MediaPipe Texts (Green)
        cv2.putText(panel, "MediaPipe (Pose Tasks):", (15, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv2.putText(panel, f"Inference: {tiempo_mp_ms:.1f} ms", (15, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
        fps_mp = 1000 / tiempo_mp_ms if tiempo_mp_ms > 0 else 0
        cv2.putText(panel, f"Model FPS: {fps_mp:.1f}", (15, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)

        # Color legend
        cv2.putText(panel, "LEGEND:", (15, 380), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)
        cv2.circle(panel, (25, 410), 6, (0, 255, 0), -1)
        cv2.putText(panel, "MediaPipe Points", (40, 415), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

        # Horizontally concatenate the camera frame with the data panel
        interfaz_final = cv2.hconcat([frame, panel])

        # Show the window on screen
        cv2.imshow("TFG: MediaPipe Tasks vs YOLO11 Pose Comparison", interfaz_final)

        # Exit with the 'q' key
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # Clean up and release resources
    cap.release()
    cv2.destroyAllWindows()
    mediapipe_landmarker.close()
    print("Modules closed successfully.")

if __name__ == "__main__":
    main()
