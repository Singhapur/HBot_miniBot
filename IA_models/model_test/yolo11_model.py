import cv2
import time
import numpy as np
from ultralytics import YOLO

def detect_people():
    # 1. Initialize YOLO11 Pose Model
    yolo_model = YOLO("yolo11n-pose.pt")
    
    # 2. Initialize the local webcam
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("Error: Could not open webcam.")
        return
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 256)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 256)

    print("Starting detection... Press 'q' in the window to exit.")

    while cap.isOpened():
        ret, frame = cap.read()
        if not ret:
            break

        start_yolo = time.time()
        results_yolo = yolo_model.predict(frame, verbose=False)
        end_yolo = time.time()
        time_yolo_ms = (end_yolo - start_yolo) * 1000

        # --- KEYPOINT DRAWING ---
        # Draw YOLO11 Pose points (Orange)
        if len(results_yolo) > 0 and results_yolo[0].keypoints is not None:
            # keypoints.xy contains the pixel coordinate matrices for each person
            for kp_set in results_yolo[0].keypoints.xy:
                for kp in kp_set:
                    x, y = int(kp[0]), int(kp[1])
                    if x > 0 and y > 0: # Avoid drawing undetected points (0,0)
                        cv2.circle(frame, (x, y), 4, (0, 165, 255), -1)

        # --- SIDE DATA PANEL ---
        # Create a black background on the right side to display metrics cleanly
        panel = np.zeros((frame.shape[0], 256, 3), dtype=np.uint8)

        # YOLO11 Texts (Orange)
        cv2.putText(panel, "YOLO11 Pose (Nano):", (15, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 165, 255), 2)
        cv2.putText(panel, f"Inference: {time_yolo_ms:.1f} ms", (15, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
        fps_yolo = 1000 / time_yolo_ms if time_yolo_ms > 0 else 0
        cv2.putText(panel, f"Model FPS: {fps_yolo:.1f}", (15, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)

        # Color legend
        cv2.putText(panel, "LEGEND:", (15, 140), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)
        cv2.circle(panel, (25, 180), 6, (0, 165, 255), -1)
        cv2.putText(panel, "YOLO11 Points", (40, 185), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

        # Horizontally concatenate the camera frame with the data panel
        final_interface = cv2.hconcat([frame, panel])

        # Show the window on screen
        cv2.imshow("TFG: YOLO11 Pose Evaluation", final_interface)

        # Exit with the 'q' key
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # Clean up and release resources
    cap.release()
    cv2.destroyAllWindows()
    print("Modules closed successfully.")

if __name__ == "__main__":
    detect_people()