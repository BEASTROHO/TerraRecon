# ai_module/yolov5_inference.py

import torch
import cv2
import time
import os

class ObjectDetector:
    """
    Runs real-time object detection using YOLOv5 on camera feed.
    Designed for integration with TerraRecon's vision module.
    """

    def __init__(self, model_name='yolov5s', confidence_threshold=0.5):
        self.model = torch.hub.load('ultralytics/yolov5', model_name, pretrained=True)
        self.model.conf = confidence_threshold
        self.device = 'cuda' if torch.cuda.is_available() else 'cpu'
        self.model.to(self.device)

    def detect_from_camera(self, camera_id=0, log_path='detections.log'):
        cap = cv2.VideoCapture(camera_id)
        if not cap.isOpened():
            print("❌ Error: Camera not accessible.")
            return

        print("✅ Camera stream started. Press 'q' to quit.")
        with open(log_path, 'w') as log_file:
            while True:
                ret, frame = cap.read()
                if not ret:
                    print("⚠️ Frame capture failed.")
                    break

                results = self.model(frame)
                detections = results.pandas().xyxy[0]

                for _, row in detections.iterrows():
                    x1, y1, x2, y2 = int(row['xmin']), int(row['ymin']), int(row['xmax']), int(row['ymax'])
                    label = row['name']
                    conf = row['confidence']
                    cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    cv2.putText(frame, f"{label} {conf:.2f}", (x1, y1 - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                    log_file.write(f"{time.time()},{label},{conf:.2f},{x1},{y1},{x2},{y2}\n")

                cv2.imshow("TerraRecon Vision", frame)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break

        cap.release()
        cv2.destroyAllWindows()
        print("🛑 Camera stream ended.")

if __name__ == "__main__":
    detector = ObjectDetector()
    detector.detect_from_camera()
