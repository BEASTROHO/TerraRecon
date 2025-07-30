# ai_module/object_detection_utils.py

import cv2
import numpy as np

def preprocess_image(image, size=(640, 640)):
    """Resize and normalize image for YOLOv5 input."""
    image_resized = cv2.resize(image, size)
    image_rgb = cv2.cvtColor(image_resized, cv2.COLOR_BGR2RGB)
    image_normalized = image_rgb / 255.0
    return np.transpose(image_normalized, (2, 0, 1)).astype(np.float32)

def draw_detections(image, detections, class_names):
    """Draw bounding boxes and labels on the image."""
    for det in detections:
        x1, y1, x2, y2, conf, cls_id = det
        label = f"{class_names[int(cls_id)]} {conf:.2f}"
        cv2.rectangle(image, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
        cv2.putText(image, label, (int(x1), int(y1) - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
    return image

def postprocess_output(output, conf_threshold=0.5):
    """Filter detections based on confidence threshold."""
    detections = []
    for det in output:
        if det[4] >= conf_threshold:
            detections.append(det)
    return detections
