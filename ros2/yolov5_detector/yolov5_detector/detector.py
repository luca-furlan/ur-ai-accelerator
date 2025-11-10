import cv2
import numpy as np
from typing import List, Tuple


class Detection:
    __slots__ = (
        'class_id',
        'class_name',
        'confidence',
        'bbox',  # (x_min, y_min, width, height)
        'center',  # (x_center, y_center)
    )

    def __init__(self, class_id: int, class_name: str, confidence: float, bbox: Tuple[int, int, int, int]):
        self.class_id = class_id
        self.class_name = class_name
        self.confidence = confidence
        self.bbox = bbox
        x, y, w, h = bbox
        self.center = (x + w / 2.0, y + h / 2.0)


class YoloV5Detector:
    def __init__(
        self,
        weights_path: str,
        class_names: List[str],
        input_width: int = 640,
        input_height: int = 640,
        conf_threshold: float = 0.4,
        iou_threshold: float = 0.45,
        use_cuda: bool = False,
    ) -> None:
        self.input_width = input_width
        self.input_height = input_height
        self.conf_threshold = conf_threshold
        self.iou_threshold = iou_threshold
        self.class_names = class_names

        self.net = cv2.dnn.readNetFromONNX(weights_path)
        if use_cuda:
            self.net.setPreferableBackend(cv2.dnn.DNN_BACKEND_CUDA)
            self.net.setPreferableTarget(cv2.dnn.DNN_TARGET_CUDA_FP16)
        else:
            self.net.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)
            self.net.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)

    def infer(self, frame):
        """Run inference on a BGR frame."""
        if frame is None:
            return []

        blob = cv2.dnn.blobFromImage(
            frame,
            scalefactor=1 / 255.0,
            size=(self.input_width, self.input_height),
            mean=(0, 0, 0),
            swapRB=True,
            crop=False,
        )
        self.net.setInput(blob)
        outputs = self.net.forward()

        if isinstance(outputs, tuple) or isinstance(outputs, list):
            outputs = outputs[0]

        # outputs shape: (1, num_detections, 85)
        detections = []
        rows = outputs.shape[0]
        if outputs.ndim == 3:
            rows = outputs.shape[1]
            outputs = outputs[0]

        frame_height, frame_width = frame.shape[:2]
        x_factor = frame_width / self.input_width
        y_factor = frame_height / self.input_height

        boxes = []
        confidences = []
        class_ids = []

        for i in range(rows):
            detection = outputs[i]
            objectness = float(detection[4])
            if objectness < self.conf_threshold:
                continue

            scores = detection[5:]
            class_id = int(np.argmax(scores))
            class_score = float(scores[class_id])
            confidence = float(objectness * class_score)
            if confidence < self.conf_threshold:
                continue

            cx, cy, w, h = detection[0:4]
            left = int((cx - 0.5 * w) * x_factor)
            top = int((cy - 0.5 * h) * y_factor)
            width = int(w * x_factor)
            height = int(h * y_factor)
            if width <= 0 or height <= 0:
                continue

            boxes.append([left, top, width, height])
            confidences.append(confidence)
            class_ids.append(class_id)

        if not boxes:
            return []

        indices = cv2.dnn.NMSBoxes(boxes, confidences, self.conf_threshold, self.iou_threshold)
        if len(indices) == 0:
            return []

        results = []
        for idx in indices.flatten():
            x, y, w, h = boxes[idx]
            x = max(0, x)
            y = max(0, y)
            w = min(w, frame_width - x)
            h = min(h, frame_height - y)
            class_id = class_ids[idx]
            class_name = self.class_names[class_id] if 0 <= class_id < len(self.class_names) else str(class_id)
            results.append(
                Detection(
                    class_id=class_id,
                    class_name=class_name,
                    confidence=float(confidences[idx]),
                    bbox=(x, y, w, h),
                )
            )

        return results

    @staticmethod
    def draw_detections(frame, detections: List[Detection]):
        for det in detections:
            x, y, w, h = det.bbox
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
            label = f"{det.class_name}: {det.confidence:.2f}"
            cv2.putText(frame, label, (x, max(0, y - 10)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
        return frame
