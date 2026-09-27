"""Lazy Ultralytics segmentation adapter.

Ultralytics is deliberately imported at runtime. ROS package discovery and
unit tests therefore still work on robot images where the optional inference
environment has not been installed yet.
"""

from dataclasses import dataclass
from typing import List

import cv2
import numpy as np


@dataclass(frozen=True)
class Segmentation:
    mask: np.ndarray
    score: float
    class_id: int
    class_name: str
    xyxy: np.ndarray


class YoloSegmentationBackend:
    def __init__(self, weights: str, confidence: float = 0.35, device: str = "cpu"):
        try:
            from ultralytics import YOLO
        except ImportError as error:
            raise RuntimeError(
                "ultralytics is not installed; run scripts/install_bottle_yolo.sh "
                "or select detector_backend:=color for the diagnostic-only fallback"
            ) from error
        self._model = YOLO(weights)
        self._confidence = float(confidence)
        self._device = device
        names = self._model.names
        items = names.items() if isinstance(names, dict) else enumerate(names)
        self._bottle_class_ids = [
            int(class_id) for class_id, class_name in items
            if str(class_name).strip().lower() == "bottle"
        ]
        if not self._bottle_class_ids:
            raise RuntimeError(f"YOLO weights contain no 'bottle' class: {weights}")

    def infer(self, bgr_image: np.ndarray) -> List[Segmentation]:
        results = self._model.predict(
            source=bgr_image,
            conf=self._confidence,
            device=self._device,
            verbose=False,
            classes=self._bottle_class_ids,
        )
        detections = []
        height, width = bgr_image.shape[:2]
        for result in results:
            if result.boxes is None:
                continue
            boxes = result.boxes
            polygons = result.masks.xy if result.masks is not None else []
            names = result.names
            for index in range(len(boxes)):
                class_id = int(boxes.cls[index].item())
                score = float(boxes.conf[index].item())
                xyxy = boxes.xyxy[index].detach().cpu().numpy()
                mask = np.zeros((height, width), dtype=np.uint8)
                if index < len(polygons) and len(polygons[index]) >= 3:
                    polygon = np.rint(polygons[index]).astype(np.int32)
                    cv2.fillPoly(mask, [polygon], 255)
                else:
                    x1, y1, x2, y2 = np.rint(xyxy).astype(int)
                    mask[max(0, y1):min(height, y2), max(0, x1):min(width, x2)] = 255
                detections.append(
                    Segmentation(mask.astype(bool), score, class_id, str(names[class_id]), xyxy)
                )
        return detections


class ColorBottleBackend:
    """Red-object diagnostic fallback, never presented as a YOLO result."""

    def __init__(self, minimum_area: int = 300):
        self.minimum_area = int(minimum_area)

    def infer(self, bgr_image: np.ndarray) -> List[Segmentation]:
        hsv = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, (0, 80, 45), (12, 255, 255))
        mask |= cv2.inRange(hsv, (168, 80, 45), (180, 255, 255))
        count, labels, stats, _ = cv2.connectedComponentsWithStats(mask)
        detections = []
        for label in range(1, count):
            x, y, width, height, area = stats[label]
            if area < self.minimum_area:
                continue
            component = labels == label
            detections.append(
                Segmentation(
                    component,
                    1.0,
                    39,
                    "bottle-diagnostic-color",
                    np.array([x, y, x + width, y + height], dtype=np.float32),
                )
            )
        return detections
