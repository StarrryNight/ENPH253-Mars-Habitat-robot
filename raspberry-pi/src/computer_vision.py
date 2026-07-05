from dataclasses import dataclass, field

import cv2
import numpy as np
from ultralytics import YOLO

MODEL_PATH = "model_ncnn_model"  # directory containing .param and .bin files

CONF = 0.25
IOU = 0.7
IMGSZ = 640


@dataclass
class Detection:
    label: str
    confidence: float
    cx: int          # pixel x of bounding box center
    cy: int          # pixel y of bounding box center
    # positive = target is right of frame center, negative = left.
    # Caller uses this to generate an omega correction for the drive command.
    bearing_deg: float
    is_new_teletubby: bool = False


class TeletubbySensor:
    def __init__(self):
        self.detected: list[float] = []  # dominant hues of confirmed teletubbies

    def get_dominant_hue(self, frame, bbox: tuple[int, int, int, int]) -> float | None:
        x, y, w, h = bbox
        roi = frame[y:y+h, x:x+w]
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        # Only look at saturated pixels — ignore grey/white
        mask = cv2.inRange(hsv, np.array([0, 50, 50]), np.array([180, 255, 255]))
        hues = hsv[:, :, 0][mask > 0]
        if len(hues) == 0:
            return None
        return float(np.median(hues))

    def is_new(self, hue: float, threshold: int = 15) -> bool:
        # TODO: hue wraps at 180 in OpenCV — use circular distance:
        #   min(abs(hue - recorded), 180 - abs(hue - recorded))
        return all(abs(hue - recorded) >= threshold for recorded in self.detected)

    def record(self, frame, bbox: tuple[int, int, int, int]) -> bool:
        """Returns True if this is a new unique teletubby."""
        hue = self.get_dominant_hue(frame, bbox)
        if hue is None:
            return False
        if self.is_new(hue):
            self.detected.append(hue)
            return True
        return False

    def maxed_out(self) -> bool:
        return len(self.detected) >= 2


class ComputerVision:
    def __init__(
        self, camera_index: int = 0, frame_width: int = 640, frame_height: int = 480
    ):
        self.frame_width = frame_width
        self.frame_height = frame_height

        self.cap = cv2.VideoCapture(camera_index)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, frame_width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, frame_height)
        if not self.cap.isOpened():
            raise RuntimeError(f"Could not open camera {camera_index}")

        self.model = YOLO(MODEL_PATH, task="detect")
        self.teletubby_sensor = TeletubbySensor()

    def capture(self) -> Detection | None:
        """Grab a frame, run local NCNN inference, return highest-confidence detection or None.
        Sets is_new_teletubby=True only when the detection is a colour not seen before."""
        ok, frame = self.cap.read()
        if not ok:
            return None

        results = self.model(frame, conf=CONF, iou=IOU, imgsz=IMGSZ, verbose=False)

        boxes = results[0].boxes
        if boxes is None or len(boxes) == 0:
            return None

        best_idx = int(boxes.conf.argmax())
        x1, y1, x2, y2 = [int(v) for v in boxes.xyxy[best_idx].tolist()]
        conf  = float(boxes.conf[best_idx])
        label = results[0].names[int(boxes.cls[best_idx])]

        cx = (x1 + x2) // 2
        cy = (y1 + y2) // 2

        # TODO: measure actual camera FOV and update hfov_deg
        hfov_deg = 60.0
        bearing_deg = ((cx - self.frame_width / 2) / self.frame_width) * hfov_deg

        # bbox as (x, y, w, h) for TeletubbySensor
        bbox = (x1, y1, x2 - x1, y2 - y1)
        is_new = self.teletubby_sensor.record(frame, bbox)

        return Detection(
            label=label,
            confidence=conf,
            cx=cx,
            cy=cy,
            bearing_deg=bearing_deg,
            is_new_teletubby=is_new,
        )

    def teletubby_maxed_out(self) -> bool:
        return self.teletubby_sensor.maxed_out()

    # TODO: Transform coordinates
    def transformCoordinates(detection):
        return detection

    def close(self):
        self.cap.release()
