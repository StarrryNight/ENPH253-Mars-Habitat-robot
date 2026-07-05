from dataclasses import dataclass

import cv2
from ultralytics import YOLO

MODEL_PATH = "model_ncnn_model"  # directory containing .param and .bin files

CONF = 0.25
IOU = 0.7
IMGSZ = 640


@dataclass
class Detection:
    label: str
    confidence: float
    cx: int  # pixel x of bounding box center
    cy: int  # pixel y of bounding box center
    # positive = target is right of frame center, negative = left.
    # Caller uses this to generate an omega correction for the drive command.
    bearing_deg: float


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

    def capture(self) -> Detection | None:
        """Grab a frame, run local NCNN inference, return highest-confidence detection or None."""
        ok, frame = self.cap.read()
        if not ok:
            return None

        results = self.model(frame, conf=CONF, iou=IOU, imgsz=IMGSZ, verbose=False)

        boxes = results[0].boxes
        if boxes is None or len(boxes) == 0:
            return None

        # Pick highest-confidence detection
        best_idx = int(boxes.conf.argmax())
        x1, y1, x2, y2 = boxes.xyxy[best_idx].tolist()
        conf = float(boxes.conf[best_idx])
        label = results[0].names[int(boxes.cls[best_idx])]

        cx = int((x1 + x2) / 2)
        cy = int((y1 + y2) / 2)

        # #TODO: Tune this. Currently assumes ~60° horizontal FOV. Update once camera FOV is measured.
        hfov_deg = 60.0
        bearing_deg = ((cx - self.frame_width / 2) / self.frame_width) * hfov_deg

        return Detection(
            label=label, confidence=conf, cx=cx, cy=cy, bearing_deg=bearing_deg
        )

    # TODO: Transform coordinates
    def transformCoordinates(detection):
        return detection

    def close(self):
        self.cap.release()
