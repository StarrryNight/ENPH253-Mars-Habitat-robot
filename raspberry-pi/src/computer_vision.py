import os
import threading
import time
from dataclasses import dataclass

import cv2
import ncnn
import numpy as np

from config import MODEL_PATH, CV_CONF, CV_IOU, CV_IMGSZ, HFOV_DEG, TELETUBBY_LABELS, CAMERA_INDEX, CLASS_NAMES


@dataclass
class Detection:
    confidence: float
    cx: int          # pixel x of bounding box center
    cy: int          # pixel y of bounding box center
    # positive = target is right of frame center, negative = left.
    # Caller uses this to generate an omega correction for the drive command.
    bearing_deg: float
    label: str
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


def _letterbox(frame: np.ndarray, size: int) -> tuple[np.ndarray, float, int, int]:
    """Resize preserving aspect ratio and pad to (size, size) with grey.
    Returns the padded image plus the scale and (pad_x, pad_y) needed to map
    detection boxes back to original frame coordinates."""
    h, w = frame.shape[:2]
    scale = min(size / w, size / h)
    new_w, new_h = round(w * scale), round(h * scale)
    resized = cv2.resize(frame, (new_w, new_h), interpolation=cv2.INTER_LINEAR)

    pad_x, pad_y = (size - new_w) // 2, (size - new_h) // 2
    padded = np.full((size, size, 3), 114, dtype=np.uint8)
    padded[pad_y:pad_y + new_h, pad_x:pad_x + new_w] = resized
    return padded, scale, pad_x, pad_y


class ComputerVision:
    def __init__(
        self, camera_index: int = CAMERA_INDEX, frame_width: int = 640, frame_height: int = 480
    ):
        self.frame_width = frame_width
        self.frame_height = frame_height

        self.cap = cv2.VideoCapture(camera_index)
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, frame_width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, frame_height)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)  # always read the latest frame, not a queued stale one
        if not self.cap.isOpened():
            raise RuntimeError(f"Could not open camera {camera_index}")

        self.net = ncnn.Net()
        self.net.opt.num_threads = max(1, os.cpu_count() - 1)  # leave a core free for UART/OS so inference doesn't starve everything else
        self.net.load_param(os.path.join(MODEL_PATH, "model.ncnn.param"))
        self.net.load_model(os.path.join(MODEL_PATH, "model.ncnn.bin"))

        self.teletubby_sensor = TeletubbySensor()

        # The camera driver buffers frames internally regardless of CAP_PROP_BUFFERSIZE,
        # so reading only when we run inference hands back a stale queued frame. A
        # background thread drains the camera continuously; capture() just grabs
        # whatever's freshest.
        self._latest_frame: np.ndarray | None = None
        self._frame_lock = threading.Lock()
        self._reader_thread = threading.Thread(target=self._read_loop, daemon=True)
        self._reader_thread.start()

    def _read_loop(self):
        while True:
            ok, frame = self.cap.read()
            if ok:
                with self._frame_lock:
                    self._latest_frame = frame

    def _infer(self, frame: np.ndarray) -> list[tuple[float, float, float, float, float, int]]:
        """Run NCNN inference on a BGR frame. Returns (x1, y1, x2, y2, conf,
        class_id) boxes in original-frame pixel coordinates, already filtered
        by confidence threshold and NMS."""
        padded, scale, pad_x, pad_y = _letterbox(frame, CV_IMGSZ)

        blob = cv2.cvtColor(padded, cv2.COLOR_BGR2RGB).astype(np.float32) / 255.0
        blob = np.ascontiguousarray(blob.transpose(2, 0, 1))  # HWC -> CHW

        ex = self.net.create_extractor()
        ex.input("in0", ncnn.Mat(blob))
        _, out0 = ex.extract("out0")
        pred = np.array(out0)  # (4 + num_classes, num_anchors)
        if pred.shape[0] == 4 + len(CLASS_NAMES):
            pred = pred.T  # -> (num_anchors, 4 + num_classes)

        class_scores = pred[:, 4:]
        class_ids = class_scores.argmax(axis=1)
        confs = class_scores[np.arange(len(pred)), class_ids]

        keep = confs >= CV_CONF
        pred, confs, class_ids = pred[keep], confs[keep], class_ids[keep]
        if len(pred) == 0:
            return []

        cx, cy, w, h = pred[:, 0], pred[:, 1], pred[:, 2], pred[:, 3]
        boxes_xywh = np.stack([cx - w / 2, cy - h / 2, w, h], axis=1)

        indices = cv2.dnn.NMSBoxes(boxes_xywh.tolist(), confs.tolist(), CV_CONF, CV_IOU)
        if len(indices) == 0:
            return []

        results = []
        for i in np.array(indices).flatten():
            x, y, w_box, h_box = boxes_xywh[i]
            x1 = (x - pad_x) / scale
            y1 = (y - pad_y) / scale
            x2 = (x + w_box - pad_x) / scale
            y2 = (y + h_box - pad_y) / scale
            results.append((x1, y1, x2, y2, float(confs[i]), int(class_ids[i])))
        return results

    def capture(self, save_dir: str | None = None) -> Detection | None:
        """Grab a frame, run local NCNN inference, return the highest-confidence
        teletubby detection or None. Sets is_new_teletubby=True only when the
        detection is a colour not seen before.

        If save_dir is given, the raw frame is written there as a timestamped
        JPEG before inference runs."""
        with self._frame_lock:
            frame = self._latest_frame
        if frame is None:
            return None

        if save_dir is not None:
            cv2.imwrite(os.path.join(save_dir, f"{time.time():.3f}.jpg"), frame)

        detections = self._infer(frame)
        teletubby_dets = [d for d in detections if CLASS_NAMES[d[5]] in TELETUBBY_LABELS]
        if not teletubby_dets:
            return None

        x1, y1, x2, y2, conf, class_id = max(teletubby_dets, key=lambda d: d[4])
        label = CLASS_NAMES[class_id]

        cx = int((x1 + x2) / 2)
        cy = int((y1 + y2) / 2)

        bearing_deg = ((cx - self.frame_width / 2) / self.frame_width) * HFOV_DEG

        bbox = (int(x1), int(y1), int(x2 - x1), int(y2 - y1))
        is_new = self.teletubby_sensor.record(frame, bbox)

        return Detection(
            confidence=conf,
            cx=cx,
            cy=cy,
            bearing_deg=bearing_deg,
            label=label,
            is_new_teletubby=is_new,
        )

    def teletubby_maxed_out(self) -> bool:
        return self.teletubby_sensor.maxed_out()

    def close(self):
        self.cap.release()
