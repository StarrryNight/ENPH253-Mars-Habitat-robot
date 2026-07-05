import sys

import cv2
import numpy as np

ROCK_MIN_AREA = 200000
ROCK_MAX_AREA = 500000
ROI_PADDING = 80
YELLOW_LOWER = np.array([20, 100, 100])
YELLOW_UPPER = np.array([35, 255, 255])
YELLOW_PIXEL_THRESHOLD = 500


def is_rock_shaped(contour):
    area = cv2.contourArea(contour)
    if not (ROCK_MIN_AREA < area < ROCK_MAX_AREA):
        return False
    hull_area = cv2.contourArea(cv2.convexHull(contour))
    if hull_area == 0:
        return False
    solidity = area / hull_area
    x, y, w, h = cv2.boundingRect(contour)
    aspect_ratio = w / h
    return 0.5 < aspect_ratio < 2.0 and solidity > 0.5


def find_rocks(frame):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    rock_lower = np.array([0, 0, 50])
    rock_upper = np.array([30, 80, 180])
    rock_mask = cv2.inRange(hsv, rock_lower, rock_upper)
    white_lower = np.array([0, 0, 200])
    white_upper = np.array([180, 30, 255])
    white_mask = cv2.inRange(hsv, white_lower, white_upper)
    rock_mask = cv2.bitwise_and(rock_mask, cv2.bitwise_not(white_mask))
    contours, _ = cv2.findContours(
        rock_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
    )
    rocks = []
    for contour in contours:
        if is_rock_shaped(contour):
            rocks.append((cv2.boundingRect(contour), contour))
    return rocks


def check_roi_for_teletubby(frame, rock_bbox):
    x, y, w, h = rock_bbox
    h_frame, w_frame = frame.shape[:2]
    x1 = max(0, x - ROI_PADDING)
    y1 = max(0, y - ROI_PADDING)
    x2 = min(w_frame, x + w + ROI_PADDING)
    y2 = min(h_frame, y + h + ROI_PADDING)
    roi = frame[y1:y2, x1:x2]
    hsv_roi = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    yellow_mask = cv2.inRange(hsv_roi, YELLOW_LOWER, YELLOW_UPPER)
    yellow_count = cv2.countNonZero(yellow_mask)
    return yellow_count > YELLOW_PIXEL_THRESHOLD, yellow_count, (x1, y1, x2, y2)


def run(image_path):
    frame = cv2.imread(image_path)
    if frame is None:
        print(f"Failed to load image: {image_path}")
        return

    debug = frame.copy()
    rocks = find_rocks(frame)
    print(f"Found {len(rocks)} rock candidates")

    for rock_bbox, contour in rocks:
        x, y, w, h = rock_bbox
        cv2.rectangle(debug, (x, y), (x + w, y + h), (255, 0, 0), 2)
        cv2.putText(
            debug, "Rock", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2
        )

        found, yellow_count, roi_coords = check_roi_for_teletubby(frame, rock_bbox)
        print(f"Rock at ({x},{y}) {w}x{h} — yellow pixels: {yellow_count}")

        if found:
            x1, y1, x2, y2 = roi_coords
            cv2.rectangle(debug, (x1, y1), (x2, y2), (0, 255, 255), 1)
            cv2.rectangle(debug, (x, y), (x + w, y + h), (0, 255, 0), 3)
            cv2.putText(
                debug,
                f"TELETUBBY ({yellow_count}px)",
                (x, y - 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 255, 0),
                2,
            )

    scale = 0.3
    display = cv2.resize(
        debug, (int(debug.shape[1] * scale), int(debug.shape[0] * scale))
    )
    cv2.imwrite("detection_debug.png", display)
    print("Saved detection_debug.png")


if __name__ == "__main__":
    image_path = sys.argv[1] if len(sys.argv) > 1 else "IMG_3213.png"
    run(image_path)
