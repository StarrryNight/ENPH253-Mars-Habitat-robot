"""Live camera + detection preview for debugging on a machine with a display.
Not used by mrs_krabs.py — the robot itself is headless.
"""
import cv2

from computer_vision import ComputerVision
from config import CV_CONF, CV_IOU, CV_IMGSZ

cv_ = ComputerVision()

print("[debug_view] press q to quit")
while True:
    ok, frame = cv_.cap.read()
    if not ok:
        continue

    results = cv_.model(frame, conf=CV_CONF, iou=CV_IOU, imgsz=CV_IMGSZ, verbose=False)
    annotated = results[0].plot()
    cv2.imshow("mrs_krabs camera", annotated)

    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

cv_.close()
cv2.destroyAllWindows()
