"""Live camera + detection preview for debugging on a machine with a display.
Not used by mrs_krabs.py — the robot itself is headless.
"""
import cv2

from computer_vision import ComputerVision
from config import CLASS_NAMES

cv_ = ComputerVision()

print("[debug_view] press q to quit")
while True:
    ok, frame = cv_.cap.read()
    if not ok:
        continue

    annotated = frame.copy()
    for x1, y1, x2, y2, conf, class_id in cv_._infer(frame):
        x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)
        cv2.rectangle(annotated, (x1, y1), (x2, y2), (0, 255, 0), 2)
        label = f"{CLASS_NAMES[class_id]} {conf:.2f}"
        cv2.putText(annotated, label, (x1, max(y1 - 5, 0)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
    cv2.imshow("mrs_krabs camera", annotated)

    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

cv_.close()
cv2.destroyAllWindows()
