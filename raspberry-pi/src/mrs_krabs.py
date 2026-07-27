import os
import sys
import time

import serial
from computer_vision import ComputerVision
from config import UART_BAUD, UART_PORT
from proto import robot_messages_pb2 as proto

sys.stdout.reconfigure(
    line_buffering=True
)  # keep prints live when stdout isn't a tty (ssh/systemd/log redirect)

CAPTURE_DIR = os.path.join(os.path.dirname(__file__), "..", "captures")
os.makedirs(CAPTURE_DIR, exist_ok=True)

uart = serial.Serial(UART_PORT, UART_BAUD)
cv = ComputerVision()


def send_command(teletubby_detected: bool):
    msg = proto.Command()
    msg.teletubby_detected = teletubby_detected
    data = msg.SerializeToString()
    uart.write(len(data).to_bytes(4, "little") + data)
    print(f"[mrs_krabs] sent teletubby_detected={teletubby_detected}")


# Main loop
print("[mrs_krabs] running")
_last_heartbeat = time.monotonic()
while True:
    detection = cv.capture(save_dir=CAPTURE_DIR)
    if detection is not None:
        print(f"[mrs_krabs] found {detection.label} (conf={detection.confidence:.2f})")
        send_command(True)

    now = time.monotonic()
    if now - _last_heartbeat >= 2.0:
        print(
            f"[mrs_krabs] alive, {len(cv.teletubby_sensor.detected)} teletubby(s) found so far"
        )
        _last_heartbeat = now

    time.sleep(0.05)  # 2 Hz
