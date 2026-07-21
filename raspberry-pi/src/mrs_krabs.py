import time

import serial
from computer_vision import ComputerVision
from config import UART_PORT, UART_BAUD
from proto import robot_messages_pb2 as proto

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
    detection = cv.capture()
    if detection is not None:
        print(f"[mrs_krabs] found {detection.label} (conf={detection.confidence:.2f})")
        send_command(True)

    now = time.monotonic()
    if now - _last_heartbeat >= 2.0:
        print(f"[mrs_krabs] alive, {len(cv.teletubby_sensor.detected)} teletubby(s) found so far")
        _last_heartbeat = now

    time.sleep(0.05)  # 20 Hz
