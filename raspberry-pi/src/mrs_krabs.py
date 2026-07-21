import time

import serial
from computer_vision import ComputerVision
from config import UART_PORT, UART_BAUD
from proto import robot_messages_pb2 as proto

uart = serial.Serial(UART_PORT, UART_BAUD)
cv = ComputerVision()


def send_command(teletubby_bearing_deg: float):
    msg = proto.Command()
    msg.state = "teletubbying"
    msg.teletubby_bearing_deg = teletubby_bearing_deg
    data = msg.SerializeToString()
    uart.write(len(data).to_bytes(4, "little") + data)
    print(f"[mrs_krabs] sent teletubby command, bearing={teletubby_bearing_deg:.1f} deg")


# Main loop
print("[mrs_krabs] running")
_last_heartbeat = time.monotonic()
while True:
    if not cv.teletubby_maxed_out():
        detection = cv.capture()
        if detection is not None and detection.is_new_teletubby:
            send_command(detection.bearing_deg)

    now = time.monotonic()
    if now - _last_heartbeat >= 2.0:
        print(f"[mrs_krabs] alive, {len(cv.teletubby_sensor.detected)} teletubby(s) found so far")
        _last_heartbeat = now

    time.sleep(0.05)  # 20 Hz
