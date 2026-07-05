import time

import serial
from proto import robot_messages_pb2 as proto
from state_machine import RobotStateMachine
from computer_vision import ComputerVision

uart = serial.Serial("/dev/ttyUSB0", 115200)
state_machine = RobotStateMachine()
cv = ComputerVision()


def send_command(state: str, teletubby_bearing_deg: float):
    msg = proto.Command()
    msg.state = state
    msg.teletubby_bearing_deg = teletubby_bearing_deg
    data = msg.SerializeToString()
    uart.write(len(data).to_bytes(4, "little") + data)


def receive_state() -> proto.SensorState | None:
    if uart.in_waiting == 0:
        return None
    data = uart.read(uart.in_waiting)
    msg = proto.SensorState()
    msg.ParseFromString(data)
    return msg


def tick(sensor_state: proto.SensorState):
    # Update metal detector conditions from sensor data
    state_machine._metal_detected = sensor_state.metal_detected
    state_machine._metal_not_detected = not sensor_state.metal_detected

    # Only look for teletubbies if we haven't found both yet
    teletubby_bearing = 0.0
    if not cv.teletubby_maxed_out():
        detection = cv.capture()
        if detection is not None and detection.is_new_teletubby:
            state_machine._enter_teletubby = True
            teletubby_bearing = detection.bearing_deg

    # When ESP32 finishes executing the current state, advance
    if sensor_state.state_completed:
        state_machine.send("cycle")
        state_machine._enter_teletubby = False  # clear after transition

    send_command(state_machine.current_state.id, teletubby_bearing)


# Main loop
while True:
    sensor_state = receive_state()
    if sensor_state is not None:
        tick(sensor_state)
    time.sleep(0.05)  # 20 Hz
