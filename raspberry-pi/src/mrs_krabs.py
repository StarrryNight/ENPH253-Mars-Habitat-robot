import time

import serial
from proto import robot_messages_pb2 as proto
from state_machine import RobotStateMachine

uart = serial.Serial("/dev/ttyUSB0", 115200)
state_machine = RobotStateMachine()


def send_command(state: str):
    msg = proto.Command()
    msg.state = state
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
    # Update condition flags from sensor data
    state_machine._metal_detected = sensor_state.metal_detected
    state_machine._metal_not_detected = not sensor_state.metal_detected

    # When ESP32 finishes executing the current state, advance
    if sensor_state.state_completed:
        state_machine.send("cycle")

    send_command(state_machine.current_state.id)


# Main loop
while True:
    sensor_state = receive_state()
    if sensor_state is not None:
        tick(sensor_state)
    time.sleep(0.05)  # 20 Hz
