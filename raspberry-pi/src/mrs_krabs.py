import time

import serial
from computer_vision import ComputerVision
from proto import robot_messages_pb2 as proto
from state_machine import RobotStateMachine

uart = serial.Serial("/dev/ttyUSB0", 115200)
state_machine = RobotStateMachine()
cv = ComputerVision()

METAL_DETECT_TIMEOUT_S = 15.0
HABITAT_TIMEOUT_S      = 45.0

# Bearing of the most recently spotted teletubby not yet acted on.
# Survives ticks so a sighting during metal_detecting isn't lost.
_pending_teletubby_bearing: float | None = None
_state_start_time: float = time.monotonic()
_prev_top_state: str = "line_following"


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


def check_timeouts() -> bool:
    """Force a state transition if the current state has been running too long.
    Returns True if a timeout fired."""
    global _state_start_time
    top = state_machine.current_top_state_id()
    elapsed = time.monotonic() - _state_start_time

    if top == "metal_detecting" and elapsed > METAL_DETECT_TIMEOUT_S:
        state_machine._metal_not_detected = True
        state_machine._metal_detected = False
        state_machine.send("cycle")
        _state_start_time = time.monotonic()
        return True

    if top == "habitating" and elapsed > HABITAT_TIMEOUT_S:
        state_machine.send("cycle")  # habitating.to(line_following) has no condition
        _state_start_time = time.monotonic()
        return True

    return False


def tick(sensor_state: proto.SensorState):
    global _pending_teletubby_bearing, _state_start_time, _prev_top_state

    state_machine._metal_detected = sensor_state.metal_detected
    state_machine._metal_not_detected = not sensor_state.metal_detected

    # TODO: run cv.capture() in a background thread — inference can take 100–400 ms
    # and currently blocks the UART loop for that duration.
    if not cv.teletubby_maxed_out():
        detection = cv.capture()
        if detection is not None and detection.is_new_teletubby:
            _pending_teletubby_bearing = detection.bearing_deg

    # Arm the teletubby transition only from line_following.
    # If spotted mid-metal_detecting, the bearing is held until we return.
    if _pending_teletubby_bearing is not None:
        if state_machine.current_top_state_id() == "line_following":
            state_machine._enter_teletubby = True

    if sensor_state.state_completed:
        state_machine.send("cycle")
        if state_machine.current_top_state_id() == "teletubbying":
            state_machine._enter_teletubby = False
            _pending_teletubby_bearing = None

    check_timeouts()

    # Track when we enter a new top-level state so timeouts are relative to entry.
    top = state_machine.current_top_state_id()
    if top != _prev_top_state:
        _state_start_time = time.monotonic()
        _prev_top_state = top

    send_command(top, _pending_teletubby_bearing or 0.0)


# Main loop
while True:
    sensor_state = receive_state()
    if sensor_state is not None:
        tick(sensor_state)
    time.sleep(0.05)  # 20 Hz
