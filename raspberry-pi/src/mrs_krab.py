iimport serial
import proto.messages_pb2 as proto
import time

# Setup
uart = serial.Serial('/dev/ttyUSB0', 115200)

def send_command(state, target_pose, drive_command):
    msg = proto.Command()
    msg.state = state
    msg.target_pose = target_pose
    msg.drive_command.CopyFrom(drive_command)
    uart.write(msg.SerializeToString())

def receive_state():
    data = uart.read(uart.in_waiting)
    msg = proto.SensorState()
    msg.ParseFromString(data)
    return msg

def decide_command(sensor_state):
    # State machine logic here
    pass

# Main loop
while True:
    sensor_state = receive_state()
    command = decide_command(sensor_state)
    send_command(command.state, command.pose, command.drive)
    time.sleep(0.05)  # 20Hzmport shared.messages.robot_messages.proto;

def setup():
    return 0

def super_loop():
    while (True):
        return 0

