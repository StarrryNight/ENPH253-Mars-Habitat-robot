# ENPH253 Mars Habitat Robot

Autonomous robot for the ENPH 253 course project. The robot navigates a simulated Mars habitat environment using computer vision and closed-loop motor control.

## Architecture

The system splits responsibilities across two microcontrollers:

- **ESP32-S3** — low-level motor control, PID velocity regulation, and odometry-based localization. Built with PlatformIO (Arduino framework).
- **Raspberry Pi** — high-level state machine, computer vision (OpenCV), and communication with the ESP32 over a protobuf serial protocol.

## Repository Layout

```
esp-32/          # PlatformIO project for the ESP32-S3
  src/
    MrCrab            # Superloop
    localizer         # Odometry / position tracking
    translation       # Velocity commands → wheel speeds
    pid_controller    # Generic PID implementation, used for arm and wheels
    motor_driver      # PWM output to H-bridge
    motion_profile    # Plans motion profile of robot
raspberry-pi/    # C++ high-level controller
  src/
    robot_fsm         # Finite state machine
    robot_communication  # Serial comms with ESP32
shared/
  messages/robot_messages.proto  # Shared protobuf message definitions
cv_test.py       # HSV color blob detection test (OpenCV)
```

## To-do List (by important-first order)

### esp-32

- [ ] Write setup and skeleton of MrCrab
- [ ] Test on esp
- [ ] Write localizer (positionl tracker)
- [ ] Write motion profile for robot
- [x] Write generic PID controller

### raspberry-pi

- [ ] Write setup software 
- [ ] Write statemachine

### Integration

## Setup

### ESP32 firmware

Requires [PlatformIO](https://platformio.org/).

```bash
cd esp-32
pio run            # build
pio run -t upload  # flash
pio device monitor # serial monitor (115200 baud)
```

### Raspberry Pi

```bash
python3 -m venv .venv && source .venv/bin/activate
pip install opencv-python numpy
```
