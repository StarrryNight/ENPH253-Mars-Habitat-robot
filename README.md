# ENPH253 Mars Habitat Robot

Autonomous robot for the ENPH 253 competition. Navigates an 8×8ft surface to assemble habitat pieces, detect Teletubbies, and recover aluminum rocks. Target: 8 points.

## Hardware

- **Chassis:** Octagonal, kiwi drive (3 omni wheels at 120°)
- **Arm:** 4-DOF — pitch @ base, pitch @ elbow, yaw @ wrist, servo claw
- **Drive:** H-bridge per motor

## Processors

| Processor | Role |
|-----------|------|
| ESP32-S3 | All real-time control |
| RPi 4 2GB | State machine + UART coordination |

### ESP32 runs
- Superloop (Mr Crab)
- Timer ISR — motor PID ×3, arm PID ×3 at fixed rate
- Motion profile — setpoint ramping
- Kiwi drive mixing — (vx, vy, ω) → 3 motor commands
- Line following — IR sensor array
- Metal detector — interrupt driven
- Hardcoded pose sequencer — arm joint targets

### RPi runs
- State machine — drives all high-level decisions
- UART manager — sends commands, receives sensor state

## UART Contract

| Direction | Payload |
|-----------|---------|
| RPi → ESP32 | Current state, target pose, drive commands (vx, vy, ω) |
| ESP32 → RPi | Wheel encoder positions, arm joint positions, metal detected event |

## State Machine

`Line-following → Holding → Arming → Metalling → Teletubbying`

## Sensors

| Sensor | Use |
|--------|-----|
| Wheel encoders | Velocity PID + dead reckoning |
| Arm encoders | Position PID, reset on limit switch |
| IR line array | Line following |
| Inductive metal detector | Rock detection (interrupt driven) |
| USB camera (RPi) | Teletubby color blob detection |

## Repository Layout

```
esp-32/          # PlatformIO project (Arduino/FreeRTOS)
  src/
    MrCrab            # Superloop
    localizer         # Dead reckoning position tracker
    translation       # (vx, vy, ω) → kiwi wheel speeds
    pid_controller    # Generic PID — wheels and arm joints
    motor_driver      # PWM output to H-bridge
    motion_profile    # Setpoint ramping
raspberry-pi/    # High-level C++ controller
  src/
    robot_fsm         # State machine
    robot_communication  # UART manager
shared/
  messages/robot_messages.proto  # Protobuf message definitions
cv_test.py       # HSV color blob detection (OpenCV)
```

## To-do

### esp-32
- [ ] Write setup and skeleton of MrCrab
- [ ] Test on ESP
- [ ] Write localizer (position tracker)
- [ ] Write motion profile for robot
- [ ] Write motion profile for arm
- [x] Write generic PID controller

### raspberry-pi
- [ ] Write Ms Crab (RPi superloop)
- [ ] Set up camera
- [ ] Write state machine
- [ ] Write arm poses
- [ ] Write color blob detection
- [ ] Write hardcoded kinematics

### Integration
- [ ] Write proto messages
- [ ] Write UART translation layer for both sides
- [ ] Test end-to-end communication

## Setup

### ESP32

Requires [PlatformIO](https://platformio.org/).

```bash
cd esp-32
pio run            # build
pio run -t upload  # flash
pio device monitor # 115200 baud
```

### Raspberry Pi

```bash
python3 -m venv .venv && source .venv/bin/activate
pip install opencv-python numpy
```
