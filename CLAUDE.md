# ENPH 253 – Mars Habitat Robot Competition

## Competition

Autonomous robot competition on an 8x8ft wooden surface with ramp and 6" elevated platform.
Two robots compete head-to-head in 2-minute heats. Robots can be restarted; highest score counts.

## Scoring

| Task                   | Points      | Status  |
| ---------------------- | ----------- | ------- |
| Habitat assembly       | 4 (1/piece) | ✅      |
| Teletubby discovery    | 2 (1/each)  | ✅      |
| Aluminum rock recovery | 2           | ✅      |
| Solar panel uncovering | 2           | ❌ Skip |
| Tower assembly         | 3 (1/piece) | ❌ Skip |

**Target: 8 points**

## Hardware

- Octagonal chassis, tank drive (2 wheels + 2 casters)
- 4-DOF arm: pitch @ base, pitch @ elbow, yaw @ wrist, open @ claw
- Hole in chassis for rock deposit

## Electronics

- ESP32-S3: all real-time control
- RPi 4 2GB: CV + high-level state commands
- UART + protobuf for inter-processor communication

## Software

**ESP32:** superloop (Mr Crab), timer ISR PID, motor PID, arm PID, line following, metal detector, motion profiles, state machine
**RPi:** OpenCV HSV color blob detection (Teletubby), state commands to ESP32

**States:** Line-following → Holding → Arming → Metalling → Teletubbying

**Arm:** Fixed-sequence position-controlled manipulator. Hardcoded joint-angle poses, independent PID per joint.
**Poses:** home, sweep, rock pickup, rock deposit, habitat grab, habitat place

**Localization:** Encoder dead reckoning + line following as drift correction. Reset on limit switch or state transition.

## Constraints

- Fully autonomous
- Max 18x18x18 inches
- Battery powered only
- No discrete H-bridge driver chips
- $200 team budget cap

# Strategy, Stack & Architecture

## Strategy

Score 8 points reliably over 13 possible. Prioritize deterministic tasks over complex ones.
Robust line following + dead reckoning over sophisticated localization.
Hardcoded poses over runtime IK. Simple CV over ML.

## Task Priority

1. Habitat assembly — 4pts, arm + line following
2. Teletubby detection — 2pts, CV + flash light
3. Aluminum rock collection — 2pts, metal detector + arm

## Hardware Stack

- **Chassis:** Octagonal, kiwi drive (3 omni wheels at 120°)
- **Arm:** 4-DOF (pitch @ base, pitch @ elbow, yaw @ wrist, open @ claw), DC motors + encoders, servo claw
- **Drivetrain:** 3x DC motors + wheel encoders, H-bridge driven
- **Sensors:** IR line following array, wheel encoders, arm encoders, metal detector (inductive loop), USB camera

## Software Stack

- **ESP32-S3:** C++, FreeRTOS, superloop (Mr Crab)
- **RPi 4 2GB:** Python, OpenCV, pyserial
- **Protocol:** UART + Google Protobuf

## Architecture
