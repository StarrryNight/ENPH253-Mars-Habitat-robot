## UART Message Contract

**RPi → ESP32**

- Current state (string)
- Flash light command (bool, for Teletubby)

**ESP32 → RPi**

- state_completed (bool)
- metal_detected (bool)

## State Machine (RPi)

```python
class StateMachine(StateChart):
    line_following = State(initial=True)
    teletubbying = State()

    class metal_detecting(State.Compound):
        scanning = State(initial=True)
        retrieving = State()
        _cycle = scanning.to(retrieving, cond="metal_detected")

    class habitating(State.Compound):
        grabbing = State(initial=True)
        holding = State()
        placing = State()
        _cycle = grabbing.to(holding) | holding.to(placing)

    cycle = (
        line_following.to(metal_detecting, cond="enter_metal_detection")
        | line_following.to(teletubbying, cond="enter_teletubby")
        | line_following.to(habitating, cond="enter_habitat")
        | metal_detecting.scanning.to(line_following, cond="metal_not_detected")
        | metal_detecting.retrieving.to(line_following)
        | teletubbying.to(line_following)
        | habitating.to(line_following)
    )
```

## Arm Architecture

- **Type:** Fixed-sequence servo manipulator
- **Joints:** All servos — no PID needed, internal position control
- **Poses:** home, sweep, rock pickup, rock deposit, habitat grab, habitat place
- **Control:** ESP32 writes servo PWM angles directly per state
- **No arm PID required** — servos hold position internally

## Kiwi Drive

- 3 omni wheels at 120° separation
- Inverse mixing: (vx, vy, ω) → 3 motor commands
- Forward mixing: 3 encoder readings → (vx, vy, ω) for dead reckoning
- Motor PID: velocity control at 100Hz via timer ISR
- Direction reversal: coast one control cycle, dead time per MOSFET t_d(off) + t_f

## Localization

- Primary: IR line following (ground truth)
- Secondary: Encoder dead reckoning between tape landmarks
- Reset: Limit switch or state transition

## CV Pipeline (RPi)

- Rock detection: HSV color mask (grey-brown) + contour filtering
- Teletubby detection: regional proposals around rock bounding boxes + yellow HSV filter
- Deduplication: dominant hue recorded per confirmed detection, compared on subsequent detections
- Confirmation: N consecutive frames required before state transition triggered

## Constraints

- Fully autonomous
- Max 18x18x18 inches
- Battery powered only
- No discrete H-bridge driver chips
- $200 team budget cap
