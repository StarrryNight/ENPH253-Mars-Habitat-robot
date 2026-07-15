#pragma once

// All physical GPIO pin assignments, gathered here so the wiring is visible
// in one place. LEDC PWM channel numbers (not pins) stay with their owning
// component (e.g. motor_controller.h) since they're a software assignment,
// not a wire.

// Motor PWM pins
static constexpr int WHEEL_LEFT_PWM_PIN_0 = 46;
static constexpr int WHEEL_LEFT_PWM_PIN_1 = 45;
// "LEFT" on altium

static constexpr int WHEEL_RIGHT_PWM_PIN_0 = 41;
static constexpr int WHEEL_RIGHT_PWM_PIN_1 = 42;
// "RIGHT" on altium

static constexpr int WHEEL_BACK_PWM_PIN_0 = 16;
static constexpr int WHEEL_BACK_PWM_PIN_1 = 15;
// "BACK" on altium

// Encoder pins; second is set to zero since we're doing 1 encoder pin per wheel
static constexpr int WHEEL_LEFT_ENCODER_0 = 1;
// "LEFT"

static constexpr int WHEEL_RIGHT_ENCODER_0 = 3;
// "RIGHT"

static constexpr int WHEEL_BACK_ENCODER_0 = 2;
// "BACK"

// Photoresistors
static constexpr int LEFT_PHOTORESISTOR = 39;
static constexpr int MID_LEFT_PHOTORESISTOR = 40;
static constexpr int MID_RIGHT_PHOTORESISTOR = 21;
static constexpr int RIGHT_PHOTORESISTOR = 38;

// Arm servo pins (8V?)
//TODO change (5V)
static constexpr int WRIST_YAW_SERVO_PIN = 13;
static constexpr int CLAW_OPEN_SERVO_PIN = 14;

// Arm servo bus pin (single-wire half-duplex UART to the SCServo bus).
static constexpr int BASE_ELBOW_PITCH_SERIAL_PIN = 6;
