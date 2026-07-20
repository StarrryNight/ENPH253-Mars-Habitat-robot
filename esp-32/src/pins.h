#pragma once

// All physical GPIO pin assignments, gathered here so the wiring is visible
// in one place. LEDC PWM channel numbers (not pins) stay with their owning
// component (e.g. motor_controller.h) since they're a software assignment,
// not a wire.

// Motor PWM pins
static constexpr int WHEEL_LEFT_PWM_PIN_0 = 45;
static constexpr int WHEEL_LEFT_PWM_PIN_1 = 46;
// "LEFT" on altium

static constexpr int WHEEL_RIGHT_PWM_PIN_0 = 41;
static constexpr int WHEEL_RIGHT_PWM_PIN_1 = 42;
// "RIGHT" on altium

static constexpr int WHEEL_BACK_PWM_PIN_0 = 15;
static constexpr int WHEEL_BACK_PWM_PIN_1 = 16;
// "BACK" on altium

// Encoder pins; second is set to zero since we're doing 1 encoder pin per wheel
static constexpr int WHEEL_LEFT_ENCODER_0 = 1;
// "LEFT"

static constexpr int WHEEL_RIGHT_ENCODER_0 = 3;
// "RIGHT"

static constexpr int WHEEL_BACK_ENCODER_0 = 2;
// "BACK"

// Photoresistors
static constexpr int LEFT_PHOTORESISTOR = 7;
static constexpr int MID_LEFT_PHOTORESISTOR = 8;
static constexpr int MID_RIGHT_PHOTORESISTOR = 9;
static constexpr int RIGHT_PHOTORESISTOR = 10;

//Arm servo
static constexpr int WRIST_YAW_SERVO_PIN = 13;
static constexpr int CLAW_OPEN_SERVO_PIN = 14;
// Arm servo bus pin (single-wire half-duplex UART to the SCServo bus).
static constexpr int BASE_ELBOW_PITCH_SERIAL_PIN = 6;


//Metal detecting
static constexpr int METAL_DETECTOR_PIN = 7;
