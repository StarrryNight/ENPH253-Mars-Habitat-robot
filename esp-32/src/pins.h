#pragma once

// All physical GPIO pin assignments, gathered here so the wiring is visible
// in one place. LEDC PWM channel numbers (not pins) stay with their owning
// component (e.g. motor_controller.h) since they're a software assignment,
// not a wire.

// Motor PWM pins
static constexpr int WHEEL_LEFT_PWM_PIN_0 = 47;
static constexpr int WHEEL_LEFT_PWM_PIN_1 = 48; // "LEFT" on altium

static constexpr int WHEEL_RIGHT_PWM_PIN_0 = 38;
static constexpr int WHEEL_RIGHT_PWM_PIN_1 = 39;
// "RIGHT" on altium

static constexpr int WHEEL_BACK_PWM_PIN_0 = 21;
static constexpr int WHEEL_BACK_PWM_PIN_1 = 40;
// "BACK" on altium

// Encoder pins; second is set to zero since we're doing 1 encoder pin per wheel
static constexpr int WHEEL_LEFT_ENCODER_0 = 8;
// "LEFT"

static constexpr int WHEEL_RIGHT_ENCODER_0 = 9;
// "RIGHT"

static constexpr int WHEEL_BACK_ENCODER_0 = 7;
// "BACK"

// Photoresistors
static constexpr int LEFT_PHOTORESISTOR = 15;
static constexpr int MID_LEFT_PHOTORESISTOR = 16;
static constexpr int MID_RIGHT_PHOTORESISTOR = 17;
static constexpr int RIGHT_PHOTORESISTOR = 18;

//Arm servo
static constexpr int CLAW_OPEN_SERVO_PIN = 4;
static constexpr int WRIST_YAW_SERVO_PIN = 5;
// Arm servo bus pin (single-wire half-duplex UART to the SCServo bus).
static constexpr int BASE_ELBOW_PITCH_SERIAL_PIN = 6;


//Metal detecting
static constexpr int METAL_DETECTOR_PIN = 10;

// Go
static constexpr int GO_PIN = 11;


// OLED display (I2C)
static constexpr int OLED_SDA = 1;
static constexpr int OLED_SCK = 2;

// SONAR
// TODO: placeholder GPIOs, not yet confirmed against wiring — update once known.
static constexpr int SONAR_TRIG = 41;
static constexpr int SONAR_ECHO = 42;


// LED
static constexpr int TELETUBBY_LED = 12;
