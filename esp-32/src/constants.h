#pragma once
#include <cstdint>


// Control loop — 10 ms period
static constexpr int CONTROL_LOOP_PERIOD_US = 10000;        // µs, for esp_timer_start_periodic
static constexpr double CONTROL_LOOP_PERIOD = 0.01;         // seconds, for PID delta_t

// Photoresistors
static constexpr int LEFT_PHOTORESISTOR = 39;
static constexpr int MID_LEFT_PHOTORESISTOR = 40;
static constexpr int MID_RIGHT_PHOTORESISTOR = 21;
static constexpr int RIGHT_PHOTORESISTOR = 38;

// Raw 12-bit ADC threshold (0–4095). Calibrate on your surface.
static constexpr double LIGHT_THRESHOLD_ADC = 2000;
static constexpr double SMALL_ERROR_VALUE = 1;
static constexpr double BIG_ERROR_VALUE = 5;

// Motor PWM channels and pins
static constexpr int WHEEL_LEFT_PWM_CHANNEL_0 = 0;
static constexpr int WHEEL_LEFT_PWM_CHANNEL_1 = 1;
static constexpr int WHEEL_LEFT_PWM_PIN_0 = 46;
static constexpr int WHEEL_LEFT_PWM_PIN_1 = 45;
// "LEFT" on altium

static constexpr int WHEEL_RIGHT_PWM_CHANNEL_0 = 2;
static constexpr int WHEEL_RIGHT_PWM_CHANNEL_1 = 3;
static constexpr int WHEEL_RIGHT_PWM_PIN_0 = 41;
static constexpr int WHEEL_RIGHT_PWM_PIN_1 = 42;
// "RIGHT" on altium

static constexpr int WHEEL_BACK_PWM_CHANNEL_0 = 4;
static constexpr int WHEEL_BACK_PWM_CHANNEL_1 = 5;
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

// Distance traveled per encoder tick (m). Set from wheel circumference / ticks-per-rev.
static constexpr double ENCODER_RESOLUTION_DISTANCE_M = 0.009163; // at 70 mm diameter, 24 ticks-per-rev

// Forward speed during line following (m/s). Tune empirically.
static constexpr double FORWARD_SPEED = 1;

// Open-loop forward drive test duration (µs) — see MrKrabs::stepControl.
static constexpr uint64_t FORWARD_DRIVE_TEST_DURATION_US = 10000000000; // 1 s

// Rotation is considered complete once within this angular distance (rad) of
// the target angle. See OrientationController::reachedTarget.
static constexpr double ROTATION_TOLERANCE_RAD = 0.05;

// Converts PID output (m/s) to PWM counts. Tune to match motor transfer function.
static constexpr double VELOCITY_TO_PWM = 200.0;

// Below this commanded PWM magnitude, treat speed as zero instead of applying
// the deadband floor below. Prevents FP/PID noise around a 0 target (e.g. the
// back wheel while driving straight) from producing a small nonzero duty cycle.
static constexpr double MOTOR_SPEED_DEADZONE = 5.0;

// Values
static constexpr int speed8bit = 230; // speed scales linear from 0 to 256

// Arm servo pins
static constexpr int pitch_base = 0;
static constexpr int pitch_elbow = 0;
static constexpr int yaw_wrist = 0;
static constexpr int open_claw = 0;

// Arm


// BUFFER SIZE
static constexpr int VELOCITY_BUFFER_SIZE = 10;
