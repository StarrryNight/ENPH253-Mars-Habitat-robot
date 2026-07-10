#include <cstdint>

// Control loop — 10 ms period
static constexpr int CONTROL_LOOP_PERIOD_US = 10000;        // µs, for esp_timer_start_periodic
static constexpr double CONTROL_LOOP_PERIOD = 0.01;         // seconds, for PID delta_t

// Photoresistors
static constexpr int LEFT_PHOTORESISTOR = 10;
static constexpr int MID_LEFT_PHOTORESISTOR = 11;
static constexpr int MID_RIGHT_PHOTORESISTOR = 12;
static constexpr int RIGHT_PHOTORESISTOR = 13;

// Raw 12-bit ADC threshold (0–4095). Calibrate on your surface.
static constexpr double LIGHT_THRESHOLD_ADC = 2000;
static constexpr double SMALL_ERROR_VALUE = 1;
static constexpr double BIG_ERROR_VALUE = 5;

// Motor PWM channels and pins
static constexpr int WHEEL_1_PWM_CHANNEL_0 = 0;
static constexpr int WHEEL_1_PWM_CHANNEL_1 = 1;
static constexpr int WHEEL_1_PWM_PIN_0 = 2;
static constexpr int WHEEL_1_PWM_PIN_1 = 1;

static constexpr int WHEEL_2_PWM_CHANNEL_0 = 2;
static constexpr int WHEEL_2_PWM_CHANNEL_1 = 3;
static constexpr int WHEEL_2_PWM_PIN_0 = 4;
static constexpr int WHEEL_2_PWM_PIN_1 = 3;

static constexpr int WHEEL_3_PWM_CHANNEL_0 = 4;
static constexpr int WHEEL_3_PWM_CHANNEL_1 = 5;
static constexpr int WHEEL_3_PWM_PIN_0 = 6;    // TODO: conflicts with WHEEL_1_ENCODER_0 — fix wiring
static constexpr int WHEEL_3_PWM_PIN_1 = 5;

// Encoder pins
static constexpr int WHEEL_1_ENCODER_0 = 3;    // TODO: conflicts with WHEEL_3_PWM_PIN_0 — fix wiring
static constexpr int WHEEL_1_ENCODER_1 = 4;

static constexpr int WHEEL_2_ENCODER_0 = 0;    // TODO: conflicts with WHEEL_3_ENCODER_0 — fix wiring
static constexpr int WHEEL_2_ENCODER_1 = 1;

static constexpr int WHEEL_3_ENCODER_0 = 0;    // TODO: conflicts with WHEEL_2_ENCODER_0 — fix wiring
static constexpr int WHEEL_3_ENCODER_1 = 1;

// Distance traveled per encoder tick (m). Set from wheel circumference / ticks-per-rev.
static constexpr double ENCODER_RESOLUTION_DISTANCE_M = 0.3; // TODO: verify this value

// Forward speed during line following (m/s). Tune empirically.
static constexpr double FORWARD_SPEED = 0.3;

// Open-loop forward drive test duration (µs) — see MrKrabs::stepControl.
static constexpr uint64_t FORWARD_DRIVE_TEST_DURATION_US = 1000000; // 1 s

// Converts PID output (m/s) to PWM counts. Tune to match motor transfer function.
static constexpr double VELOCITY_TO_PWM = 200.0;

// Values
static constexpr int speed8bit = 230; // speed scales linear from 0 to 256

// Arm

