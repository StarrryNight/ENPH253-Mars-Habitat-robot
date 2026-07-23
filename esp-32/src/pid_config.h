#pragma once

// PID gains for every controller on the robot, gathered here for tuning.
// Constructor order is PidController(p, i, d, max_i) — max_i clamps the
// running integral to [-max_i, max_i] to prevent wind-up.


// Per-wheel velocity PIDs (MotorController). Error is in m/s; output is
// multiplied by VELOCITY_TO_PWM to get PWM counts. Each wheel gets its own
// gains — the three motors don't respond identically, so tune independently
// on hardware.
#include "constants.h"
static constexpr double WHEEL_LEFT_PID_P = 30;
static constexpr double WHEEL_LEFT_PID_I = 0;
static constexpr double WHEEL_LEFT_PID_D = 0.10;
static constexpr double WHEEL_LEFT_PID_MAX_I = 2;

static constexpr double WHEEL_RIGHT_PID_P = 30;
static constexpr double WHEEL_RIGHT_PID_I = 0;
static constexpr double WHEEL_RIGHT_PID_D = 0.1;
static constexpr double WHEEL_RIGHT_PID_MAX_I = 2;

// the back wheel oscillates/overshoots.
static constexpr double WHEEL_BACK_PID_P = 30;
static constexpr double WHEEL_BACK_PID_I = 0;
static constexpr double WHEEL_BACK_PID_D = 0.10;
static constexpr double WHEEL_BACK_PID_MAX_I = 2;

// Converts a wheel velocity (m/s) into a PWM duty magnitude for
// MotorDriver::set_velocity (which expects commanded PWM, not m/s — see
// MOTOR_SPEED_DEADZONE). Placeholder — tune against real wheel speed on hardware.
static constexpr int VELOCITY_TO_PWM = 300;

// Back-wheel-specific feedforward override — see WHEEL_BACK_PID_* above for
// why this wheel needs more PWM per unit commanded velocity than the front
// two. Used in place of VELOCITY_TO_PWM for the back wheel only, so the PID
// trim has less steady-state error to make up to begin with. Starting point
// (30% over VELOCITY_TO_PWM) — tune against real back-wheel speed on hardware.
static constexpr int WHEEL_BACK_VELOCITY_TO_PWM = 300;
// Orientation (rotate-to-angle) PID (OrientationController). Error in radians.
static constexpr double ORIENTATION_PID_P = FORWARD_SPEED*4;
static constexpr double ORIENTATION_PID_I = 0;
static constexpr double ORIENTATION_PID_D = 0.2;
static constexpr double ORIENTATION_PID_MAX_I = 1;

// Line-following lateral correction PID (LineFollower). Error from the
// photoresistor array (SMALL_ERROR_VALUE / BIG_ERROR_VALUE units).
static constexpr double LINE_PID_P = 1;
static constexpr double LINE_PID_I = 0.0;
static constexpr double LINE_PID_D = 0;
static constexpr double LINE_PID_MAX_I = 0;
