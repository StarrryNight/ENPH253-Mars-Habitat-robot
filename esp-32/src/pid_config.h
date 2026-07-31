#pragma once

// PID gains for every controller on the robot, gathered here for tuning.
// Constructor order is PidController(p, i, d, max_i) — max_i clamps the
// running integral to [-max_i, max_i] to prevent wind-up.


// Per-wheel velocity PIDs (MotorController). Error is in m/s; output is
// multiplied by VELOCITY_TO_PWM to get PWM counts. Each wheel gets its own
// gains — the three motors don't respond identically, so tune independently
// on hardware.
#include "constants.h"
static constexpr double WHEEL_LEFT_PID_P = 250;
static constexpr double WHEEL_LEFT_PID_I = 150;
static constexpr double WHEEL_LEFT_PID_D = 1.0;
static constexpr double WHEEL_LEFT_PID_MAX_I = 2000;

static constexpr double WHEEL_RIGHT_PID_P = 250;
static constexpr double WHEEL_RIGHT_PID_I = 150;
static constexpr double WHEEL_RIGHT_PID_D = 1.0;
static constexpr double WHEEL_RIGHT_PID_MAX_I = 2000;

// the back wheel oscillates/overshoots.
static constexpr double WHEEL_BACK_PID_P = 250;
static constexpr double WHEEL_BACK_PID_I = 150;
static constexpr double WHEEL_BACK_PID_D = 1.0;
static constexpr double WHEEL_BACK_PID_MAX_I = 2000;

// Converts a wheel velocity (m/s) into a PWM duty magnitude for
// MotorDriver::set_velocity (which expects commanded PWM, not m/s — see
// MOTOR_SPEED_DEADZONE). Placeholder — tune against real wheel speed on hardware.
static constexpr int VELOCITY_TO_PWM = 250;

// Back-wheel-specific feedforward override — see WHEEL_BACK_PID_* above for
// why this wheel needs more PWM per unit commanded velocity than the front
// two. Used in place of VELOCITY_TO_PWM for the back wheel only, so the PID
// trim has less steady-state error to make up to begin with. Starting point
// (30% over VELOCITY_TO_PWM) — tune against real back-wheel speed on hardware.
static constexpr int WHEEL_BACK_VELOCITY_TO_PWM = 250;

// Per-wheel static PWM offset, added to the duty of every nonzero command for
// that motor (MotorDriver::speedToDutyCycle, passed in via the constructor).
// This is the stiction/drag floor — the duty a wheel needs before it turns at
// all — so raise a wheel's offset if it hums instead of moving at low commanded
// speed. It costs headroom: the command saturates once the feedforward
// (speed * VELOCITY_TO_PWM, or WHEEL_BACK_VELOCITY_TO_PWM for the back wheel)
// exceeds (255 - offset), so at 350 and an offset of 60 the ceiling is
// 0.557 m/s. Values below are the ones tuned on hardware; 60 is where all three
// sat when this was one hardcoded number.
static constexpr int WHEEL_LEFT_PWM_OFFSET = 70;
static constexpr int WHEEL_RIGHT_PWM_OFFSET =60;
static constexpr int WHEEL_BACK_PWM_OFFSET = 65;

// Orientation (rotate-to-angle) PID (OrientationController). Error in radians.
static constexpr double ORIENTATION_PID_P = FORWARD_SPEED*5;
static constexpr double ORIENTATION_PID_I = 0;
static constexpr double ORIENTATION_PID_D = 0.2;
static constexpr double ORIENTATION_PID_MAX_I = 1;

// Heading-hold PID (OrientationController::holdCorrection) — the trim that keeps
// the robot square while a strafe drives it sideways. Error is heading (rad)
// from the encoder-derived rotation estimate, output is omega (rad/s), so P is
// rad/s per rad of error: at P = 1.0 a 10 deg drift buys 0.17 rad/s of
// correction, and MAX_HOLD_OMEGA_RAD_S caps it from ~23 deg up.
//
// Separate from ORIENTATION_PID_* on purpose. Those gains are irrelevant in
// practice (MIN_ROTATION_OMEGA_RAD_S saturates them for every turn the robot
// actually makes), whereas these ones are the only thing setting hold
// behavior — so they need to be tunable without touching rotate-to-angle.
// Leave I at 0: heading error here is a drift to be nulled, and an integral on
// a quantized encoder estimate mostly winds up on noise.
static constexpr double STRAFE_HOLD_PID_P = 1.0;
static constexpr double STRAFE_HOLD_PID_I = 0.0;
static constexpr double STRAFE_HOLD_PID_D = 0.0;
static constexpr double STRAFE_HOLD_PID_MAX_I = 0.0;

// Line-following lateral correction PID (LineFollower). Error from the
// photoresistor array (SMALL_ERROR_VALUE / BIG_ERROR_VALUE units).
static constexpr double LINE_PID_P = 1;
static constexpr double LINE_PID_I = 0.0;
static constexpr double LINE_PID_D = 0;
static constexpr double LINE_PID_MAX_I = 0;
