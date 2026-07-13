#pragma once

// PID gains for every controller on the robot, gathered here for tuning.
// Constructor order is PidController(p, i, d, max_i) — max_i clamps the
// running integral to [-max_i, max_i] to prevent wind-up.


// Orientation (rotate-to-angle) PID (OrientationController). Error in radians.
static constexpr double ORIENTATION_PID_P = 1;
static constexpr double ORIENTATION_PID_I = 0;
static constexpr double ORIENTATION_PID_D = 0;
static constexpr double ORIENTATION_PID_MAX_I = 1;

// Line-following lateral correction PID (LineFollower). Error from the
// photoresistor array (SMALL_ERROR_VALUE / BIG_ERROR_VALUE units).
static constexpr double LINE_PID_P = 1;
static constexpr double LINE_PID_I = 0;
static constexpr double LINE_PID_D = 2;
static constexpr double LINE_PID_MAX_I = 2;
