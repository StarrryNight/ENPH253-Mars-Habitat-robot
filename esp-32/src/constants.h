#pragma once
#include <cstdint>

// Constants used by exactly one file live as static constexpr at the top of
// that file's header (e.g. FORWARD_SPEED in mr_krabs.h, ROTATION_TOLERANCE_RAD
// in orientation_controller.h). Only constants shared across multiple files
// live here.

// General control loop (line following / orientation), driven by esp_timer_start_periodic.
// CONTROL_LOOP_PERIOD (seconds) is derived from the µs constant so the two can never drift apart.
// Shared by MrKrabs (timer period), LineFollower, and OrientationController.
static constexpr int CONTROL_LOOP_PERIOD_US = 10000;        // µs, for esp_timer_start_periodic
static constexpr double CONTROL_LOOP_PERIOD = CONTROL_LOOP_PERIOD_US * 1e-6; // seconds, for PID delta_t

// Motor control loop (velocity PID / encoder sampling) — separate cadence from the loop above,
// kept as its own constant since motor control runs on its own esp_timer.
// Shared by MrKrabs (timer period), MotorController, and MotorDriver.
static constexpr int MOTOR_CONTROL_LOOP_PERIOD_US = 10000;  // µs, for esp_timer_start_periodic
static constexpr double MOTOR_CONTROL_LOOP_PERIOD = MOTOR_CONTROL_LOOP_PERIOD_US * 1e-6; // seconds, for PID delta_t

// Distance traveled per encoder tick (m). Set from wheel circumference / ticks-per-rev.
// Shared by MotorController and MotorDriver.
static constexpr double ENCODER_RESOLUTION_DISTANCE_M = 0.009163; // at 70 mm diameter, 24 ticks-per-rev

// Values
// TODO: unused — no current callers. Remove or wire up.
static constexpr int speed8bit = 230; // speed scales linear from 0 to 256

// Time to block after commanding a new ArmPose so the SCServos/hobby servos
// finish moving before the caller proceeds.
// TODO: unused — the referenced caller (MrKrabs::fullPose) no longer exists.
static constexpr uint32_t SERVO_SETTLE_TIME_MS = 1000;
