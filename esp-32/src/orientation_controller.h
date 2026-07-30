#pragma once
#include "pid_controller.h"

// Rotation is considered complete once within this angular distance (rad) of
// the target angle. See OrientationController::reachedTarget.
static constexpr double ROTATION_TOLERANCE_RAD = 0.05;

// Minimum |omega| (rad/s) calculateCorrection() will ever return while actively
// rotating (i.e. reachedTarget() is false), so the PID's proportional term can't
// taper below what it takes to clear MOTOR_SPEED_DEADZONE as heading error
// shrinks — without this floor the wheels get commanded to 0 PWM well before
// ROTATION_TOLERANCE_RAD is reached and the robot just stalls out mid-turn.
// Derived from MOTOR_SPEED_DEADZONE(5) / (VELOCITY_TO_PWM(200) + wheel PID P) /
// WHEEL_DISTANCE_FROM_CENTER_M(0.3) ≈ 0.082 rad/s; rounded up for margin.
static constexpr double MIN_ROTATION_OMEGA_RAD_S = 1.0;

// Ceiling on |omega| (rad/s) from holdCorrection(). A heading hold runs
// *alongside* a translation command, so its authority has to stay small next to
// it: every wheel sees R*omega = WHEEL_DISTANCE_FROM_CENTER_M * omega added to
// its translation share, and in a pure strafe the front wheels only get half the
// commanded speed (0.065 m/s at HABITAT_STRAFE_SPEED). At 0.4 rad/s the yaw term
// is 0.043 m/s — two thirds of that, about the practical limit before a front
// wheel crosses zero, trips MotorDriver's direction-change guard and coasts
// mid-strafe.
static constexpr double MAX_HOLD_OMEGA_RAD_S = 0.4;

// Heading error (rad) inside which holdCorrection() commands nothing. Not a
// completion test (a hold never completes) but a deadband, and sized off encoder
// resolution rather than off how straight we would like to be: one encoder tick
// on one wheel is ENCODER_RESOLUTION_DISTANCE_M / (3 * WHEEL_DISTANCE_FROM_CENTER_M)
// = 0.0297 rad (1.7 deg) of apparent yaw, so a tighter band than a couple of
// ticks just chases quantization noise and chatters.
static constexpr double HOLD_DEADBAND_RAD = 0.03;

class OrientationController
{
public:
	OrientationController();

	// Returns a rotational velocity (rad/s) correction to drive toward
	// target_angle_, floored in magnitude to MIN_ROTATION_OMEGA_RAD_S so it
	// never silently falls into the motor deadzone while still short of target.
	// current_angle: current heading (rad) from MotorController::computeAngle().
	double calculateCorrection(double current_angle);

	// Returns a small rotational velocity (rad/s) that holds current_angle at
	// target_angle_ while some other velocity component does the actual driving —
	// use this, never calculateCorrection(), for a heading hold during a strafe
	// or straight leg. Differs from calculateCorrection() in every way that
	// matters for that job: no MIN_ROTATION_OMEGA_RAD_S floor (which would slam
	// 1.5 rad/s at arbitrarily small error), clamped to MAX_HOLD_OMEGA_RAD_S,
	// dead inside HOLD_DEADBAND_RAD, and driven by its own PID instance and gains
	// (STRAFE_HOLD_PID_* in pid_config.h) so trim tuning and rotate-to-angle
	// tuning stay independent.
	//
	// current_angle: heading (rad) from MotorController::getElapsedRotation(),
	// which the caller must have zeroed via MotorController::startRotation() when
	// the leg began — this holds relative to that zero, not to any field frame.
	double holdCorrection(double current_angle);

	// True once current_angle is within ROTATION_TOLERANCE_RAD of target_angle_.
	bool reachedTarget(double current_angle) const;

	// Sets the target angle (rad) and resets accumulated PID state.
	void startRotation(double target_angle = 0.0);
	void reset();

private:
	PidController orientation_pid;
	// Separate instance for holdCorrection(): different gains, and a hold runs
	// for the whole length of a leg, so sharing accumulated state with the
	// rotate-to-angle PID would let each one's history perturb the other.
	PidController hold_pid_;
	double target_angle_;
};
