#pragma once
#include "pid_controller.h"

// Rotation is considered complete once within this angular distance (rad) of
// the target angle. See OrientationController::reachedTarget.
static constexpr double ROTATION_TOLERANCE_RAD = 0.01;

// Minimum |omega| (rad/s) calculateCorrection() will ever return while actively
// rotating (i.e. reachedTarget() is false), so the PID's proportional term can't
// taper below what it takes to clear MOTOR_SPEED_DEADZONE as heading error
// shrinks — without this floor the wheels get commanded to 0 PWM well before
// ROTATION_TOLERANCE_RAD is reached and the robot just stalls out mid-turn.
// Derived from MOTOR_SPEED_DEADZONE(5) / (VELOCITY_TO_PWM(200) + wheel PID P) /
// WHEEL_DISTANCE_FROM_CENTER_M(0.3) ≈ 0.082 rad/s; rounded up for margin.
static constexpr double MIN_ROTATION_OMEGA_RAD_S = 1.2;

class OrientationController
{
public:
	OrientationController();

	// Returns a rotational velocity (rad/s) correction to drive toward
	// target_angle_, floored in magnitude to MIN_ROTATION_OMEGA_RAD_S so it
	// never silently falls into the motor deadzone while still short of target.
	// current_angle: current heading (rad) from MotorController::computeAngle().
	double calculateCorrection(double current_angle);

	// True once current_angle is within ROTATION_TOLERANCE_RAD of target_angle_.
	bool reachedTarget(double current_angle) const;

	// Sets the target angle (rad) and resets accumulated PID state.
	void startRotation(double target_angle = 0.0);
	void reset();

private:
	PidController orientation_pid;
	double target_angle_;
};
