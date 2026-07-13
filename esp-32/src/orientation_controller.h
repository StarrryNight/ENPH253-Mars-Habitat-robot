#pragma once
#include "pid_controller.h"


class OrientationController
{
public:
	OrientationController();

	// Returns a rotational velocity (rad/s) correction to drive toward target_angle_.
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
