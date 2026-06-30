#pragma once
#include "motor_driver.h"
#include "pid_controller.h"
#include <optional>
#include <chrono>

// Per-wheel speed targets in m/s (positive = forward for that wheel's orientation).
struct WheelVelocities
{
	double wheel_1; // front-left (150°)
	double wheel_2; // front-right (30°)
	double wheel_3; // back (270°)
};

// Robot-frame velocity in SI units.
// x: rightward (m/s), y: forward (m/s), omega: counter-clockwise (rad/s).
struct RobotVelocity
{
	double x;
	double y;
	double omega;

	RobotVelocity operator+(const RobotVelocity &other) const
	{
		return {x + other.x, y + other.y, omega + other.omega};
	}
};

// Closed-loop velocity controller for the kiwi (3-omni-wheel) drivetrain.
// Accepts robot-frame velocities, converts to per-wheel targets via inverse
// kinematics, then runs independent PID loops per wheel each call to setVelocity.
class MotorController
{
public:
	MotorController();
	void setup();

	// Replaces the current velocity target and runs one PID step.
	void setVelocity(RobotVelocity target_velocity);

	// Adds correction_velocity on top of the stored target and runs one PID step.
	// Used by line follower and orientation controller to nudge the robot without
	// overwriting the base motion command.
	void addVelocity(RobotVelocity correction_velocity);

private:
	// Kiwi-drive inverse kinematics: maps robot-frame (x, y, omega) to wheel speeds.
	// Wheel angles: wheel_1=150°, wheel_2=30°, wheel_3=270° from +Y (forward).
	// Positive wheel velocity = counter-clockwise (viewed from outside the robot).
	WheelVelocities euclideanToWheel(RobotVelocity target_velocity);
	WheelVelocities getWheelVelocities();

	PidController wheel_1_pid_;
	PidController wheel_2_pid_;
	PidController wheel_3_pid_;

	MotorDriver wheel_1_motor_;
	MotorDriver wheel_2_motor_;
	MotorDriver wheel_3_motor_;

	double current_position_;
	RobotVelocity current_target_velocity;
	WheelVelocities current_wheel_velocities_;
	std::optional<std::chrono::time_point<std::chrono::steady_clock>> prev_step_time_;

	// Distance from robot center to each wheel contact point (m).
	// Used to scale omega into a linear wheel speed contribution.
	static constexpr double WHEEL_DISTANCE_FROM_CENTER_M = 0.3;
};
