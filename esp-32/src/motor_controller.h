#pragma once
#include "motor_driver.h"
#include "pid_controller.h"
#include <optional>
#include <chrono>

struct WheelVelocities
{
	double wheel_1;
	double wheel_2;
	double wheel_3;
};

struct RobotVelocity
{
	double x;
	double y;
	double omega;
};

class MotorController
{
public:
	MotorController();
	void setup();
	void setVelocity(RobotVelocity target_velocity);
	void addVelocity(RobotVelocity correction_velocity);

private:
	// positive is anti_clockwise
	//
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

	static constexpr double WHEEL_DISTANCE_FROM_CENTER_M = 0.3;
};
