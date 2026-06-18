#pragma once
#include "motor_driver.h"
#include "pid_controller.h"
#include <optional>
#include <chrono>

struct WheelVelocities{
	double wheel_1;
	double wheel_2;
	double wheel_3;
};

class MotorController{
	public:
		MotorController();
		void setup();	
		void setVelocity(double target_velocity_x, double target_velocity_y, double target_angular_velocity);
		

	private:



	
		
		// positive is anti_clockwise
		//
		WheelVelocities euclideanToWheel(double velocity_x, double velocity_y, double angular_velocity);
		WheelVelocities getWheelVelocities();

		PidController wheel_1_pid_;
		PidController wheel_2_pid_;
		PidController wheel_3_pid_;

		MotorDriver wheel_1_motor_;
		MotorDriver wheel_2_motor_;
		MotorDriver wheel_3_motor_;

		double current_position_;
		WheelVelocities current_wheel_velocities_;
		
		static constexpr double WHEEL_DISTANCE_FROM_CENTER_M = 0.3;


};
