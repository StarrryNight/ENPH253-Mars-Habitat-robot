#pragma once
#include "line_follower.h"
#include "motor_controller.h"
#include "motor_driver.h"
class MrKrab
{

public:
	MrKrab();
	void setup();
	void reset();
	void update();

private:
	void stepControl();
	LineFollower line_follower_;
	MotorController motor_controller_;
	MotorDriver motor_driver_1_;
	MotorDriver motor_driver_2_;
};
