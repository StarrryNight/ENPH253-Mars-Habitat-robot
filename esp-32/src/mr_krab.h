#pragma once
#include <optional>
#include "line_follower.h"
#include "motor_controller.h"
#include "motor_driver.h"
class MrKrab
{

public:
	void setup();

	void reset();

	void update();

private:
	void stepControl();
	LineFollower line_follower_;
	MotorController motor_controller_;
	std::optional<MotorDriver> motor_driver_1_;
	std::optional<MotorDriver> motor_driver_2_;
};
