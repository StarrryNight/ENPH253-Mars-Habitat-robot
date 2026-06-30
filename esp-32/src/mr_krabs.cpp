#include <Arduino.h>
#include "mr_krabs.h"
#include "motor_driver.h"
#include "constants.h"

void MrKrabs::setup()
{
	Serial.begin(115200);
	delay(100);
	Serial.println("ESP32 is ready!");

	// set up motor
	MotorDriver motor_driver_1 = MotorDriver(1, WHEEL_1_PWM_CHANNEL_0, WHEEL_1_PWM_CHANNEL_1, WHEEL_1_PWM_PIN_0, WHEEL_1_PWM_PIN_1, WHEEL_1_ENCODER_0, 0);
	MotorDriver motor_driver_2 = MotorDriver(2, WHEEL_2_PWM_CHANNEL_0, WHEEL_2_PWM_CHANNEL_1, WHEEL_2_PWM_PIN_0, WHEEL_2_PWM_PIN_1, 0, 0);
	motor_driver_1.rotateClockwise(230);
	motor_driver_2.rotateCounterClockwise(230);
	delay(2000);
	motor_driver_1.rotateCounterClockwise(230);
	motor_driver_2.rotateClockwise(230);
	delay(1000);
	motor_driver_1.rotateClockwise(230);
	motor_driver_2.rotateCounterClockwise(230);
	delay(2000);

	// setup line follower
	line_follower_ = LineFollower();
	motor_controller_ = MotorController();
}

void MrKrabs::reset()
{
}

void MrKrabs::update()
{
}

void MrKrabs::stepControl()
{
	if (is_rotating_)
	{
		double horizontal_correction = line_follower_.calculateCorrection();
		RobotVelocity target_velocity = RobotVelocity{0 + horizontal_correction, 2, 0};
		motor_controller_.addVelocity(target_velocity);
	}
	else
	{
		double rotational_correction = orientation_controller_.calculateCorrection();
		RobotVelocity target_velocity = RobotVelocity{0, 0, rotational_correction};
		motor_controller_.addVelocity(target_velocity);
	}
}

void MrKrabs::startRotation()
{
	is_rotating_ = true;
	orientation_controller_.reset();
}

MrKrabs mr_krabs_;

void setup()
{
	mr_krabs_.setup();
}
void loop()
{
	mr_krabs_.update();
}
