#include <Arduino.h>
<<<<<<< HEAD:esp-32/src/mr_krabs.cpp
#include "mr_krabs.h"
#include "motor_driver.h"
=======
#include "mr_krab.h"
>>>>>>> 028a5a6c2b69952a0f99a2081b56e3d5a0a858a7:esp-32/src/mr_krab.cpp
#include "constants.h"
#include "motor_controller.h"

MrKrab::MrKrab()
	: motor_driver_1_(1, WHEEL_1_PWM_CHANNEL_0, WHEEL_1_PWM_CHANNEL_1, WHEEL_1_PWM_PIN_0, WHEEL_1_PWM_PIN_1, WHEEL_1_ENCODER_0, 0),
	  motor_driver_2_(2, WHEEL_2_PWM_CHANNEL_0, WHEEL_2_PWM_CHANNEL_1, WHEEL_2_PWM_PIN_0, WHEEL_2_PWM_PIN_1, 0, 0),
	  motor_controller_()
{}

void MrKrab::setup()
{
	Serial.begin(115200);
	delay(100);
	Serial.println("ESP32 is ready!");
<<<<<<< HEAD:esp-32/src/mr_krabs.cpp

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
=======
	motor_controller_.begin();
	motor_controller_.setVelocity(-1,-1,0);
	delay(3000);
	motor_controller_.setVelocity(1,1,0);
//	motor_controller_.setVelocity(-1,-1,0);
//	motor_driver_1_.begin();
//
//	motor_driver_1_.rotateClockwise(230);
//	delay(3000);
//	motor_driver_1_.rotateCounterClockwise(230);
>>>>>>> 028a5a6c2b69952a0f99a2081b56e3d5a0a858a7:esp-32/src/mr_krab.cpp
}

void MrKrab::reset()
{
}

void MrKrab::update()
{
}

void MrKrab::stepControl()
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

void MrKrab::startRotation()
{
	is_rotating_ = true;
	orientation_controller_.reset();
}

MrKrab mr_krab_;

void setup()
{
	mr_krab_.setup();
}
void loop()
{
	mr_krab_.update();
}
