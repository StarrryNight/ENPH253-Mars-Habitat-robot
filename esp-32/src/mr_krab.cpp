#include <Arduino.h>
#include "mr_krab.h"
#include "motor_driver.h"
#include "constants.h"

void MrKrab::setup()
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
}

void MrKrab::reset()
{
}

void MrKrab::update()
{
}

void MrKrab::stepControl()
{
	line_follower_.followLine();
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
