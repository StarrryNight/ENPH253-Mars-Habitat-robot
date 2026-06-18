#include <Arduino.h>
#include "mr_krab.h"
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
