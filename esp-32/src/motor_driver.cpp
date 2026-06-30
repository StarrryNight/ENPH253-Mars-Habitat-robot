#include <cstdint>
#include <chrono>
#include <Arduino.h>
#include "HardwareSerial.h"
#include "driver/ledc.h" // package for pwm output
#include "constants.h"
#include "esp32-hal-gpio.h"
#include "esp_timer.h"
#include "motor_driver.h"

MotorDriver::MotorDriver(int wheel_number, int pwm_channel_0, int pwm_channel_1, int pwm_pin_0, int pwm_pin_1, int encoder_pin_0, int encoder_pin_1) : wheel_number_(wheel_number),
																																					   pwm_channel_0_(pwm_channel_0),
																																					   pwm_channel_1_(pwm_channel_1),
																																					   pwm_pin_0_(pwm_pin_0),
																																					   pwm_pin_1_(pwm_pin_1),
																																					   encoder_pin_0_(encoder_pin_0),
																																					   encoder_pin_1_(encoder_pin_1),
																																					   prev_encoder_time_(micros()),
																																					   encoder_count_(0),
																																					   prev_measurement_encoder_count_(0),
																																					   current_motor_speed_(0)
{

	ledcSetup(pwm_channel_0_, 1000, 8); // (pwmchannel to use,  frequency in Hz, number of bits)
	ledcAttachPin(pwm_pin_0_, pwm_channel_0_);
	ledcWrite(pwm_channel_0_, 0);

	ledcSetup(pwm_channel_1_, 1000, 8); // (pwmchannel to use,  frequency in Hz, number of bits)
	ledcAttachPin(pwm_pin_1_, pwm_channel_1_);
	ledcWrite(pwm_channel_1_, 0);

	pinMode(encoder_pin_0_, INPUT_PULLUP);
	attachInterruptArg(encoder_pin_0_, [](void *arg) IRAM_ATTR
					   {
	    MotorDriver* m = static_cast<MotorDriver*>(arg);
	    uint64_t now = micros();
	    if (now - m->last_encoder_time_ < 3000) return; // ignore if <1ms since last tick
	    m->last_encoder_time_ = now;
	    m->encoder_count_ += 1; }, this, RISING);
	attachInterruptArg(encoder_pin_0_, [](void *arg) IRAM_ATTR
					   { static_cast<MotorDriver *>(arg)->encoder_count_ += 1; }, this, RISING);

	esp_timer_create_args_t timer_args = {
		.callback = [](void *arg)
		{
			static_cast<MotorDriver *>(arg)->tickVelocity();
		},
		.arg = this,
		.name = "velocity_counter_timer"};
	esp_timer_create(&timer_args, &velocity_count_timer_);
	esp_timer_start_periodic(velocity_count_timer_, 10000);
}

void MotorDriver::set_velocity(int speed)
{
	if (speed > 0)
	{
		rotateClockwise(speed)
	}
	else
	{
		rotateCounterClockwise(-speed);
	}
	cur
}
void MotorDriver::rotateClockwise(int speed)
{
	ledcWrite(pwm_channel_1_, 0);
	delay(100); // this is a reasonable delay for switching direction
	ledcWrite(pwm_channel_0_, speed);
}
void MotorDriver::rotateCounterClockwise(int speed)
{
	ledcWrite(pwm_channel_0_, 0);
	delay(100); // this is a reasonable delay for switching direction
	ledcWrite(pwm_channel_1_, speed);
}

int MotorDriver::get_current_motor_speed()
{
	return current_motor_speed_;
}
double MotorDriver::getCurrentVelocity()
{
	return 0;
}

double MotorDriver::tickVelocity()
{

	auto now = micros();
	auto delta_t = (now - prev_encoder_time_) * 1e-6;
	wheel_velocity_ = ((encoder_count_ - prev_measurement_encoder_count_) * ENCODER_RESOLUTION_DISTANCE_M) / delta_t;
	prev_encoder_time_ = now;
	prev_measurement_encoder_count_ = encoder_count_;
	return wheel_velocity_;
	Serial.println(encoder_count_);
}
