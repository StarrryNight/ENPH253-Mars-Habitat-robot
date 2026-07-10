#include <algorithm>
#include <cstdint>
#include <Arduino.h>
#include "HardwareSerial.h"
#include "driver/ledc.h"
#include "constants.h"
#include "esp32-hal-gpio.h"
#include "motor_driver.h"

MotorDriver::MotorDriver(int wheel_number, int pwm_channel_0, int pwm_channel_1, int pwm_pin_0, int pwm_pin_1, int encoder_pin_0, int encoder_pin_1)
	: wheel_number_(wheel_number),
	  pwm_channel_0_(pwm_channel_0),
	  pwm_channel_1_(pwm_channel_1),
	  pwm_pin_0_(pwm_pin_0),
	  pwm_pin_1_(pwm_pin_1),
	  encoder_pin_0_(encoder_pin_0),
	  encoder_pin_1_(encoder_pin_1),
	  wheel_velocity_(0),
	  last_encoder_time_(0),
	  prev_encoder_time_(micros()),
	  encoder_count_(0),
	  prev_measurement_encoder_count_(0),
	  current_motor_speed_(0),
	  last_direction_(0)
{
	ledcSetup(pwm_channel_0_, 1000, 8);
	ledcAttachPin(pwm_pin_0_, pwm_channel_0_);
	ledcWrite(pwm_channel_0_, 0);

	ledcSetup(pwm_channel_1_, 1000, 8);
	ledcAttachPin(pwm_pin_1_, pwm_channel_1_);
	ledcWrite(pwm_channel_1_, 0);

	pinMode(encoder_pin_0_, INPUT_PULLUP);
	// Debounce: ignore pulses less than 3 ms apart to filter mechanical noise.
	attachInterruptArg(encoder_pin_0_, [](void *arg) IRAM_ATTR
					   {
	    MotorDriver* m = static_cast<MotorDriver*>(arg);
	    uint64_t now = micros();
	    if (now - m->last_encoder_time_ < 3000) return;
	    m->last_encoder_time_ = now;
	    m->encoder_count_ += 1; }, this, RISING);
}

void MotorDriver::set_velocity(double speed)
{
	int duty_cycle = speedToDutyCycle(speed);
	int new_direction;
	if (duty_cycle > 0)      new_direction =  1;
	else if (duty_cycle < 0) new_direction = -1;
	else                new_direction =  0;

	if (new_direction == 0)
	{
		ledcWrite(pwm_channel_0_, 0);
		ledcWrite(pwm_channel_1_, 0);
	}
	else if (new_direction == last_direction_)
	{
		// Same direction — update PWM with no delay.
		int pwm = static_cast<int>(duty_cycle > 0 ? duty_cycle : -duty_cycle);
		if (new_direction > 0) ledcWrite(pwm_channel_0_, pwm);
		else                   ledcWrite(pwm_channel_1_, pwm);
	}
	else
	{
		// Direction reversal — coast to stop briefly, then apply new direction.
		ledcWrite(pwm_channel_0_, 0);
		ledcWrite(pwm_channel_1_, 0);
		// TODO: remove delay and use micros or esp timer
		delay(10);
		if (duty_cycle > 0) rotateClockwise(static_cast<int>(duty_cycle));
		else           rotateCounterClockwise(static_cast<int>(-duty_cycle));
	}

	last_direction_ = new_direction;
	current_motor_speed_ = static_cast<int>(duty_cycle);
}

void MotorDriver::rotateClockwise(int duty_cycle)
{
	ledcWrite(pwm_channel_1_, 0);
	ledcWrite(pwm_channel_0_, duty_cycle);
}

void MotorDriver::rotateCounterClockwise(int duty_cycle)
{
	ledcWrite(pwm_channel_0_, 0);
	ledcWrite(pwm_channel_1_, duty_cycle);
}

int MotorDriver::speedToDutyCycle(int speed){
	if (speed==0){
		return 0;
	}
	int abs_speed = std::abs(speed);
	int dir = speed/abs_speed;
	return std::clamp(((abs_speed)+40)*dir,-255, 255);
}
int MotorDriver::get_current_motor_speed()
{
	return current_motor_speed_;
}

int MotorDriver::getEncoderCount()
{
	return encoder_count_;
}

double MotorDriver::tickVelocity()
{
	auto now = micros();
	auto delta_t = (now - prev_encoder_time_) * 1e-6;
	wheel_velocity_ = ((encoder_count_ - prev_measurement_encoder_count_) * ENCODER_RESOLUTION_DISTANCE_M) / delta_t;
	prev_encoder_time_ = now;
	prev_measurement_encoder_count_ = encoder_count_;
	return wheel_velocity_;
}
