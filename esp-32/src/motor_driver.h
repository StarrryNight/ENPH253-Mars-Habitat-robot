#pragma once
#include <Arduino.h>
#include <chrono>
#include <cstdint>
#include <optional>
#include "esp_timer.h"

class MotorDriver
{

public:
<<<<<<< HEAD
	MotorDriver(int wheel_number, int pwm_channel_0, int pwm_channel_1, int pwm_pin_0, int pwm_pin_1, int encoder_pin_0, int encoder_pin_1);
	void rotateClockwise(int speed);
	void rotateCounterClockwise(int speed);
=======
	MotorDriver(int wheel_number, int pwm_channel_0, int pwm_channel_1,int pwm_pin_0, int pwm_pin_1  ,int encoder_pin_0, int encoder_pin_1);
	void begin();
	void rotateClockwise(int	speed);
	void rotateCounterClockwise(int	speed);
>>>>>>> 028a5a6c2b69952a0f99a2081b56e3d5a0a858a7

	void set_velocity(int speed);
	int get_current_motor_speed();
	double tickVelocity();

private:
	const int wheel_number_;

	volatile double wheel_velocity_;
	// TODO change naming
	int current_motor_speed_;
	volatile uint64_t last_encoder_time_;

	const int pwm_channel_0_;
	const int pwm_channel_1_;

	const int pwm_pin_0_;
	const int pwm_pin_1_;

	const int encoder_pin_0_;
	const int encoder_pin_1_;

	volatile int encoder_count_;
	int prev_measurement_encoder_count_;
	uint64_t prev_encoder_time_;

	esp_timer_handle_t velocity_count_timer_;
};
