#pragma once
#include <Arduino.h>
#include <chrono>
#include <cstdint>
#include <optional>
#include "esp_timer.h"

class MotorDriver{

public:
	MotorDriver(int wheel_number, int pwm_channel_0, int pwm_channel_1,int pwm_pin_0, int pwm_pin_1  ,int encoder_pin_0, int encoder_pin_1);
	void rotateClockwise(int	speed);
	void rotateCounterClockwise(int	speed);

	double getCurrentVelocity();

	private:

	void IRAM_ATTR tickVelocity();
	const int wheel_number_;

	volatile double wheel_velocity_;

	volatile uint64_t last_encoder_time_;

	const int pwm_channel_0_;
	const int pwm_channel_1_;

	const int pwm_pin_0_;
	const int pwm_pin_1_;
	
	const int encoder_pin_0_;	
	const int encoder_pin_1_;	

	volatile int encoder_count_;
	int prev_measurement_encoder_count_;
	uint64_t  prev_encoder_time_;

	esp_timer_handle_t velocity_count_timer_;
	

	
};
