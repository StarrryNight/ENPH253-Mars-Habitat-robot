#pragma once
#include <Arduino.h>
#include <chrono>
#include <cstdint>
#include <optional>
#include "esp_timer.h"

// Controls a single DC motor with quadrature encoder feedback.
// PWM is driven via ESP32 LEDC channels. Velocity is measured by a periodic
// esp_timer ISR (tickVelocity) rather than per-interrupt to avoid ISR overhead.
// All setup happens in the constructor — no separate begin() call needed.
class MotorDriver
{

public:
	// pwm_channel_0/1: LEDC channels (must be unique per motor).
	// pwm_pin_0 drives forward, pwm_pin_1 drives reverse.
	// encoder_pin_0: single-phase encoder input (RISING edge counted).
	MotorDriver(int wheel_number, int pwm_channel_0, int pwm_channel_1, int pwm_pin_0, int pwm_pin_1, int encoder_pin_0, int encoder_pin_1);

	// Direct PWM control. Zeroes the opposite channel first; includes a 100 ms
	// direction-change delay to protect the H-bridge from shoot-through.
	void rotateClockwise(int speed);
	void rotateCounterClockwise(int speed);

	// Signed wrapper around rotateClockwise/CounterClockwise and records current speed.
	// speed > 0 = clockwise, speed < 0 = counter-clockwise.
	void set_velocity(int speed);

	// Returns the last PWM value written by set_velocity.
	int get_current_motor_speed();

	// Called by the periodic esp_timer ISR every 10 ms.
	// Computes wheel surface velocity (m/s) from encoder count delta and elapsed time.
	double tickVelocity();

private:
	const int wheel_number_;

	volatile double wheel_velocity_;
	// TODO change naming
	int current_motor_speed_;
	volatile uint64_t last_encoder_time_; // µs; used to debounce encoder ISR

	const int pwm_channel_0_;
	const int pwm_channel_1_;

	const int pwm_pin_0_;
	const int pwm_pin_1_;

	const int encoder_pin_0_;
	const int encoder_pin_1_;

	volatile int encoder_count_;
	int prev_measurement_encoder_count_; // snapshot taken each tickVelocity call
	uint64_t prev_encoder_time_;         // µs timestamp of last tickVelocity call

	esp_timer_handle_t velocity_count_timer_;
};
