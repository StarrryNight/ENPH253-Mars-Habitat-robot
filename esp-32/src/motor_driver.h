#pragma once
#include <Arduino.h>
#include <cstdint>

// Controls a single DC motor with quadrature encoder feedback.
// PWM is driven via ESP32 LEDC channels. Velocity is measured by tickVelocity(),
// which is called once per control loop tick (every 10 ms) by MotorController —
// no separate per-motor timer is used.
// All setup happens in the constructor — no separate begin() call needed.
class MotorDriver
{

public:
	// pwm_channel_0/1: LEDC channels (must be unique per motor).
	// pwm_pin_0 drives forward, pwm_pin_1 drives reverse.
	// encoder_pin_0: single-phase encoder input (RISING edge counted).
	MotorDriver(int wheel_number, int pwm_channel_0, int pwm_channel_1, int pwm_pin_0, int pwm_pin_1, int encoder_pin_0);

	// Direct PWM control. Zeroes the opposite channel before applying speed.
	// These do NOT include a direction-change delay; use set_velocity for the
	// closed-loop path which handles H-bridge protection automatically.
	void rotateClockwise(int speed);
	void rotateCounterClockwise(int speed);

	// Signed wrapper: speed > 0 = clockwise, speed < 0 = counter-clockwise, 0 = stop.
	// Applies a 10 ms coast-to-stop delay only when direction actually reverses,
	// protecting the H-bridge from shoot-through without blocking every tick.
	void set_velocity(double speed);

	// Returns the last PWM value written by set_velocity.
	int get_current_motor_speed();

	// Returns the raw cumulative encoder count since construction (monotonic, unsigned).
	int getEncoderCount();

	// Computes wheel surface speed (m/s) from encoder count delta since last call.
	// Must be called once per control loop tick (10 ms) by MotorController.
	void tickSpeed();

	double getSpeed();

private:
	int speedToDutyCycle(int speed);

	const int wheel_number_;


	volatile double wheel_speed_;
	int current_motor_speed_;
	int last_direction_;                  // -1, 0, or 1 — tracks last direction for H-bridge protection

	const int pwm_channel_0_;
	const int pwm_channel_1_;

	const int pwm_pin_0_;
	const int pwm_pin_1_;

	const int encoder_pin_0_;

	volatile int encoder_count_;
	int prev_measurement_encoder_count_; // snapshot taken each tickVelocity call
										 //
	uint64_t prev_pwm_reset_time_;
	bool pending_direction_change_;

	static constexpr uint64_t SHOOTTHROUGH_GUARD_THRESHOLD_US = 10000;
};
