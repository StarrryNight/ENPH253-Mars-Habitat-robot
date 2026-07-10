#pragma once

static constexpr int SERVO_MIN_PULSE_US = 1000;
static constexpr int SERVO_MAX_PULSE_US = 2000;
static constexpr int SERVO_CENTER_PULSE_US = 1500;

class Servo
{

public:
	Servo(int pin, int initial_pulse_us = SERVO_CENTER_PULSE_US);

	void setPulseUs(int pulse_us);
	void setAngleDeg(double deg);

	int pin() const;
	int pulseUs() const;

private:
	const int pin_;
	int pulse_us_;
	const int min_pulse_us_;
	const int max_pulse_us_;
};
