#include <Arduino.h>
#include "servo.h"

Servo::Servo(int pin, int initial_pulse_us)
	: pin_(pin), pulse_us_(initial_pulse_us), min_pulse_us_(SERVO_MIN_PULSE_US), max_pulse_us_(SERVO_MAX_PULSE_US)
{
	pinMode(pin_, OUTPUT);
	digitalWrite(pin_, LOW);
}

void Servo::setPulseUs(int pulse_us)
{
	if (pulse_us < min_pulse_us_) pulse_us = min_pulse_us_;
	if (pulse_us > max_pulse_us_) pulse_us = max_pulse_us_;
	pulse_us_ = pulse_us;
}

void Servo::setAngleDeg(double deg)
{
	if (deg < 0) deg = 0;
	if (deg > 180) deg = 180;
	setPulseUs(min_pulse_us_ + (int)((max_pulse_us_ - min_pulse_us_) * (deg / 180.0)));
}

int Servo::pin() const
{
	return pin_;
}

int Servo::pulseUs() const
{
	return pulse_us_;
}
