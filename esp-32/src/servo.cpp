#include <Arduino.h>
#include "servo.h"

Servo::Servo(int pin, int initial_pulse_us)
	: pin_(pin), pulse_us_(initial_pulse_us), min_pulse_us_(SERVO_MIN_PULSE_US), max_pulse_us_(SERVO_MAX_PULSE_US)
{
	// Configure 50Hz frequency (standard for hobby servos). This core's
	// analogWriteFrequency/Resolution apply globally to all analogWrite pins,
	// not per-pin, so every Servo instance must agree on the same settings.
	analogWriteFrequency(50);

	// Set resolution to 12-bit (gives a duty cycle range of 0 to 4095 for precise timing)
	analogWriteResolution(12);

	// Initialize the position cleanly without breaking PWM configurations
	setPulseUs(pulse_us_);
}

void Servo::setPulseUs(int pulse_us)
{
	// Clamp the input pulse width to safely stay within bounds
	if (pulse_us < min_pulse_us_) pulse_us = min_pulse_us_;
	if (pulse_us > max_pulse_us_) pulse_us = max_pulse_us_;
	pulse_us_ = pulse_us;

	// Calculate the correct duty cycle value based on a 12-bit resolution (0-4095 range).
	// A 50Hz signal has a total period duration of 20,000 microseconds.
	// Formula: (pulse_width / 20000) * 4095
	uint32_t dutyCycle = (pulse_us_ * 4095) / 20000;

	// Safely pass the calculated duty cycle to the hardware peripheral
	analogWrite(pin_, dutyCycle);
}

void Servo::setAngleDeg(double deg)
{
	if (deg < 0.0) deg = 0.0;
	if (deg > 180.0) deg = 180.0;
	
	// Explicitly map 0-180 degrees to your microsecond range
	double percentage = deg / 180.0;
	int calculated_pulse = min_pulse_us_ + (int)((max_pulse_us_ - min_pulse_us_) * percentage);
	
	setPulseUs(calculated_pulse);
}

int Servo::pin() const
{
	return pin_;
}

int Servo::pulseUs() const
{
	return pulse_us_;
}
