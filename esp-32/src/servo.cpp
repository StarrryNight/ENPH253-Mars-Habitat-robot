#include <Arduino.h>
#include "ai.h"
#include "pins.h"
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

	// No-op if this is the same pulse we already commanded — avoids
	// resending identical PWM (and re-triggering servo jitter) every cycle.
	if (has_written_ && pulse_us == pulse_us_) return;

	pulse_us_ = pulse_us;
	has_written_ = true;

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





STSServo::STSServo(int pin):servo_1_current_angle_(0),servo_2_current_angle_(0), bus_servo_(){
	Serial2.begin(1000000, SERIAL_8N1, /*rxPin=*/pin, /*txPin=*/pin);
	bus_servo_.pSerial = &Serial2;

	// Force position (servo) mode. MODE (reg 33, EPROM) may be left at 1
	// (wheel/motor mode) from earlier testing (e.g. the Waveshare web UI's
	// "Set Motor Mode" button) — in wheel mode WritePosEx() is silently
	// ignored, which looks exactly like "commands sent, servo never moves".
	// EPROM must be unlocked to write it, then re-locked.
	for (u8 id : {1, 2}) {
		bus_servo_.unLockEprom(id);
		bus_servo_.writeByte(id, SMS_STS_MODE, 0);
		// Min==Max==0 forces motor/wheel behavior on Feetech firmware
		// regardless of MODE — reset to full range so position mode actually
		// sticks. See CalibrationOfs/offset debugging earlier: these may have
		// been left at 0/0 by prior EPROM experiments.
		bus_servo_.writeWord(id, SMS_STS_MIN_ANGLE_LIMIT_L, 0);
		bus_servo_.writeWord(id, SMS_STS_MAX_ANGLE_LIMIT_L, 4095);
		bus_servo_.LockEprom(id);
	}

	// Torque Switch (reg 40, SRAM) resets to disabled on every power-cycle —
	// without this, WritePosEx() writes the goal position but the servo stays
	// physically disengaged and never moves toward it.
	bus_servo_.EnableTorque(1, 1);
	bus_servo_.EnableTorque(2, 1);
};
void STSServo::setAngle(double servo_1_angle, double servo_2_angle, int servo_1_speed, int servo_2_speed){
	// Reject anything outside the measured calibration range instead of
	// silently clamping — an out-of-range request is a caller bug worth
	// noticing, not something to quietly reinterpret.
	if (servo_1_angle < SERVO_1_MIN_DEG || servo_1_angle > SERVO_1_MAX_DEG ||
	    servo_2_angle < SERVO_2_MIN_DEG || servo_2_angle > SERVO_2_MAX_DEG){
		return;
	}
	int servo_1_pos = ServoPositionConversion(servo_1_angle, SERVO_1_MIN_DEG, SERVO_1_MAX_DEG, SERVO_1_MIN_RAW, SERVO_1_MAX_RAW);
	int servo_2_pos = ServoPositionConversion(servo_2_angle, SERVO_2_MIN_DEG, SERVO_2_MAX_DEG, SERVO_2_MIN_RAW, SERVO_2_MAX_RAW);
	// Speed is an unsigned magnitude in position mode — the servo works out
	// direction itself from current vs. goal position.
	bus_servo_.WritePosEx(/*ID=*/1, /*Position=*/servo_1_pos, /*Speed=*/servo_1_speed, /*ACC=*/50);
	bus_servo_.WritePosEx(/*ID=*/2, /*Position=*/servo_2_pos, /*Speed=*/servo_2_speed, /*ACC=*/50);
	servo_1_current_angle_ = servo_1_pos;
	servo_2_current_angle_ = servo_2_pos;
}
int STSServo::ServoPositionConversion(double degrees, double deg_min, double deg_max, int raw_at_min, int raw_at_max){
	double t = (degrees - deg_min) / (deg_max - deg_min);
	return raw_at_min + static_cast<int>(t * (raw_at_max - raw_at_min));
}
