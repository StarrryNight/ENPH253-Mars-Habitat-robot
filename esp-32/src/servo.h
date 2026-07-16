#pragma once
#include <SMS_STS.h>

static constexpr int SERVO_MIN_PULSE_US = 500;
static constexpr int SERVO_MAX_PULSE_US = 2500;
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

class STSServo
{

	public:
	STSServo(int pin);
void setAngle(double servo_1_angle, double servo_2_angle);
	
	private:
int ServoPositionConversion(double degrees);
	int servo_1_current_angle_;
	int servo_2_current_angle_;

    SMS_STS bus_servo_ ;
	static constexpr int SERVO_SPEED = 2400;
	static constexpr int DIRECTION_FACTOR = 32768;
	static constexpr int SERIAL_SERVO_MIN = 0;
	static constexpr int SERIAL_SERVO_MAX = 2048;
};
