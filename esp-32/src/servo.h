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
	// Maps an angle (deg) to a raw servo position via linear interpolation
	// between the two measured calibration points [deg_min, deg_max] ->
	// [raw_at_min, raw_at_max]. Caller must clamp deg to [deg_min, deg_max]
	// first — this does no bounds checking of its own.
	static int ServoPositionConversion(double degrees, double deg_min, double deg_max, int raw_at_min, int raw_at_max);

	int servo_1_current_angle_;
	int servo_2_current_angle_;

    SMS_STS bus_servo_ ;
	static constexpr int SERVO_SPEED = 1000;

	// Calibration (measured on hardware): base-arm servo (ID 1), angle from
	// vertical. Raw direction runs opposite the degree axis here — 0 deg is
	// raw 1000, 70 deg is raw 100 (confirmed on hardware: commanding 70 was
	// landing at the physical 0 deg position and vice versa).
	static constexpr double SERVO_1_MIN_DEG = 0.0;   // raw 1000
	static constexpr double SERVO_1_MAX_DEG = 70.0;  // raw 100
	static constexpr int SERVO_1_MIN_RAW = 1200;
	static constexpr int SERVO_1_MAX_RAW = 1850;

	// Calibration (measured on hardware): forearm servo (ID 2), angle down
	// from horizontal.
	static constexpr double SERVO_2_MIN_DEG = 23.0;  // raw 2800, "high"
	static constexpr double SERVO_2_MAX_DEG = 90.0;  // raw 3300, "down"
	static constexpr int SERVO_2_MIN_RAW = 4000;
	static constexpr int SERVO_2_MAX_RAW = 3100;
};
