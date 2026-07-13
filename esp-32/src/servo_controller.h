#pragma once
#include "servo.h"

class ServoController
{

public:
	ServoController();

	bool addServo(Servo *servo);
	void update();

private:
	static constexpr int kMaxServos = 8;
	static constexpr unsigned long kCyclePeriodUs = 20000;

	Servo *servos_[kMaxServos];
	int servo_count_;
	unsigned long last_cycle_time_us_;
};
