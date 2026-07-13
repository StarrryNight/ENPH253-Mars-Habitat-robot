#include <Arduino.h>
#include "servo_controller.h"

ServoController::ServoController()
	: servo_count_(0), last_cycle_time_us_(0) {}

bool ServoController::addServo(Servo *servo)
{
	if (servo_count_ >= kMaxServos) return false;
	servos_[servo_count_] = servo;
	servo_count_++;
	return true;
}

void ServoController::update()
{
	unsigned long now = micros();
	if (now - last_cycle_time_us_ < kCyclePeriodUs) return;
	last_cycle_time_us_ = now;

	for (int i = 0; i < servo_count_; i++)
		digitalWrite(servos_[i]->pin(), HIGH);

	bool all_done = false;
	while (!all_done)
	{
		all_done = true;
		unsigned long elapsed = micros() - last_cycle_time_us_;

		for (int i = 0; i < servo_count_; i++)
		{
			if (elapsed >= (unsigned long)servos_[i]->pulseUs())
				digitalWrite(servos_[i]->pin(), LOW);
			else
				all_done = false;
		}
	}
}
