#include <algorithm>
#include <cstdint>
#include <exception>
#include <ratio>
#include <Arduino.h>
#include "HardwareSerial.h"
#include "driver/ledc.h"
#include "constants.h"
#include "esp32-hal-gpio.h"
#include "esp_timer.h"
#include "motor_driver.h"

MotorDriver::MotorDriver(int wheel_number, int pwm_channel_0, int pwm_channel_1, int pwm_pin_0, int pwm_pin_1, int encoder_pin_0,
                         int pwm_offset)
	: wheel_number_(wheel_number),
	  pwm_channel_0_(pwm_channel_0),
	  pwm_channel_1_(pwm_channel_1),
	  pwm_pin_0_(pwm_pin_0),
	  pwm_pin_1_(pwm_pin_1),
	  encoder_pin_0_(encoder_pin_0),
	  pwm_offset_(pwm_offset),
	  wheel_speed_(0),
	  encoder_count_(0),
	  prev_measurement_encoder_count_(0),
	  motor_target_speed_(0),
	  last_direction_(0),
	  prev_pwm_reset_time_(0),
	  pending_direction_change_(false),
	  velocity_count_buffer_{},
	  buffer_index_(0),
	  buffer_filled_count_(0),
	  last_delta_count_(0)
{
	ledcSetup(pwm_channel_0_, 1000, 8);
	ledcAttachPin(pwm_pin_0_, pwm_channel_0_);
	ledcWrite(pwm_channel_0_, 0);

	ledcSetup(pwm_channel_1_, 1000, 8);
	ledcAttachPin(pwm_pin_1_, pwm_channel_1_);
	ledcWrite(pwm_channel_1_, 0);

	pinMode(encoder_pin_0_, INPUT_PULLUP);
	// Debounce: ignore pulses less than 3 ms apart to filter mechanical noise.
	attachInterruptArg(encoder_pin_0_, [](void *arg) IRAM_ATTR
					   {
	    MotorDriver* m = static_cast<MotorDriver*>(arg);
	    m->encoder_count_ += 1; }, this, RISING);
}

void MotorDriver::set_velocity(double speed)
{
    int duty_cycle = speedToDutyCycle(speed);
    int new_direction;
    if (duty_cycle == 0)   new_direction =  0;
    else if (speed > 0)    new_direction =  1;
    else                   new_direction = -1;

	//Serial.printf("duty_cycle: %d\n",duty_cycle*new_direction);
    // CAPTURE INTENT IMMEDIATELY: Tell the encoder math exactly what the PID wants right now
    intended_direction_ = new_direction;

    // Record the raw commanded speed unconditionally, even when the deadzone below
    // zeroes the actual PWM write, so getCurrentTargetSpeed() reflects what was
    // actually requested rather than what got written to the pins.
    motor_target_speed_ = speed;

    if (pending_direction_change_)
    {
        if (static_cast<uint64_t>(esp_timer_get_time()) - prev_pwm_reset_time_ > SHOOTTHROUGH_GUARD_THRESHOLD_US)
        {
            if (new_direction > 0)       rotateClockwise(duty_cycle);
            else if (new_direction < 0)  rotateCounterClockwise(duty_cycle);

            last_direction_ = new_direction;
            pending_direction_change_ = false;

            std::fill_n(velocity_count_buffer_, VELOCITY_BUFFER_SIZE, 0.0);
            buffer_index_ = 0;
            buffer_filled_count_ = 0;
        }
        else
        {
            ledcWrite(pwm_channel_0_, 0);
            ledcWrite(pwm_channel_1_, 0);
        }
        return;
    }

    if (new_direction == 0)
    {
        ledcWrite(pwm_channel_0_, 0);
        ledcWrite(pwm_channel_1_, 0);
        // Deliberately NOT resetting last_direction_ here: tickSpeed()'s sign
        // fallback reads it whenever intended_direction_ is 0, and a command
        // dithering at the deadzone shouldn't discard real encoder counts by
        // forcing that fallback to 0 too. A genuinely stopped wheel already
        // reads 0 velocity because delta_count itself is 0.
    }
    else if (new_direction == last_direction_)
    {
        if (new_direction > 0) ledcWrite(pwm_channel_0_, duty_cycle);
        else                   ledcWrite(pwm_channel_1_, duty_cycle);
    }
    else
    {
        ledcWrite(pwm_channel_0_, 0);
        ledcWrite(pwm_channel_1_, 0);
        prev_pwm_reset_time_ = static_cast<uint64_t>(esp_timer_get_time());
        pending_direction_change_ = true;
    }
}

void MotorDriver::rotateClockwise(int duty_cycle)
{
	ledcWrite(pwm_channel_1_, 0);
	ledcWrite(pwm_channel_0_, duty_cycle);
}

void MotorDriver::rotateCounterClockwise(int duty_cycle)
{
	ledcWrite(pwm_channel_0_, 0);
	ledcWrite(pwm_channel_1_, duty_cycle);
}

int MotorDriver::speedToDutyCycle(int speed){
	int abs_speed = static_cast<int>(std::abs(speed));
	if (abs_speed < MOTOR_SPEED_DEADZONE){
		return 0;
	}
	return std::clamp(abs_speed + pwm_offset_, 0, 255);
}
double MotorDriver::getCurrentTargetSpeed()
{
	return motor_target_speed_;
}

void MotorDriver::primeForRestart()
{
	last_direction_ = 0;
	// Mirrors the direction-reversal branch in set_velocity() — without this,
	// stale pre-stop samples linger in the moving average and getSpeed()
	// keeps reporting decaying nonzero velocity for VELOCITY_BUFFER_SIZE
	// ticks after a real stop (MotorController::stopImmediate calls this).
	std::fill_n(velocity_count_buffer_, VELOCITY_BUFFER_SIZE, 0.0);
	buffer_index_ = 0;
	buffer_filled_count_ = 0;
}

uint32_t MotorDriver::getEncoderCount()
{
	return encoder_count_;
}

void MotorDriver::resetSpeedBaseline()
{
    noInterrupts();
    uint32_t current_count = encoder_count_;
    interrupts();
    prev_measurement_encoder_count_ = current_count;
}

void MotorDriver::tickSpeed()
{
    noInterrupts();
    uint32_t current_count = encoder_count_;
    interrupts();

    uint32_t delta_count = current_count - prev_measurement_encoder_count_;
    prev_measurement_encoder_count_ = current_count;
    last_delta_count_ = delta_count;

    double raw_speed = (static_cast<double>(delta_count) * ENCODER_RESOLUTION_DISTANCE_M) / MOTOR_CONTROL_LOOP_PERIOD;

    // Use the immediate intended direction to break out-of-phase oscillation
    int current_sign = intended_direction_;
    if (current_sign == 0) {
        current_sign = last_direction_; // Fallback only if stopping entirely
    }

    velocity_count_buffer_[buffer_index_] = raw_speed * current_sign;
    buffer_index_ = (buffer_index_ + 1) % VELOCITY_BUFFER_SIZE;
    if (buffer_filled_count_ < VELOCITY_BUFFER_SIZE) {
        buffer_filled_count_++;
    }

    double sum = 0;
    for (int i = 0; i < VELOCITY_BUFFER_SIZE; i++) {
        sum += velocity_count_buffer_[i];
    }
    
    wheel_speed_ = sum / buffer_filled_count_;
}

double MotorDriver::getSpeed(){

	return wheel_speed_;

}

uint32_t MotorDriver::getCurrentDeltaCount()
{
	return last_delta_count_;
}

int MotorDriver::getIntendedDirection()
{
	return intended_direction_;
}
