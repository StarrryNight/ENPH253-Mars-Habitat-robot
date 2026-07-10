#include <Arduino.h>
#include "mr_krabs.h"
#include "constants.h"
#include "esp_timer.h"

MrKrabs::MrKrabs() : is_line_following_(false), control_loop_timer_(nullptr), drive_test_start_us_(0) {}

void MrKrabs::setup()
{
	Serial.begin(115200);
	delay(100);
	Serial.println("ESP32 is ready!");

	// Emplace hardware objects now that Arduino init has run.
	line_follower_.emplace();
	motor_controller_.emplace();
	motor_controller_->setup();
	arm_.emplace();

	delay(2000); // wait 2 s before driving so the robot can be placed safely
	drive_test_start_us_ = esp_timer_get_time();

	// Start fixed-rate control loop at CONTROL_LOOP_PERIOD_US (10 ms).
	esp_timer_create_args_t timer_args = {
		.callback = [](void *arg) { static_cast<MrKrabs *>(arg)->stepControl(); },
		.arg = this,
		.name = "control_loop_timer"
	};
	esp_timer_create(&timer_args, &control_loop_timer_);
	esp_timer_start_periodic(control_loop_timer_, CONTROL_LOOP_PERIOD_US);
}

void MrKrabs::reset()
{
}

void MrKrabs::update()
{
	// TODO: read RPi UART (Serial1) and dispatch state commands
	arm_->tickArm();
}

void MrKrabs::stepControl()
{
	// Open-loop forward drive test — no PID, no line following.
	// Stops after FORWARD_DRIVE_TEST_DURATION_US so the robot doesn't run away.
	bool still_driving = (esp_timer_get_time() - drive_test_start_us_) < FORWARD_DRIVE_TEST_DURATION_US;
	motor_controller_->driveOpenLoop({0, still_driving ? FORWARD_SPEED : 0.0, 0});
}

void MrKrabs::startLineFollowing()
{
	is_line_following_ = true;
}

void MrKrabs::startRotation(double target_angle)
{
	is_line_following_ = false;
	orientation_controller_.startRotation(target_angle);
}

MrKrabs mr_krabs_;

void setup()
{
	mr_krabs_.setup();
}
void loop()
{
	mr_krabs_.update();
}
