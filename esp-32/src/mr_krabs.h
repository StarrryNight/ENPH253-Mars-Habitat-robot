#pragma once
#include <optional>
#include "esp_timer.h"
#include "line_follower.h"
#include "orientation_controller.h"
#include "motor_controller.h"

class MrKrabs
{

public:
	MrKrabs();
	void setup();

	void reset();

	// Processes incoming RPi UART commands. Called from Arduino loop().
	void update();

	// Enters line-following mode. Robot drives forward at FORWARD_SPEED with
	// lateral PID correction from the photoresistor array.
	void startLineFollowing();

	// Enters rotation mode. Turns to target_angle (rad) using encoder dead-reckoning.
	void startRotation(double target_angle = 0.0);

private:
	// Called every CONTROL_LOOP_PERIOD_US by the esp_timer.
	void stepControl();

	bool is_line_following_;

	// Held in optionals so the global MrKrabs object is safe to construct before
	// Arduino init. setup() emplaces them once hardware is ready.
	std::optional<LineFollower> line_follower_;
	std::optional<MotorController> motor_controller_;

	// OrientationController has no hardware deps in its constructor — safe as direct member.
	OrientationController orientation_controller_;

	esp_timer_handle_t control_loop_timer_;

	// esp_timer_get_time() timestamp (µs) when the open-loop forward drive
	// test started; used by stepControl to stop after FORWARD_DRIVE_TEST_DURATION_US.
	uint64_t drive_test_start_us_;
};
