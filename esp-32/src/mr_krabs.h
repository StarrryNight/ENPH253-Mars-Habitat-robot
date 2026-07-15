#pragma once
#include <optional>
#include "esp_timer.h"
#include "line_follower.h"
#include "orientation_controller.h"
#include "motor_controller.h"
#include "arm.h"
#include "ai.h"
#include "pins.h"
#include <SCServo.h>

// Forward speed during line following (m/s). Tune empirically.
static constexpr double FORWARD_SPEED = 2.0;

// Open-loop forward drive test duration (µs) — see MrKrabs::stepControl.
static constexpr uint64_t FORWARD_DRIVE_TEST_DURATION_US = 10000000000; // 1 s

// Non-blocking settle delay (µs) held between the three drive actions
// (line-following, rotating, applying an arm pose) so each has time to
// physically settle before the next begins. See MrKrabs::stepControl.
static constexpr uint64_t ACTION_TRANSITION_DELAY_US = 1000000; // 1 s

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
	void motorStepControl();

	// Reconciles AI's desired drive command against drive_mode_, calling
	// startLineFollowing()/startRotation() on change. Returns true iff a
	// transition just happened this tick (caller should hold off driving —
	// the transition already set a fresh settle delay).
	bool handleDriveTransition();

	// Drives the motors for the currently active drive_mode_: line-following
	// correction, or rotating toward AI's target heading and, once reached,
	// telling AI to apply the arm pose.
	void driveCurrentMode();

	// Which of the two drive-mechanism actions the loop is currently executing.
	// AI decides which one is wanted (AI::desiredDriveCommand()); rotating is
	// not tracked separately — it's just the drivetrain's role while
	// APPLYING_ROBOT_POSE, on the way to letting AI apply the arm pose.
	AI::DriveCommand drive_mode_;

	// Last heading (deg) passed to startRotation(), so stepControl() only
	// re-issues it when AI's target actually changes.
	double last_commanded_rotation_degrees_;

	// Non-blocking settle-delay deadline (esp_timer_get_time() timestamp).
	// While esp_timer_get_time() < this, stepControl() holds still instead of
	// starting the next action. 0 means no active wait. See stepControl().
	uint64_t action_settle_until_us_;

	// Held in optionals so the global MrKrabs object is safe to construct before
	// Arduino init. setup() emplaces them once hardware is ready.
	std::optional<LineFollower> line_follower_;
	std::optional<MotorController> motor_controller_;
	std::optional<Arm> arm_;
	std::optional<AI> ai_;

	// OrientationController has no hardware deps in its constructor — safe as direct member.
	OrientationController orientation_controller_;

	esp_timer_handle_t control_loop_timer_;
	esp_timer_handle_t motor_control_loop_timer_;

	// esp_timer_get_time() timestamp (µs) when the open-loop forward drive
	// test started; used by stepControl to stop after FORWARD_DRIVE_TEST_DURATION_US.
	uint64_t drive_test_start_us_;

	// SMS_STS/SCSerial's constructor only sets defaults (pSerial = nullptr) and
	// touches no hardware, so it's safe as a direct member; pSerial is bound to
	// Serial2 in setup() once Arduino init (and Serial2.begin()) has run.
	SMS_STS test_servo_;

};
