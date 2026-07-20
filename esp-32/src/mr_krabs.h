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
static constexpr double FORWARD_SPEED = 0.5;

// Non-blocking settle delay (µs) held between the three drive actions
// (line-following, rotating, applying an arm pose) so each has time to
// physically settle before the next begins. See MrKrabs::stepControl.
static constexpr uint64_t ACTION_TRANSITION_DELAY_US = 1000000; // 1 s

// Teleoperation (manual numpad/o/p control over USB serial, bypassing the
// RPi/AI). See MrKrabs::handleTeleopChar. Numpad 7/8/9/4/6/1/2/3 drive the 8
// compass directions around 5 (stop); o/p rotate CCW/CW.
static constexpr double TELEOP_LINEAR_SPEED = 0.3;   // m/s
static constexpr double TELEOP_ANGULAR_SPEED = 2.0;  // rad/s
// A key is considered "still held" as long as its character keeps arriving at
// least this often; the sending script must repeat a key while held (raw
// serial has no key-up event).
static constexpr uint64_t TELEOP_KEY_TIMEOUT_US = 300000; // 300 ms

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

	// Dispatches one teleop command character (numpad 1-9 + o/p) read from
	// Serial. Enters teleop mode (taking priority over AI/line-following/
	// rotation in stepControl) and refreshes that key's "held" timer — see
	// computeTeleopVelocity. '5' and ' ' are an explicit stop: they clear
	// every key's held timer immediately without exiting teleop mode.
	void handleTeleopChar(char c);

	// Sums the RobotVelocity contribution of every teleop key whose timer
	// hasn't expired, so keys held together combine into a single
	// diagonal/combined velocity on this holonomic drive.
	RobotVelocity computeTeleopVelocity();

	enum TeleopKey {
		TELEOP_1, TELEOP_2, TELEOP_3, TELEOP_4, TELEOP_6, TELEOP_7, TELEOP_8, TELEOP_9,
		TELEOP_O, TELEOP_P, TELEOP_KEY_COUNT
	};

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

	// Set by handleTeleopChar() on any teleop keypress, but stepControl()'s
	// drive dispatch runs the AI pipeline unconditionally and never reads
	// this — teleop input handling (this flag, computeTeleopVelocity()) is
	// currently dormant, kept for a future teleop/auto switch.
	bool is_teleop_;

	// esp_timer_get_time() timestamp (µs) each key's character was last seen;
	// indexed by TeleopKey. A key counts as held while within TELEOP_KEY_TIMEOUT_US.
	uint64_t teleop_key_last_seen_us_[TELEOP_KEY_COUNT];

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

	// SMS_STS/SCSerial's constructor only sets defaults (pSerial = nullptr) and
	// touches no hardware, so it's safe as a direct member; pSerial is bound to
	// Serial2 in setup() once Arduino init (and Serial2.begin()) has run.
	SMS_STS test_servo_;

};
