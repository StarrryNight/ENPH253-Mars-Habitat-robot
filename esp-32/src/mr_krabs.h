#pragma once
#include <optional>
#include "esp_timer.h"
#include "line_follower.h"
#include "orientation_controller.h"
#include "motor_controller.h"
#include "arm.h"
#include "ai.h"
#include "oled.h"
#include "pins.h"
#include <SCServo.h>
#include "proto_gen/robot_messages.pb.h"
#include "constants.h"

// Fixed rotation speed magnitude (rad/s) while searching for the line in
// AI::DriveMode::SEARCHING_FOR_LINE. The direction applied is the reverse of
// the last commanded rotation (see last_rotation_sign_) — the assumption
// being the line was overshot while rotating that way, so sweeping back is
// more likely to reacquire it than continuing the same direction.
// Deliberately gentler than TELEOP_ANGULAR_SPEED for a controlled sweep.
// Tune empirically.
static constexpr double LINE_SEARCH_OMEGA_RAD_S = 1.3;

// Hardware validation: fixed angular speed (rad/s) driven open-loop (no wheel
// PID, no dead-reckoning target) from the moment setup() finishes. Re-issued
// every stepControl() tick since driveOpenLoop()'s raw PWM write is otherwise
// clobbered by the next motor_control_loop_timer_ tick (see stepControl).
static constexpr double OPEN_LOOP_TEST_OMEGA_RAD_S = 2.0;

// Non-blocking settle delay (µs) held between the three drive actions
// (line-following, rotating, applying an arm pose) so each has time to
// physically settle before the next begins. See MrKrabs::stepControl.
static constexpr uint64_t ACTION_TRANSITION_DELAY_US = 2500000; // 1.5 s

// Teleoperation (manual numpad/o/p control over USB serial, bypassing the
// RPi/AI). See MrKrabs::handleTeleopChar. Numpad 7/8/9/4/6/1/2/3 drive the 8
// compass directions around 5 (stop); o/p rotate CCW/CW.
static constexpr double TELEOP_LINEAR_SPEED = 0.3;   // m/s
static constexpr double TELEOP_ANGULAR_SPEED = 2.0;  // rad/s
// A key is considered "still held" as long as its character keeps arriving at
// least this often; the sending script must repeat a key while held (raw
// serial has no key-up event).
static constexpr uint64_t TELEOP_KEY_TIMEOUT_US = 300000; // 300 ms

// Length-prefixed nanopb framing for RPi -> ESP32 Command messages (see
// MrKrabs::handleProtoByte). Wire format: 4-byte little-endian payload
// length, then that many bytes of a serialized Command. Shares the Serial
// stream with teleop keys (see isTeleopChar) — while a frame is mid-flight,
// every byte is consumed as frame data regardless of its ASCII value.
static constexpr size_t MAX_PROTO_PAYLOAD_SIZE = 64; // Command_size is 2; ample margin
// If no further frame byte arrives within this window, the partial frame is
// abandoned (protects against desyncing forever on stray/teleop bytes).
static constexpr uint64_t PROTO_FRAME_TIMEOUT_US = 200000; // 200 ms

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

	// Enters idle mode. Motors held at zero velocity.
	void startIdle();

	// Enters line-search mode. Rotates at LINE_SEARCH_OMEGA_RAD_S, in the
	// direction opposite last_rotation_sign_, until AI's state transitions
	// away (see RobotState::REACQUIRING_LINE).
	void startSearchingForLine();

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

	// True for the raw bytes handleTeleopChar recognizes (plus CR/LF, which
	// it silently ignores). Used by update() to decide, while idle between
	// proto frames, whether an incoming byte is a teleop key or the first
	// byte of a length-prefixed Command frame.
	static bool isTeleopChar(uint8_t c);

	// Feeds one raw Serial byte through the length-prefix framing state
	// machine (proto_rx_state_). Once a full frame is assembled, decodes it
	// as a Command and hands it to handleCommand().
	void handleProtoByte(uint8_t b);

	// Called once a Command has been fully received and nanopb-decoded.
	// Draws a face on the OLED: happy (Display::showTeletubbyFace) when
	// teletubby_detected=true, angry (Display::showAngryFace) otherwise.
	void handleCommand(const Command &cmd);

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

	// Blinks the onboard RGB LED red while metal_detected is true, off
	// otherwise. now is esp_timer_get_time(), passed in rather than read
	// again so it matches the rest of stepControl()'s timing this tick.
	void updateMetalDetectorLed(bool metal_detected, uint64_t now);

	// Which drive-mechanism action the loop is currently executing. AI decides
	// which one is wanted (AI::desiredDriveMode()); rotating is not tracked
	// separately — it's just the drivetrain's role while APPLYING_SEQUENCE, on
	// the way to letting AI apply the current pose in the sequence.
	AI::DriveMode drive_mode_;

	// Set by handleTeleopChar() on any teleop keypress, but stepControl()'s
	// drive dispatch runs the AI pipeline unconditionally and never reads
	// this — teleop input handling (this flag, computeTeleopVelocity()) is
	// currently dormant, kept for a future teleop/auto switch.
	bool is_teleop_;

	// esp_timer_get_time() timestamp (µs) each key's character was last seen;
	// indexed by TeleopKey. A key counts as held while within TELEOP_KEY_TIMEOUT_US.
	uint64_t teleop_key_last_seen_us_[TELEOP_KEY_COUNT];

	// State for handleProtoByte()'s length-prefixed Command framing.
	enum ProtoRxState { PROTO_RX_IDLE, PROTO_RX_LENGTH, PROTO_RX_PAYLOAD };
	ProtoRxState proto_rx_state_ = PROTO_RX_IDLE;
	uint8_t proto_len_buf_[4];
	size_t proto_len_bytes_read_ = 0;
	uint32_t proto_payload_len_ = 0;
	uint8_t proto_payload_buf_[MAX_PROTO_PAYLOAD_SIZE];
	size_t proto_payload_bytes_read_ = 0;
	uint64_t proto_last_byte_us_ = 0;

	// Last heading (deg) passed to startRotation(), so stepControl() only
	// re-issues it when AI's target actually changes.
	double last_commanded_rotation_degrees_;

	// Last MotorController::getTotalTranslationM() reading, so
	// driveCurrentMode() can add just this tick's incremental distance to
	// AI's progress instead of re-adding the whole odometer total.
	double last_total_translation_m_ = 0.0;

	// Sign (+1/-1) of the last nonzero angle passed to startRotation().
	// Zero-degree targets (an unrotated pose in a sequence) leave this
	// unchanged, so it always reflects the most recent actual turn.
	// startSearchingForLine() reverses this to pick a search direction.
	double last_rotation_sign_;

	// Angular velocity (rad/s, signed) driven during SEARCHING_FOR_LINE,
	// latched by startSearchingForLine() from last_rotation_sign_.
	double search_omega_rad_s_;

	// Non-blocking settle-delay deadline (esp_timer_get_time() timestamp).
	// While esp_timer_get_time() < this, stepControl() holds still instead of
	// starting the next action. 0 means no active wait. See stepControl().
	uint64_t action_settle_until_us_;

	// esp_timer_get_time() timestamp of the last periodic debug state print
	// (see stepControl). Rate-limited so it doesn't flood Serial at the
	// control loop's 100 Hz.
	uint64_t last_state_print_us_;

	// Set once per stepControl() call (rate-limited off last_state_print_us_)
	// and read by driveCurrentMode() so its per-mode debug print fires on the
	// same tick as the general one, without re-checking the clock.
	bool debug_print_now_;

	// State for updateMetalDetectorLed()'s blink: whether the onboard RGB LED
	// is currently lit, and the esp_timer_get_time() timestamp it last
	// toggled at.
	bool metal_led_on_ = false;
	uint64_t metal_led_last_toggle_us_ = 0;

	// Held in optionals so the global MrKrabs object is safe to construct before
	// Arduino init. setup() emplaces them once hardware is ready.
	std::optional<LineFollower> line_follower_;
	std::optional<MotorController> motor_controller_;
	std::optional<Arm> arm_;
	std::optional<AI> ai_;
	std::optional<Display> display_;

	// OrientationController has no hardware deps in its constructor — safe as direct member.
	OrientationController orientation_controller_;

	esp_timer_handle_t control_loop_timer_;
	esp_timer_handle_t motor_control_loop_timer_;

	// SMS_STS/SCSerial's constructor only sets defaults (pSerial = nullptr) and
	// touches no hardware, so it's safe as a direct member; pSerial is bound to
	// Serial2 in setup() once Arduino init (and Serial2.begin()) has run.
	SMS_STS test_servo_;

	// Arm-only bench test: cycles the arm through kArmTestPoses (mr_krabs.cpp)
	// on a fixed timer, completely bypassing AI/rotation/SequenceRunner. Used
	// in place of the normal AI dispatch in stepControl() while the
	// drivetrain isn't wired up / powered — sequence-driven states' arm
	// poses are gated behind real rotation-complete feedback (see
	// OrientationController::reachedTarget), which never fires without wheels.
	size_t arm_test_pose_index_;
	uint64_t arm_test_pose_until_us_;

};
