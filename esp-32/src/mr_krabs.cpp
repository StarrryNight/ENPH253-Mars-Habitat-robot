#include <Arduino.h>
#include <cmath>
#include "mr_krabs.h"
#include "constants.h"
#include "esp_timer.h"
#include "motor_controller.h"
#include <pb_decode.h>

namespace {
// Arm-only bench test (see arm_test_pose_index_ in mr_krabs.h). base_pitch
// clamped to [0,70] deg from vertical, elbow_pitch to [23,70] deg down from
// horizontal — see STSServo's SERVO_1_*/SERVO_2_* calibration in servo.h
// yaw 120 center, 40 rock
	//release on rock {0, 23, 35, 0},
	//
	//
	//RANGE FOR COORDIANTE S YSTEM
	//0.25 minimum
constexpr ArmCoordinate kArmTestPoses[] = {
	//0 for cloe
	//50 for oepn

	//{0.3, -0.03, 120, 50, 500, 500},
	//{0.30, 0.03, 120, 50, 500, 500},
	//{0.30, -0.06, 120, 50, 250, 500},
	//{0.30, -0.06, 120, 0, 500, 500},
	//{0.24, 0.060, 120, 0, 500, 500},
	//{0.24, 0.060, Arm::WRIST_PLACE, 0, 500, 500},
	{0.30, 0.03, Arm::WRIST_PLACE, Arm::CLAW_CLOSE, 500, 500},
	{0.30, 0.03, Arm::WRIST_PLACE, Arm::CLAW_OPEN,  500, 500},
	//{0.31, -0.05, 100, 100, 250, 500},
	//{0.31, -0.05, 100, 100, 250, 500},
//	{0.36, -0.05, 120, 50, 250, 500},
//	{0.36, -0.05, 120, 0, 250, 500},
//	{0.24, 0.06, 120, 0, 250, 500},
//	{0.36, -0.05, 120, 0, 250, 500},
//	{0.31, -0.05, 120, 50, 250, 500},
//	{0.28, -0.05, 120, 50, 250, 500},
};
constexpr size_t kArmTestPoseCount = sizeof(kArmTestPoses) / sizeof(kArmTestPoses[0]);
constexpr uint64_t ARM_TEST_POSE_PERIOD_US = 5000000; // 2 s per pose

// Flip to true to re-enable the arm-only bench test loop below. Left in
// place (not deleted) for future bench testing, but off now that the real
// AI-driven sequence (ROTATING_TIL_ROCK -> METAL_DETECTING -> PICKUP_ROCK,
// see ai.cpp) is what should be moving the arm.
constexpr bool kRunArmBenchTest = false;
}

MrKrabs::MrKrabs() :
	drive_mode_(AI::DriveMode::LINE_FOLLOWING),
	last_commanded_rotation_degrees_(0),
	last_rotation_sign_(1.0),
	search_omega_rad_s_(-LINE_SEARCH_OMEGA_RAD_S),
	action_settle_until_us_(0),
	last_state_print_us_(0),
	debug_print_now_(false),
	is_teleop_(false),
	teleop_key_last_seen_us_{0},
	control_loop_timer_(nullptr),
	arm_test_pose_index_(0),
	arm_test_pose_until_us_(0)
{}

void MrKrabs::setup()
{
	Serial.begin(115200);
	delay(100);
	Serial.println("ESP32 is ready!");


	// Emplace objects
	line_follower_.emplace();
	motor_controller_.emplace();
	motor_controller_->setup();
	arm_.emplace();
	ai_.emplace();
	display_.emplace();
	sonar_.emplace();

	// ====================Set up firmware---------------------- //
	arm_->begin();
	ai_->setArm(&*arm_);
	ai_->setLineFollower(&*line_follower_);
	ai_->setSonar(&*sonar_);
	display_->begin();
	sonar_->begin();

	// arm_.emplace() (above) already ran Arm's constructor, which begins
	// Serial2 for the SCServo bus — safe to bind test_servo_ to it now.
	test_servo_.pSerial = &Serial2;

	esp_timer_create_args_t timer_args = {
		.callback = [](void *arg) { static_cast<MrKrabs *>(arg)->stepControl(); },
		.arg = this,
		.name = "control_loop_timer"
	};
	esp_timer_create(&timer_args, &control_loop_timer_);

	esp_timer_create_args_t motor_timer_args = {
		.callback = [](void *arg) { static_cast<MrKrabs *>(arg)->motorStepControl(); },
		.arg = this,
		.name = "motor_control_loop_timer"
	};
	esp_timer_create(&motor_timer_args, &motor_control_loop_timer_);

	esp_timer_start_periodic(control_loop_timer_, CONTROL_LOOP_PERIOD_US);
	motor_controller_->resetSpeedBaselines();
	esp_timer_start_periodic(motor_control_loop_timer_, MOTOR_CONTROL_LOOP_PERIOD_US);
	// Start delay
	action_settle_until_us_ = esp_timer_get_time() + ACTION_TRANSITION_DELAY_US;
	

}

void MrKrabs::reset()
{
}

void MrKrabs::update()
{
	while (Serial.available()) {
		uint8_t b = static_cast<uint8_t>(Serial.read());
		uint64_t now = esp_timer_get_time();

		if (proto_rx_state_ != PROTO_RX_IDLE && (now - proto_last_byte_us_) > PROTO_FRAME_TIMEOUT_US) {
			Serial.println("[MrKrabs] proto frame timed out mid-flight, discarding");
			proto_rx_state_ = PROTO_RX_IDLE;
		}
		proto_last_byte_us_ = now;

		if (proto_rx_state_ == PROTO_RX_IDLE && isTeleopChar(b)) {
			handleTeleopChar(static_cast<char>(b));
			continue;
		}

		handleProtoByte(b);
	}
}

bool MrKrabs::isTeleopChar(uint8_t c)
{
	switch (c) {
		case '1': case '2': case '3': case '4': case '5':
		case '6': case '7': case '8': case '9':
		case 'o': case 'p': case ' ': case '\r': case '\n':
			return true;
		default:
			return false;
	}
}

void MrKrabs::handleProtoByte(uint8_t b)
{
	switch (proto_rx_state_) {
		case PROTO_RX_IDLE:
			proto_len_buf_[0] = b;
			proto_len_bytes_read_ = 1;
			proto_rx_state_ = PROTO_RX_LENGTH;
			break;

		case PROTO_RX_LENGTH:
			proto_len_buf_[proto_len_bytes_read_++] = b;
			if (proto_len_bytes_read_ < 4) {
				break;
			}
			proto_payload_len_ = static_cast<uint32_t>(proto_len_buf_[0]) |
				(static_cast<uint32_t>(proto_len_buf_[1]) << 8) |
				(static_cast<uint32_t>(proto_len_buf_[2]) << 16) |
				(static_cast<uint32_t>(proto_len_buf_[3]) << 24);
			if (proto_payload_len_ > sizeof(proto_payload_buf_)) {
				Serial.printf("[MrKrabs] proto frame too large (%u bytes), dropping\n", proto_payload_len_);
				proto_rx_state_ = PROTO_RX_IDLE;
				break;
			}
			proto_payload_bytes_read_ = 0;
			if (proto_payload_len_ == 0) {
				Command cmd = Command_init_zero;
				handleCommand(cmd);
				proto_rx_state_ = PROTO_RX_IDLE;
			} else {
				proto_rx_state_ = PROTO_RX_PAYLOAD;
			}
			break;

		case PROTO_RX_PAYLOAD:
			proto_payload_buf_[proto_payload_bytes_read_++] = b;
			if (proto_payload_bytes_read_ < proto_payload_len_) {
				break;
			}
			{
				Command cmd = Command_init_zero;
				pb_istream_t stream = pb_istream_from_buffer(proto_payload_buf_, proto_payload_len_);
				if (pb_decode(&stream, Command_fields, &cmd)) {
					handleCommand(cmd);
				} else {
					Serial.printf("[MrKrabs] failed to decode Command: %s\n", PB_GET_ERROR(&stream));
				}
			}
			proto_rx_state_ = PROTO_RX_IDLE;
			break;
	}
}

void MrKrabs::handleCommand(const Command &cmd)
{
	if (cmd.teletubby_detected) {
		Serial.println("[MrKrabs] Teletubby detected!");
		display_->showTeletubbyFace();
		ai_->notifyTeletubbyDetected();
	} else {
		display_->showAngryFace();
	}
}

void MrKrabs::handleTeleopChar(char c)
{
	TeleopKey key;
	switch (c) {
		case '1': key = TELEOP_1; break;
		case '2': key = TELEOP_2; break;
		case '3': key = TELEOP_3; break;
		case '4': key = TELEOP_4; break;
		case '6': key = TELEOP_6; break;
		case '7': key = TELEOP_7; break;
		case '8': key = TELEOP_8; break;
		case '9': key = TELEOP_9; break;
		case 'o': key = TELEOP_O; break; // counter-clockwise
		case 'p': key = TELEOP_P; break; // clockwise
		case '5':
		case ' ':
			// Explicit stop: clear every key's held timer immediately.
			for (uint64_t &t : teleop_key_last_seen_us_) t = 0;
			is_teleop_ = true;
			return;
		default: return; // ignore unrecognized chars (e.g. \r, \n) — no effect on held keys
	}
	teleop_key_last_seen_us_[key] = esp_timer_get_time();
	is_teleop_ = true;
}

RobotVelocity MrKrabs::computeTeleopVelocity()
{
	// 1/sqrt(2): scales each axis of a diagonal numpad key so the combined
	// vector still has magnitude TELEOP_LINEAR_SPEED, matching the cardinal keys.
	static constexpr double DIAG = 0.70710678;

	
	uint64_t now = esp_timer_get_time();
	auto held = [&](TeleopKey k) { return (now - teleop_key_last_seen_us_[k]) <= TELEOP_KEY_TIMEOUT_US; };

	RobotVelocity v{0, 0, 0};
	if (held(TELEOP_8)) v.y += TELEOP_LINEAR_SPEED;
	if (held(TELEOP_2)) v.y -= TELEOP_LINEAR_SPEED;
	if (held(TELEOP_6)) v.x += TELEOP_LINEAR_SPEED;
	if (held(TELEOP_4)) v.x -= TELEOP_LINEAR_SPEED;
	if (held(TELEOP_9)) { v.x += TELEOP_LINEAR_SPEED * DIAG; v.y += TELEOP_LINEAR_SPEED * DIAG; }
	if (held(TELEOP_7)) { v.x -= TELEOP_LINEAR_SPEED * DIAG; v.y += TELEOP_LINEAR_SPEED * DIAG; }
	if (held(TELEOP_3)) { v.x += TELEOP_LINEAR_SPEED * DIAG; v.y -= TELEOP_LINEAR_SPEED * DIAG; }
	if (held(TELEOP_1)) { v.x -= TELEOP_LINEAR_SPEED * DIAG; v.y -= TELEOP_LINEAR_SPEED * DIAG; }
	if (held(TELEOP_O)) v.omega += TELEOP_ANGULAR_SPEED;
	if (held(TELEOP_P)) v.omega -= TELEOP_ANGULAR_SPEED;
	//Serial.printf("COMMANDED_vel x = %f.5\n",v.x);
	//Serial.printf("COMMANDED_vel y = %f.5\n",v.y);
	//Serial.printf("COMMANDED_vel omega = %f.5\n",v.omega);
	return v;
}

// How often the periodic debug state print (see stepControl/driveCurrentMode)
// fires. Deliberately much slower than the 100 Hz control loop.
static constexpr uint64_t STATE_PRINT_PERIOD_US = 500000; // 500 ms

void MrKrabs::stepControl()
{
	uint64_t now = esp_timer_get_time();
	debug_print_now_ = (now - last_state_print_us_) >= STATE_PRINT_PERIOD_US;
	if (debug_print_now_){
		last_state_print_us_ = now;
	}

	// No sonar ping here: this call discarded its result (it existed to feed a
	// debug print), while costing a full blocking ping every tick — and in
	// HABITAT_FIND / ROTATING_TIL_ROCK a second one on top of AI's own. The
	// states that need a reading take it themselves.
	//
	// Before the settle-delay return below and before ai_->tickAI() consumes
	// the sensors: the array has to be sampled every tick no matter what the
	// drivetrain is doing, or LineFollower's prev_state_ (which side the line
	// was last on) goes stale across rotations, strafes and arm sequences —
	// precisely the manoeuvres that move the line across it. See
	// LineFollower::tick.
	line_follower_->tick();
	updateMetalDetectorLed(ai_->metalDetected(), now);

	if (now < action_settle_until_us_){
		motor_controller_->stopImmediate();
		if (debug_print_now_){
			Serial.printf("[MrKrabs] settling, %.0f ms left\n", (action_settle_until_us_ - now) / 1000.0);
		}
		return;
	}
	// Arm-only bench test (AI/rotation dispatch disabled — see
	// arm_test_pose_index_ in mr_krabs.h for why). Cycles through
	// kArmTestPoses on a timer, independent of the drivetrain.
	// Currently disabled (see kRunArmBenchTest) so it doesn't fight with the
	// real AI-driven sequence.
	if (kRunArmBenchTest){
		motor_controller_->stopImmediate();
		if (now >= arm_test_pose_until_us_){
			ArmCoordinate pose = kArmTestPoses[arm_test_pose_index_];
			// Bench test override: drive x_pos off the live sonar reading
			// (cm -> m) instead of the pose table's fixed value, only for
			// the reach/grab poses (index 2 and 3) — the rest of the
			// sequence moves to fixed positions regardless of where the
			// rock was. +170.54mm corrects for the sonar's mounting offset
			// from the arm's coordinate origin (back shoulder joint).
			// Query fresh on pose 2 (rock still clearly in view); pose 3
			// (closing the claw around it) reuses that same reading instead
			// of re-querying, since the claw/rock can occlude or shift the
			// sonar's return by then.
			static double cached_sonar_x = 0.0;
			//if ((arm_test_pose_index_+1)%kArmTestPoseCount == 1){
			//	cached_sonar_x = sonar_->queryDistance() / 100.0 + 0.18054;
			//	pose.x_pos = cached_sonar_x+0.08;
			//} else if ( (arm_test_pose_index_+1)%kArmTestPoseCount == 2){
			//	pose.x_pos = cached_sonar_x-0.02;
			//}	else if ((arm_test_pose_index_+1)%kArmTestPoseCount == 3 || (arm_test_pose_index_+1)%kArmTestPoseCount == 4){
			//	pose.x_pos = cached_sonar_x;
			//}
			arm_->setPoseXY(pose);
			Serial.printf("[ArmTest] pose %u/%u: x=%.2f y=%.2f claw=%.0f base_speed=%.0f elbow_speed=%.0f\n",
				(unsigned)arm_test_pose_index_ + 1, (unsigned)kArmTestPoseCount,
				pose.x_pos, pose.y_pos, pose.claw_servo_degrees, pose.base_pitch_servo_speed, pose.elbow_pitch_servo_speed);
			arm_test_pose_index_ = (arm_test_pose_index_ + 1) % kArmTestPoseCount;
			arm_test_pose_until_us_ = now + ARM_TEST_POSE_PERIOD_US;
		}
	}

	 ai_->tickAI();
	
	 if (debug_print_now_){
	 	Serial.printf("[MrKrabs] state=%s drive_mode=%d\n",
	 		robotStateName(ai_->currentState()), static_cast<int>(drive_mode_));
	 }
	
	 if (handleDriveTransition()){
	 	return;
	 }
	
	 driveCurrentMode();
}

bool MrKrabs::handleDriveTransition()
{
	AI::DriveMode desired = ai_->desiredDriveMode();
	if (desired != drive_mode_){
		Serial.printf("[MrKrabs] drive_mode %d -> %d (state=%s, target_deg=%.1f)\n",
			static_cast<int>(drive_mode_), static_cast<int>(desired),
			robotStateName(ai_->currentState()), ai_->targetRotationDegrees());
		// Detected before the switch below overwrites drive_mode_ — extends
		// the settle delay startLineFollowing() sets, so the rock/arm has
		// extra time to stop swinging after the strafe-back-onto-the-line
		// before LINE_FOLLOWING_REVERSE starts driving forward.
		bool leaving_hold_and_move = (drive_mode_ == AI::DriveMode::HOLDING_AND_MOVING &&
			desired == AI::DriveMode::LINE_FOLLOWING);
		switch (desired){
			case AI::DriveMode::LINE_FOLLOWING: startLineFollowing(); break;
			case AI::DriveMode::APPLYING_SEQUENCE: startRotation(ai_->targetRotationDegrees() * DEG_TO_RAD); break;
			case AI::DriveMode::SEARCHING_FOR_LINE: startSearchingForLine(); break;
			case AI::DriveMode::ROCK_SEARCHING_FOR_LINE: startRockSearchingForLine(); break;
			case AI::DriveMode::IDLE: startIdle(); break;
			case AI::DriveMode::ROTATING_TIL_ROCK: startRotatingTilRock(); break;
			case AI::DriveMode::SQUARING_UP: startSquaringUp(); break;
			case AI::DriveMode::STRAFING_TIL_HABITAT: startStrafingTilHabitat(); break;
			case AI::DriveMode::BACKING_UP: startBackingUp(); break;
			case AI::DriveMode::HOLDING_AND_MOVING: startHoldingAndMoving(); break;
			case AI::DriveMode::REVERSE_180: startReverse180(); break;
		}
		if (leaving_hold_and_move){
			action_settle_until_us_ = esp_timer_get_time() + HABITAT_HOLD_LINE_SETTLE_US;
		}
		last_commanded_rotation_degrees_ = ai_->targetRotationDegrees();
		return true;
	}

	if (desired == AI::DriveMode::APPLYING_SEQUENCE){
		double target = ai_->targetRotationDegrees();
		if (target != last_commanded_rotation_degrees_){
			startRotation(target * DEG_TO_RAD);
			last_commanded_rotation_degrees_ = target;
			return true;
		}
	}
	return false;
}

void MrKrabs::driveCurrentMode()
{
	switch (drive_mode_){
		case AI::DriveMode::LINE_FOLLOWING: {
			double correction = line_follower_->calculateCorrection();
			double speed = FORWARD_SPEED * ai_->lineFollowingDirection();
			motor_controller_->setVelocity({0, speed, -correction});
			// Real odometry: diff of the encoder-count-based translation
			// odometer (MotorController::updateTranslationTracking(), folded
			// in each tick by motorStepControl) rather than integrating
			// (filtered) velocity * dt — avoids compounding smoothing/
			// rounding error over a long run, same reasoning as
			// updateRotationTracking().
			double total_translation_m = motor_controller_->getTotalTranslationM();
			ai_->addProgress(total_translation_m - last_total_translation_m_);
			last_total_translation_m_ = total_translation_m;
			break;
		}
		case AI::DriveMode::APPLYING_SEQUENCE: {
			double curr_position = motor_controller_->getElapsedRotation();
			if (orientation_controller_.reachedTarget(curr_position)){
				//Serial.print("reached target");
				motor_controller_->stopImmediate();
				orientation_controller_.reset();
				if (ai_->onRotationReached()){
					action_settle_until_us_ = esp_timer_get_time() + ai_->sequencePoseSettleUs();
				}
			}
			else{
				double correction = orientation_controller_.calculateCorrection(curr_position);
				motor_controller_->setVelocity({0,0, correction});
			}
			break;
		}
		// Identical drive behavior — the two modes differ only in the
		// direction/initial angle latched by their start*() functions.
		case AI::DriveMode::REVERSE_180:
		// ROCK_SEARCHING_FOR_LINE rides the same case: startRockSearchingForLine()
		// pre-sets line_search_initial_turn_done_, so the fixed-turn branch below is
		// skipped and it drops straight into the reactive spin.
		case AI::DriveMode::ROCK_SEARCHING_FOR_LINE:
		case AI::DriveMode::SEARCHING_FOR_LINE: {
			if (!line_search_initial_turn_done_){
				// Fixed initial turn first, same direction as the reactive spin
				// below — without this, a state that left the robot already
				// facing/sitting on the line (e.g. HABITAT_PICKUP/PLACE,
				// which no longer rotate) could have bothMidSensorsOnLine()
				// read true immediately, ending REACQUIRING_LINE with zero
				// actual rotation.
				double curr_position = motor_controller_->getElapsedRotation();
				if (orientation_controller_.reachedTarget(curr_position)){
					motor_controller_->stopImmediate();
					orientation_controller_.reset();
					line_search_initial_turn_done_ = true;
					ai_->setLineSearchInitialTurnDone(true);
				} else {
					double correction = orientation_controller_.calculateCorrection(curr_position);
					motor_controller_->setVelocity({0,0, correction});
					break;
				}
			}
			// AI::tickReacquiringLine() (called via ai_->tickAI() earlier this
			// tick) already checked the line sensors and will transition the
			// state once found — this just keeps spinning until that happens.
			// Direction (search_omega_rad_s_) was latched by
			// startSearchingForLine() as the reverse of the last rotation.
			motor_controller_->setVelocity({0, 0, search_omega_rad_s_});
			break;
		}
		case AI::DriveMode::IDLE:
			motor_controller_->stopImmediate();
			break;
		case AI::DriveMode::ROTATING_TIL_ROCK:
			// AI::tickRotatingTilRock() (called via ai_->tickAI() earlier this
			// tick) already checked the sonar and will transition the state
			// once a rock is found — this just keeps spinning until then.
			// Direction (rock_search_omega_rad_s_) was latched by
			// startRotatingTilRock() from AI::rockSearchOmegaRadS().
			motor_controller_->setVelocity({0, 0, rock_search_omega_rad_s_});
			break;
		case AI::DriveMode::SQUARING_UP:
			// AI::tickHabitatSquareUp() (called via ai_->tickAI() earlier this
			// tick) watches for the second side sensor and transitions away the
			// moment both read the strip — this just keeps rotating until then.
			// Pure rotation, no translation: the robot is already at the strip.
			motor_controller_->setVelocity({0, 0, SQUARE_UP_OMEGA_RAD_S * ai_->squareUpOmegaSign()});
			break;
		case AI::DriveMode::STRAFING_TIL_HABITAT: {
			// AI::tickHabitatFind() (called via ai_->tickAI() earlier this
			// tick) already checked the sonar and will transition the state
			// once the habitat is found — this just keeps strafing until then.
			// Direction comes straight from AI::habitatFindDirection(), which
			// tickHabitatFind() flips once the second habitat slot is found.
			// The omega term holds the heading the strafe started at, off the
			// encoder-derived yaw estimate — see startStrafingTilHabitat() and
			// OrientationController::holdCorrection.
			double yaw = orientation_controller_.holdCorrection(motor_controller_->getYawDriftRad());
			motor_controller_->setVelocity({HABITAT_STRAFE_SPEED * ai_->habitatFindDirection(), 0, yaw});
			break;
		}
		case AI::DriveMode::BACKING_UP: {
			// AI::tickHabitatBackup() (called via ai_->tickAI() earlier this
			// tick) tracks the distance leg itself off addProgress() below and
			// will transition the state once it's done.
			//
			// Heading held the same way as the strafe legs — a backward leg has
			// no line sensors ahead of it and no sonar bearing to correct
			// against, so without this any yaw it picks up is simply kept, and
			// every later fixed distance/angle inherits it. getYawDriftRad()
			// works here for the same reason it does while strafing: a pure
			// backward drive commands (-0.866, +0.866, 0) * vy, which sums to
			// zero, so what the estimate reports is yaw alone.
			double yaw = orientation_controller_.holdCorrection(motor_controller_->getYawDriftRad());
			motor_controller_->setVelocity({0, -HABITAT_BACKUP_SPEED, yaw});
			double total_translation_m = motor_controller_->getTotalTranslationM();
			ai_->addProgress(total_translation_m - last_total_translation_m_);
			last_total_translation_m_ = total_translation_m;
			break;
		}
		case AI::DriveMode::HOLDING_AND_MOVING: {
			// AI::tickHabitatHoldAndMove() (called via ai_->tickAI() earlier
			// this tick) already checked the line sensors and will transition
			// the state once both mids are on the line — this just keeps
			// strafing until then. No rotation happens beforehand anymore
			// (HABITAT_POST_PICKUP_BACKUP's straight backward leg runs
			// first instead), so this strafes opposite the direction
			// HABITAT_FIND strafed to reach this slot.
			//
			// Same heading hold as STRAFING_TIL_HABITAT above — more valuable
			// here if anything, since the load in the claw shifts the centre of
			// mass and makes the chassis yaw more readily under a sideways
			// command.
			double yaw = orientation_controller_.holdCorrection(motor_controller_->getYawDriftRad());
			motor_controller_->setVelocity({-HABITAT_STRAFE_SPEED * ai_->habitatFindDirection(), 0, yaw});
			break;
		}
	}
}

// Blinks the DevKitM-1's onboard addressable RGB LED (GPIO48) red while
// metal is detected, off otherwise. RGB_BUILTIN/neopixelWrite come from the
// arduino-esp32 core itself (no NeoPixel library needed) — plain
// digitalWrite() can't drive a WS2812, hence the dedicated call.
namespace {
	constexpr uint64_t METAL_LED_BLINK_PERIOD_US = 200000; // 200 ms
}

void MrKrabs::updateMetalDetectorLed(bool metal_detected, uint64_t now)
{
	if (!metal_detected){
		if (metal_led_on_){
			neopixelWrite(RGB_BUILTIN, 0, 0, 0);
			metal_led_on_ = false;
		}
		return;
	}

	if (now - metal_led_last_toggle_us_ >= METAL_LED_BLINK_PERIOD_US){
		metal_led_on_ = !metal_led_on_;
		neopixelWrite(RGB_BUILTIN, metal_led_on_ ? 255 : 0, 0, 0);
		metal_led_last_toggle_us_ = now;
	}
}

void MrKrabs::motorStepControl(){
	motor_controller_->tickMotorSpeeds();
	motor_controller_->updateRotationTracking();
	// Runs unconditionally every tick, like updateRotationTracking() —
	// BACKING_UP's distance-based completion (see AI::tickHabitatBackup)
	// needs the odometer moving too, not just LINE_FOLLOWING. Whichever
	// drive mode actually consumes progress resets the odometer on entry
	// (startRotation(), startBackingUp()), so accumulation during modes that
	// don't care doesn't leak into a later distance check.
	motor_controller_->updateTranslationTracking();
	motor_controller_->applyVelocity();
}

void MrKrabs::startLineFollowing()
{
	drive_mode_ = AI::DriveMode::LINE_FOLLOWING;
	action_settle_until_us_ = esp_timer_get_time() + ACTION_TRANSITION_DELAY_US;
}

void MrKrabs::startRotation(double target_angle)
{
	Serial.printf("[MrKrabs] startRotation(%.3f rad = %.1f deg)\n", target_angle, target_angle * RAD_TO_DEG);
	motor_controller_->stopImmediate();
	drive_mode_ = AI::DriveMode::APPLYING_SEQUENCE;
	action_settle_until_us_ = esp_timer_get_time() + ACTION_TRANSITION_DELAY_US;
	motor_controller_->startRotation();
	// Leaving LINE_FOLLOWING for the rotation/arm-pose part of a checkpoint —
	// zero the translation odometer so it starts clean at 0 once we're back
	// to line-following, instead of carrying forward whatever it read here.
	motor_controller_->resetTranslationTracking();
	last_total_translation_m_ = 0.0;
	orientation_controller_.startRotation(target_angle);
	// A zero-degree pose ("stay facing this way") isn't an actual turn —
	// leave last_rotation_sign_ reflecting whichever way we last really
	// rotated, for startSearchingForLine() to reverse.
	if (target_angle != 0.0){
		last_rotation_sign_ = (target_angle > 0.0) ? 1.0 : -1.0;
	}
}

void MrKrabs::startIdle()
{
	motor_controller_->stopImmediate();
	drive_mode_ = AI::DriveMode::IDLE;
	action_settle_until_us_ = esp_timer_get_time() + ACTION_TRANSITION_DELAY_US;
}

// Fixed initial turn (deg) driveCurrentMode() rotates through, in
// search_omega_rad_s_'s direction, before falling back to the reactive
// continuous spin — see startSearchingForLine/driveCurrentMode.
static constexpr double LINE_SEARCH_INITIAL_TURN_DEG = 45.0;

void MrKrabs::startSearchingForLine()
{
	motor_controller_->stopImmediate();
	drive_mode_ = AI::DriveMode::SEARCHING_FOR_LINE;
	action_settle_until_us_ = esp_timer_get_time() + ACTION_TRANSITION_DELAY_US;
	motor_controller_->startRotation();
	// Sweep back the way we came: opposite sign of the last real rotation —
	// unless AI asks to carry on in the same direction instead (the hand-off out
	// of HABITAT_PLACE; see AI::lineSearchContinuesLastRotation).
	double search_sign = ai_->lineSearchContinuesLastRotation() ? last_rotation_sign_ : -last_rotation_sign_;
	search_omega_rad_s_ = search_sign * LINE_SEARCH_OMEGA_RAD_S;
	// Fixed 45° turn first (same direction as search_omega_rad_s_), then
	// driveCurrentMode() falls back to the reactive continuous spin — see
	// its SEARCHING_FOR_LINE case.
	line_search_initial_turn_done_ = false;
	ai_->setLineSearchInitialTurnDone(false);
	double initial_turn_rad = (search_omega_rad_s_ >= 0.0 ? 1.0 : -1.0) * (LINE_SEARCH_INITIAL_TURN_DEG * DEG_TO_RAD);
	orientation_controller_.startRotation(initial_turn_rad);
	Serial.printf("[MrKrabs] startSearchingForLine, last_rotation_sign=%.0f omega=%.2f\n",
		last_rotation_sign_, search_omega_rad_s_);
}

// Fixed initial turn (deg) for REVERSE_180's first phase. Smaller than
// LINE_SEARCH_INITIAL_TURN_DEG — it only has to carry the mid sensors clear
// of the line, since the reactive spin that follows covers the remaining
// ~150° until the line is reacquired on the far side.
static constexpr double REVERSE_180_INITIAL_TURN_DEG = 120.0;

void MrKrabs::startReverse180()
{
	motor_controller_->stopImmediate();
	drive_mode_ = AI::DriveMode::REVERSE_180;
	action_settle_until_us_ = esp_timer_get_time() + ACTION_TRANSITION_DELAY_US;
	motor_controller_->startRotation();
	// Forced clockwise (negative omega — omega is CCW-positive, see
	// MotorController's RobotVelocity) rather than reversing whichever way we
	// last turned, so the reversal direction is deterministic.
	search_omega_rad_s_ = -LINE_SEARCH_OMEGA_RAD_S;
	line_search_initial_turn_done_ = false;
	ai_->setLineSearchInitialTurnDone(false);
	orientation_controller_.startRotation(-REVERSE_180_INITIAL_TURN_DEG * DEG_TO_RAD);
	last_rotation_sign_ = -1.0;
	Serial.printf("[MrKrabs] startReverse180, omega=%.2f\n", search_omega_rad_s_);
}

void MrKrabs::startRockSearchingForLine()
{
	motor_controller_->stopImmediate();
	drive_mode_ = AI::DriveMode::ROCK_SEARCHING_FOR_LINE;
	action_settle_until_us_ = esp_timer_get_time() + ACTION_TRANSITION_DELAY_US;
	motor_controller_->startRotation();
	// Retrace the rock search: opposite the spin that turned us off the line.
	search_omega_rad_s_ = -last_rotation_sign_ * LINE_SEARCH_OMEGA_RAD_S;
	// Declare the fixed initial turn already done, on both sides. That is what
	// makes this differ from startSearchingForLine(): driveCurrentMode()'s shared
	// case skips straight to the reactive spin, and AI::tickRockReacquireLine
	// watches the sensors from the first tick instead of waiting out a blind 45
	// degrees the robot doesn't need here.
	line_search_initial_turn_done_ = true;
	ai_->setLineSearchInitialTurnDone(true);
	Serial.printf("[MrKrabs] startRockSearchingForLine, last_rotation_sign=%.0f omega=%.2f\n",
		last_rotation_sign_, search_omega_rad_s_);
}

void MrKrabs::startRotatingTilRock()
{
	motor_controller_->stopImmediate();
	drive_mode_ = AI::DriveMode::ROTATING_TIL_ROCK;
	action_settle_until_us_ = esp_timer_get_time() + ACTION_TRANSITION_DELAY_US;
	// Not using orientation_controller_ here (no fixed target — this spins
	// until AI transitions the state), but still worth clearing the wheel
	// PIDs' stale integral from whatever drive mode preceded this.
	motor_controller_->startRotation();
	rock_search_omega_rad_s_ = ai_->rockSearchOmegaRadS();
	// startRotation(target_angle) is the only other place last_rotation_sign_
	// gets set, and ROTATING_TIL_ROCK doesn't go through it (this is a
	// continuous open-loop spin, not a rotate-to-heading) — without this,
	// startSearchingForLine()'s "-last_rotation_sign_" reverse-spin would be
	// reversing whatever stale direction was left over from the last fixed-
	// heading sequence rotation instead of the direction actually used to
	// find this rock, sending REACQUIRING_LINE the wrong way.
	last_rotation_sign_ = (rock_search_omega_rad_s_ > 0.0) ? 1.0 : -1.0;
	Serial.printf("[MrKrabs] startRotatingTilRock, omega=%.2f\n", rock_search_omega_rad_s_);
}

void MrKrabs::startSquaringUp()
{
	// Stop before turning: the entry condition is "one side sensor reached the
	// strip", so any residual forward motion carries the array past the marker
	// being aligned to while the rotation is still working.
	motor_controller_->stopImmediate();
	drive_mode_ = AI::DriveMode::SQUARING_UP;
	action_settle_until_us_ = esp_timer_get_time() + SQUARE_UP_SETTLE_US;
	// Clears the wheel PIDs' integral from the line-following leg, which was
	// chasing a forward velocity this rotation doesn't want.
	motor_controller_->startRotation();
	last_rotation_sign_ = ai_->squareUpOmegaSign();
	Serial.printf("[MrKrabs] startSquaringUp, omega_sign=%.0f\n", ai_->squareUpOmegaSign());
}

void MrKrabs::startStrafingTilHabitat()
{
	drive_mode_ = AI::DriveMode::STRAFING_TIL_HABITAT;
	action_settle_until_us_ = esp_timer_get_time() + ACTION_TRANSITION_DELAY_US;
	// Zero the yaw reference the strafe will hold: startRotation() clears the
	// signed encoder counts getYawDriftRad() reads, and the 0.0 target makes
	// holdCorrection() null out any drift away from *this* heading. Whatever the
	// heading was in absolute terms is irrelevant — the hold only keeps the
	// robot from turning during the strafe, it doesn't know or fix which way it
	// was pointing to begin with.
	motor_controller_->startRotation();
	orientation_controller_.startRotation(0.0);
	Serial.printf("[MrKrabs] startStrafingTilHabitat, direction=%d\n", ai_->habitatFindDirection());
}

void MrKrabs::startHoldingAndMoving()
{
	drive_mode_ = AI::DriveMode::HOLDING_AND_MOVING;
	action_settle_until_us_ = esp_timer_get_time() + ACTION_TRANSITION_DELAY_US;
	// Same yaw reference reset as startStrafingTilHabitat().
	motor_controller_->startRotation();
	orientation_controller_.startRotation(0.0);
	Serial.printf("[MrKrabs] startHoldingAndMoving, direction=%d\n", -ai_->habitatFindDirection());
}

void MrKrabs::startBackingUp()
{
	drive_mode_ = AI::DriveMode::BACKING_UP;
	action_settle_until_us_ = esp_timer_get_time() + ACTION_TRANSITION_DELAY_US;
	// Zero the translation odometer so the distance-based completion in
	// AI::tickHabitatBackup isn't inflated by whatever drive mode (e.g.
	// STRAFING_TIL_HABITAT) preceded this — same reasoning as startRotation().
	motor_controller_->resetTranslationTracking();
	last_total_translation_m_ = 0.0;
	// Zero the yaw reference this leg will hold, same as the strafe legs. Note
	// startRotation() also clears the rotation counts getYawDriftRad() reads,
	// which is what makes the hold relative to the heading we start backing up
	// at rather than to something inherited from the previous leg.
	motor_controller_->startRotation();
	orientation_controller_.startRotation(0.0);
	Serial.printf("[MrKrabs] startBackingUp\n");
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
