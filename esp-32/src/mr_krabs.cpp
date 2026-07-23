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
// horizontal — see STSServo's SERVO_1_*/SERVO_2_* calibration in servo.h.
constexpr ArmPose kArmTestPoses[] = {
	{0, 50, 0, 10},
	{30, 50, 0, 10},
	{70, 50, 0, 10},
	{70, 90, 0, 10},
	{45, 30, 0, 10},
};
constexpr size_t kArmTestPoseCount = sizeof(kArmTestPoses) / sizeof(kArmTestPoses[0]);
constexpr uint64_t ARM_TEST_POSE_PERIOD_US = 2000000; // 2 s per pose
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

	// ====================Set up firmware---------------------- //
	arm_->begin();
	ai_->setArm(&*arm_);
	ai_->setLineFollower(&*line_follower_);

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

	if (now < action_settle_until_us_){
		motor_controller_->setVelocity({0, 0, 0});
		if (debug_print_now_){
			Serial.printf("[MrKrabs] settling, %.0f ms left\n", (action_settle_until_us_ - now) / 1000.0);
		}
		return;
	}
	// Throttled to STATE_PRINT_PERIOD_US (500ms), not run every 10ms tick:
	// each ReadPos/Ping is a full blocking Serial2 round-trip, and four of
	// them every tick was saturating the half-duplex bus and the USB serial
	// output badly enough to garble the printed text itself.
	if (debug_print_now_){
		int16_t pos_1 = test_servo_.ReadPos(/*ID=*/1);
		int16_t pos_2 = test_servo_.ReadPos(/*ID=*/2);
		// Ping is a much simpler transaction than ReadPos (no data payload) —
		// returns the responding ID on success, -1 on no/garbled reply.
		int ping_1 = test_servo_.Ping(1);
		int ping_2 = test_servo_.Ping(2);
		Serial.print("Servo 1 Position: ");
		Serial.print(pos_1);
		Serial.print(" (ping=");
		Serial.print(ping_1);
		Serial.print(")  Servo 2 Position: ");
		Serial.print(pos_2);
		Serial.print(" (ping=");
		Serial.print(ping_2);
		Serial.println(")");
	}

	// Arm-only bench test (AI/rotation dispatch disabled — see
	// arm_test_pose_index_ in mr_krabs.h for why). Cycles through
	// kArmTestPoses on a timer, independent of the drivetrain.
	motor_controller_->setVelocity({0, 0, 0});
	if (now >= arm_test_pose_until_us_){
		const ArmPose &pose = kArmTestPoses[arm_test_pose_index_];
		arm_->setPose(pose);
		Serial.printf("[ArmTest] pose %u/%u: base=%.0f elbow=%.0f\n, claw = %.0f\n",
			(unsigned)arm_test_pose_index_ + 1, (unsigned)kArmTestPoseCount,
			pose.base_pitch_servo_degrees,pose.elbow_pitch_servo_degrees ,pose.claw_servo_degrees);
		arm_test_pose_index_ = (arm_test_pose_index_ + 1) % kArmTestPoseCount;
		arm_test_pose_until_us_ = now + ARM_TEST_POSE_PERIOD_US;
	}

	// ai_->tickAI();
	//
	// if (debug_print_now_){
	// 	Serial.printf("[MrKrabs] state=%s drive_mode=%d\n",
	// 		robotStateName(ai_->currentState()), static_cast<int>(drive_mode_));
	// }
	//
	// if (handleDriveTransition()){
	// 	return;
	// }
	//
	// driveCurrentMode();
}

bool MrKrabs::handleDriveTransition()
{
	AI::DriveMode desired = ai_->desiredDriveMode();
	if (desired != drive_mode_){
		Serial.printf("[MrKrabs] drive_mode %d -> %d (state=%s, target_deg=%.1f)\n",
			static_cast<int>(drive_mode_), static_cast<int>(desired),
			robotStateName(ai_->currentState()), ai_->targetRotationDegrees());
		switch (desired){
			case AI::DriveMode::LINE_FOLLOWING: startLineFollowing(); break;
			case AI::DriveMode::APPLYING_SEQUENCE: startRotation(ai_->targetRotationDegrees() * DEG_TO_RAD); break;
			case AI::DriveMode::SEARCHING_FOR_LINE: startSearchingForLine(); break;
			case AI::DriveMode::IDLE: startIdle(); break;
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
			// setVelocity(), not driveOpenLoop(): motor_control_loop_timer_
			// calls MotorController::applyVelocity() independently every 10ms
			// regardless of drive mode, and it drives off current_target_velocity_
			// (set only by setVelocity()). driveOpenLoop() writes PWM directly
			// without touching that target, so its command was immediately
			// clobbered back to whatever setVelocity() last set (0,0,0 by
			// default) on the very next applyVelocity() tick — the wheels never
			// got a chance to actually turn.
			Serial.printf("correction: %f.2\n", correction);
			motor_controller_->setVelocity({0, speed, -correction});
			// Stub odometry: integrates commanded speed rather than measured
			// encoder distance. Replace with real odometry once available.
			ai_->addProgress(std::abs(speed) * CONTROL_LOOP_PERIOD);
			break;
		}
		case AI::DriveMode::APPLYING_SEQUENCE: {
			double curr_position = motor_controller_->getElapsedRotation();
			if (orientation_controller_.reachedTarget(curr_position)){
				//Serial.print("reached target");
				motor_controller_->setVelocity({0,0,0});
				orientation_controller_.reset();
				if (ai_->onRotationReached()){
					action_settle_until_us_ = esp_timer_get_time() + ACTION_TRANSITION_DELAY_US;
				}
			}
			else{
				double correction = orientation_controller_.calculateCorrection(curr_position);
				motor_controller_->setVelocity({0,0, correction});
			}
			break;
		}
		case AI::DriveMode::SEARCHING_FOR_LINE:
			// AI::tickReacquiringLine() (called via ai_->tickAI() earlier this
			// tick) already checked the line sensors and will transition the
			// state once found — this just keeps spinning until that happens.
			// Direction (search_omega_rad_s_) was latched by
			// startSearchingForLine() as the reverse of the last rotation.
			motor_controller_->setVelocity({0, 0, search_omega_rad_s_});
			break;
		case AI::DriveMode::IDLE:
			motor_controller_->setVelocity({0,0,0});
			break;
	}
}

void MrKrabs::motorStepControl(){
	motor_controller_->tickMotorSpeeds();
	motor_controller_->updateRotationTracking();
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
	motor_controller_->driveOpenLoop({0,0,0});
	drive_mode_ = AI::DriveMode::APPLYING_SEQUENCE;
	action_settle_until_us_ = esp_timer_get_time() + ACTION_TRANSITION_DELAY_US;
	motor_controller_->startRotation();
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
	motor_controller_->driveOpenLoop({0,0,0});
	drive_mode_ = AI::DriveMode::IDLE;
	action_settle_until_us_ = esp_timer_get_time() + ACTION_TRANSITION_DELAY_US;
}

void MrKrabs::startSearchingForLine()
{
	motor_controller_->driveOpenLoop({0,0,0});
	drive_mode_ = AI::DriveMode::SEARCHING_FOR_LINE;
	action_settle_until_us_ = esp_timer_get_time() + ACTION_TRANSITION_DELAY_US;
	// Not using orientation_controller_ here (no fixed target — this spins
	// until AI transitions the state), but still worth clearing the wheel
	// PIDs' stale integral from whatever drive mode preceded this.
	motor_controller_->startRotation();
	// Sweep back the way we came: opposite sign of the last real rotation.
	search_omega_rad_s_ = -last_rotation_sign_ * LINE_SEARCH_OMEGA_RAD_S;
	Serial.printf("[MrKrabs] startSearchingForLine, last_rotation_sign=%.0f omega=%.2f\n",
		last_rotation_sign_, search_omega_rad_s_);
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
