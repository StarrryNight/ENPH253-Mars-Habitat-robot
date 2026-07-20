#include <Arduino.h>
#include <cmath>
#include "mr_krabs.h"
#include "constants.h"
#include "esp_timer.h"
#include "motor_controller.h"

MrKrabs::MrKrabs() :
	drive_mode_(AI::DriveMode::LINE_FOLLOWING),
	last_commanded_rotation_degrees_(0),
	action_settle_until_us_(0),
	is_teleop_(false),
	teleop_key_last_seen_us_{0},
	control_loop_timer_(nullptr)
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
		handleTeleopChar(static_cast<char>(Serial.read()));
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

void MrKrabs::stepControl()
{
	if (esp_timer_get_time() < action_settle_until_us_){
		motor_controller_->setVelocity({0, 0, 0});
		return;
	}

	ai_->tickAI();

	if (handleDriveTransition()){
		return;
	}

	driveCurrentMode();
}

bool MrKrabs::handleDriveTransition()
{
	AI::DriveMode desired = ai_->desiredDriveMode();
	if (desired != drive_mode_){
		switch (desired){
			case AI::DriveMode::LINE_FOLLOWING: startLineFollowing(); break;
			case AI::DriveMode::APPLYING_SEQUENCE: startRotation(ai_->targetRotationDegrees() * DEG_TO_RAD); break;
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
			motor_controller_->driveOpenLoop({correction, speed, 0});
			// Stub odometry: integrates commanded speed rather than measured
			// encoder distance. Replace with real odometry once available.
			ai_->addProgress(std::abs(speed) * CONTROL_LOOP_PERIOD);
			break;
		}
		case AI::DriveMode::APPLYING_SEQUENCE: {
			double curr_position = motor_controller_->getElapsedRotation();
			if (orientation_controller_.reachedTarget(curr_position)){
				Serial.print("reached target");
				motor_controller_->setVelocity({0,0,0});
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
	motor_controller_->driveOpenLoop({0,0,0});
	drive_mode_ = AI::DriveMode::APPLYING_SEQUENCE;
	action_settle_until_us_ = esp_timer_get_time() + ACTION_TRANSITION_DELAY_US;
	motor_controller_->startRotation();
	orientation_controller_.startRotation(target_angle);
}

void MrKrabs::startIdle()
{
	motor_controller_->driveOpenLoop({0,0,0});
	drive_mode_ = AI::DriveMode::IDLE;
	action_settle_until_us_ = esp_timer_get_time() + ACTION_TRANSITION_DELAY_US;
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
