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
			double curr_position = motor_controller_->calculateCurrentOrientation();
			if (orientation_controller_.reachedTarget(curr_position)){
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
