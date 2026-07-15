#include <Arduino.h>
#include "mr_krabs.h"
#include "constants.h"
#include "esp_timer.h"
#include "motor_controller.h"

MrKrabs::MrKrabs() :
	drive_mode_(AI::DriveCommand::LINE_FOLLOWING),
	last_commanded_rotation_degrees_(0),
	action_settle_until_us_(0),
	control_loop_timer_(nullptr),
	drive_test_start_us_(0)
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
	Serial2.begin(1000000, SERIAL_8N1, /*rxPin=*/BASE_ELBOW_PITCH_SERIAL_PIN, /*txPin=*/BASE_ELBOW_PITCH_SERIAL_PIN);
	arm_->begin();
	ai_->setArm(&*arm_);
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

}

void MrKrabs::reset()
{
}

void MrKrabs::update()
{
}
void MrKrabs::stepControl()
{
	//Testing code	
	bool still_driving = (((esp_timer_get_time() - drive_test_start_us_) < FORWARD_DRIVE_TEST_DURATION_US) && ((esp_timer_get_time() - drive_test_start_us_) > 5000000));
	motor_controller_->setVelocity({0, 0, still_driving ? FORWARD_SPEED : 0.0});


	// tick AI
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
	AI::DriveCommand desired = ai_->desiredDriveCommand();
	if (desired == AI::DriveCommand::LINE_FOLLOWING){
		if (drive_mode_ != AI::DriveCommand::LINE_FOLLOWING){
			startLineFollowing();
			return true;
		}
	}
	else{
		double target = ai_->targetRotationDegrees();
		if (drive_mode_ != AI::DriveCommand::APPLYING_ROBOT_POSE || target != last_commanded_rotation_degrees_){
			startRotation(target * DEG_TO_RAD);
			last_commanded_rotation_degrees_ = target;
			return true;
		}
	}
	return false;
}

void MrKrabs::driveCurrentMode()
{
	if (drive_mode_ == AI::DriveCommand::LINE_FOLLOWING){
		double correction = line_follower_->calculateCorrection();
		motor_controller_->driveOpenLoop({correction, FORWARD_SPEED, 0});
	}
	else if (drive_mode_ == AI::DriveCommand::APPLYING_ROBOT_POSE){
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
	}
}

void MrKrabs::motorStepControl(){
	motor_controller_->tickMotorSpeeds();
	motor_controller_->applyVelocity();
	
}

void MrKrabs::startLineFollowing()
{
	drive_mode_ = AI::DriveCommand::LINE_FOLLOWING;
	action_settle_until_us_ = esp_timer_get_time() + ACTION_TRANSITION_DELAY_US;
}

void MrKrabs::startRotation(double target_angle)
{
	motor_controller_->driveOpenLoop({0,0,0});
	drive_mode_ = AI::DriveCommand::APPLYING_ROBOT_POSE;
	action_settle_until_us_ = esp_timer_get_time() + ACTION_TRANSITION_DELAY_US;
	motor_controller_->startRotation();
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
