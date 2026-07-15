#include <Arduino.h>
#include "mr_krabs.h"
#include "constants.h"
#include "esp_timer.h"
#include "motor_controller.h"

MrKrabs::MrKrabs() : is_line_following_(false), control_loop_timer_(nullptr), drive_test_start_us_(0), is_rotating_(false) {}

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

	// SCServo bus servo: single data wire on GPIO 6, so RX and TX share the
	// same pin (half-duplex). ST-series default bus baud rate is 1,000,000.
	Serial2.begin(1000000, SERIAL_8N1, /*rxPin=*/BASE_ELBOW_PITCH_SERIAL_PIN, /*txPin=*/BASE_ELBOW_PITCH_SERIAL_PIN);
	arm_->begin();
	test_servo_.pSerial = &Serial2;
	




	//test_servo_.WritePosEx(/*ID=*/1, /*Position=*/1000, /*Speed=*/2400, /*ACC=*/50); // ~150 deg
	//delay(2000); // wait 2 s before driving so the robot can be placed safely
	//test_servo_.WritePosEx(/*ID=*/2, /*Position=*/4000, /*Speed=*/34900, /*ACC=*/50); // ~150 deg
	//drive_test_start_us_ = esp_timer_get_time();

	// Start fixed-rate control loop at CONTROL_LOOP_PERIOD_US.
	esp_timer_create_args_t timer_args = {
		.callback = [](void *arg) { static_cast<MrKrabs *>(arg)->stepControl(); },
		.arg = this,
		.name = "control_loop_timer"
	};
	esp_timer_create(&timer_args, &control_loop_timer_);
	esp_timer_start_periodic(control_loop_timer_, CONTROL_LOOP_PERIOD_US);

	esp_timer_create_args_t motor_timer_args = {
		.callback = [](void *arg) { static_cast<MrKrabs *>(arg)->motorStepControl(); },
		.arg = this,
		.name = "motor_control_loop_timer"
	};
	esp_timer_create(&motor_timer_args, &motor_control_loop_timer_);
	// Resync encoder baselines right before the timer starts so pulses picked up
	// during construction/the placement delay above don't read as a velocity spike
	// on the very first tick.
	motor_controller_->resetSpeedBaselines();
	esp_timer_start_periodic(motor_control_loop_timer_, MOTOR_CONTROL_LOOP_PERIOD_US);
	delay(1000);
	arm_->setPose({0,0,0,0});
	delay(1000);
	arm_->setPose({90,90,90,90});
}

void MrKrabs::reset()
{
}

void MrKrabs::update()
{
}
void MrKrabs::stepControl()
{
	bool still_driving = (((esp_timer_get_time() - drive_test_start_us_) < FORWARD_DRIVE_TEST_DURATION_US) && ((esp_timer_get_time() - drive_test_start_us_) > 5000000));
	motor_controller_->setVelocity({0, 0, still_driving ? FORWARD_SPEED : 0.0});
	if (is_line_following_){
		double correction = line_follower_->calculateCorrection();
		motor_controller_->driveOpenLoop({correction, FORWARD_SPEED, 0});	
	} 
	else if (is_rotating_){
		double curr_position = motor_controller_->calculateCurrentOrientation();
		if (orientation_controller_.reachedTarget(curr_position)){
			is_rotating_ = false;
			motor_controller_->setVelocity({0,0,0});
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
	is_line_following_ = true;
}

void MrKrabs::startRotation(double target_angle)
{
	motor_controller_->driveOpenLoop({0,0,0});
	is_line_following_ = false;
	is_rotating_ = true;
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
