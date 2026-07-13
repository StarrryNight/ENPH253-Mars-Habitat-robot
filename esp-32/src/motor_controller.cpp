#include <cmath>
#include "motor_controller.h"
#include "HWCDC.h"
#include "motor_driver.h"
#include "constants.h"
#include "esp_timer.h"

MotorController::MotorController()
	: current_target_velocity_({0, 0, 0}),
	  current_wheel_velocities_({0, 0, 0}),
	  prev_step_time_us_(0),
	  accumulated_angle_(0),
	  wheel_1_pid_(PidController(0, 1, 2, 3)),
	  wheel_2_pid_(PidController(0, 1, 2, 3)),
	  wheel_3_pid_(PidController(0, 1, 2, 3))
{
	// Motors are NOT constructed here — setup() does that after Arduino init.
}


void MotorController::setup()
{
	wheel_1_motor_.emplace(1, WHEEL_1_PWM_CHANNEL_0, WHEEL_1_PWM_CHANNEL_1,
	                       WHEEL_1_PWM_PIN_0, WHEEL_1_PWM_PIN_1,
	                       WHEEL_1_ENCODER_0);
	wheel_2_motor_.emplace(2, WHEEL_2_PWM_CHANNEL_0, WHEEL_2_PWM_CHANNEL_1,
	                       WHEEL_2_PWM_PIN_0, WHEEL_2_PWM_PIN_1,
	                       WHEEL_2_ENCODER_0);
	wheel_3_motor_.emplace(3, WHEEL_3_PWM_CHANNEL_0, WHEEL_3_PWM_CHANNEL_1,
	                       WHEEL_3_PWM_PIN_0, WHEEL_3_PWM_PIN_1,
	                       WHEEL_3_ENCODER_0);
	prev_step_time_us_ = esp_timer_get_time();
}

void MotorController::setVelocity(RobotVelocity target_velocity)
{
	current_target_velocity_ = target_velocity;
	applyVelocity_(target_velocity);
}

void MotorController::addVelocity(RobotVelocity correction_velocity)
{
	// Applies a one-shot correction without modifying the stored base target.
	applyVelocity_(current_target_velocity_ + correction_velocity);
}

void MotorController::applyVelocity_(RobotVelocity target)
{
	if (!wheel_1_motor_ || !wheel_2_motor_ || !wheel_3_motor_) return;

	uint64_t now = esp_timer_get_time();
	double delta_t = static_cast<double>(now - prev_step_time_us_) * 1e-6;
	if (delta_t <= 0.0) delta_t = CONTROL_LOOP_PERIOD;
	prev_step_time_us_ = now;


	// Accumulate heading from kiwi-drive kinematics: omega = (w1+w2+w3)/(3R)
	accumulated_angle_ += (current_wheel_velocities_.wheel_1 +
	                       current_wheel_velocities_.wheel_2 +
	                       current_wheel_velocities_.wheel_3) /
	                      (3.0 * WHEEL_DISTANCE_FROM_CENTER_M) * delta_t;

	WheelVelocities target_wheel = euclideanToWheel(target);

	// PID error is in m/s; multiply output by VELOCITY_TO_PWM to get PWM counts.
	double w1_pwm = wheel_1_motor_->get_current_motor_speed() +
	                wheel_1_pid_.step(target_wheel.wheel_1 - current_wheel_velocities_.wheel_1, delta_t) * VELOCITY_TO_PWM;
	double w2_pwm = wheel_2_motor_->get_current_motor_speed() +
	                wheel_2_pid_.step(target_wheel.wheel_2 - current_wheel_velocities_.wheel_2, delta_t) * VELOCITY_TO_PWM;
	double w3_pwm = wheel_3_motor_->get_current_motor_speed() +
	                wheel_3_pid_.step(target_wheel.wheel_3 - current_wheel_velocities_.wheel_3, delta_t) * VELOCITY_TO_PWM;

	wheel_1_motor_->set_velocity(w1_pwm);
	wheel_2_motor_->set_velocity(w2_pwm);
	wheel_3_motor_->set_velocity(w3_pwm);
}


void MotorController::driveOpenLoop(RobotVelocity v)
{
	if (!wheel_1_motor_ || !wheel_2_motor_ || !wheel_3_motor_) return;

	WheelVelocities target = euclideanToWheel(v);
	wheel_1_motor_->set_velocity(target.wheel_1 * VELOCITY_TO_PWM);
	wheel_2_motor_->set_velocity(target.wheel_2 * VELOCITY_TO_PWM);
	wheel_3_motor_->set_velocity(target.wheel_3 * VELOCITY_TO_PWM);

}

double MotorController::computeAngle()
{
	return accumulated_angle_;
}

/*
 *
	   ▲ +Y (Forward)
				│
				│
  // WHEEL 1 //  │  \\ WHEEL 2 \\
  [Front-Left]   │   [Front-Right]
	 (150°)      │      (30°)
		  \      │        /
		   \     │       /
		    \    │      /
-X ──────────────┼──────────────► +X (Right)
(Left)          /│\             (Right)
               / │ \
              /  │  \
				 │
		   == WHEEL 3 ==
			[Back Wheel]
			  (270°)
				 │
				 ▼ -Y
 *
 */
WheelVelocities MotorController::euclideanToWheel(RobotVelocity v)
{
	// v_i = -sin(θ_i)*vx + cos(θ_i)*vy + R*ω
	// θ measured CCW from +Y: wheel1=150°, wheel2=30°, wheel3=270°
	double wheel_1 = -std::sin(5.0*M_PI/6) * v.x + std::cos(5.0*M_PI/6) * v.y + WHEEL_DISTANCE_FROM_CENTER_M * v.omega;
	double wheel_2 = -std::sin(M_PI/6)     * v.x + std::cos(M_PI/6)     * v.y + WHEEL_DISTANCE_FROM_CENTER_M * v.omega;
	double wheel_3 = -std::sin(3.0*M_PI/2) * v.x + std::cos(3.0*M_PI/2) * v.y + WHEEL_DISTANCE_FROM_CENTER_M * v.omega;

	return WheelVelocities{wheel_1, wheel_2, wheel_3};
}

void MotorController::tickMotorSpeeds(){
	wheel_1_motor_->tickSpeed();
	wheel_2_motor_->tickSpeed();
	wheel_3_motor_->tickSpeed();
	Serial.printf("wheel 1: %.3f\n", wheel_1_motor_->getSpeed());
	Serial.printf("wheel 2: %.3f\n", wheel_2_motor_->getSpeed());
	Serial.printf("wheel 3: %.3f\n", wheel_3_motor_->getSpeed());

}
