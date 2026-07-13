#include <cmath>
#include "motor_controller.h"
#include "HWCDC.h"
#include "motor_driver.h"
#include "constants.h"
#include "pid_config.h"
#include "esp_timer.h"

MotorController::MotorController()
	: current_target_velocity_({0, 0, 0}),
	  current_wheel_velocities_({0, 0, 0}),
	  prev_step_time_us_(0),
	  accumulated_angle_(0),
	  accumulated_rotation_distance_m_(0),
	  wheel_left_pid_(PidController(WHEEL_PID_P, WHEEL_PID_I, WHEEL_PID_D, WHEEL_PID_MAX_I)),
	  wheel_right_pid_(PidController(WHEEL_PID_P, WHEEL_PID_I, WHEEL_PID_D, WHEEL_PID_MAX_I)),
	  wheel_back_pid_(PidController(WHEEL_PID_P, WHEEL_PID_I, WHEEL_PID_D, WHEEL_PID_MAX_I))
{
	// Motors are NOT constructed here — setup() does that after Arduino init.
}


void MotorController::setup()
{
	wheel_left_motor_.emplace(1, WHEEL_LEFT_PWM_CHANNEL_0, WHEEL_LEFT_PWM_CHANNEL_1,
	                       WHEEL_LEFT_PWM_PIN_0, WHEEL_LEFT_PWM_PIN_1,
	                       WHEEL_LEFT_ENCODER_0);
	wheel_right_motor_.emplace(2, WHEEL_RIGHT_PWM_CHANNEL_0, WHEEL_RIGHT_PWM_CHANNEL_1,
	                       WHEEL_RIGHT_PWM_PIN_0, WHEEL_RIGHT_PWM_PIN_1,
	                       WHEEL_RIGHT_ENCODER_0);
	wheel_back_motor_.emplace(3, WHEEL_BACK_PWM_CHANNEL_0, WHEEL_BACK_PWM_CHANNEL_1,
	                       WHEEL_BACK_PWM_PIN_0, WHEEL_BACK_PWM_PIN_1,
	                       WHEEL_BACK_ENCODER_0);
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
	if (!wheel_left_motor_ || !wheel_right_motor_ || !wheel_back_motor_) return;

	uint64_t now = esp_timer_get_time();
	double delta_t = static_cast<double>(now - prev_step_time_us_) * 1e-6;
	if (delta_t <= 0.0) delta_t = CONTROL_LOOP_PERIOD;
	prev_step_time_us_ = now;


	// Accumulate heading from kiwi-drive kinematics: omega = (w_left+w_right+w_back)/(3R)
	accumulated_angle_ += (current_wheel_velocities_.wheel_left +
	                       current_wheel_velocities_.wheel_right +
	                       current_wheel_velocities_.wheel_back) /
	                      (3.0 * WHEEL_DISTANCE_FROM_CENTER_M) * delta_t;

	WheelVelocities target_wheel = euclideanToWheel(target);

	// PID error is in m/s; multiply output by VELOCITY_TO_PWM to get PWM counts.
	double wheel_left_pwm = wheel_left_motor_->get_current_motor_speed() +
	                wheel_left_pid_.step(target_wheel.wheel_left - current_wheel_velocities_.wheel_left, delta_t) * VELOCITY_TO_PWM;
	double wheel_right_pwm = wheel_right_motor_->get_current_motor_speed() +
	                wheel_right_pid_.step(target_wheel.wheel_right - current_wheel_velocities_.wheel_right, delta_t) * VELOCITY_TO_PWM;
	double wheel_back_pwm = wheel_back_motor_->get_current_motor_speed() +
	                wheel_back_pid_.step(target_wheel.wheel_back - current_wheel_velocities_.wheel_back, delta_t) * VELOCITY_TO_PWM;

	wheel_left_motor_->set_velocity(wheel_left_pwm);
	wheel_right_motor_->set_velocity(wheel_right_pwm);
	wheel_back_motor_->set_velocity(wheel_back_pwm);
}


void MotorController::driveOpenLoop(RobotVelocity v)
{
	if (!wheel_left_motor_ || !wheel_right_motor_ || !wheel_back_motor_) return;

	WheelVelocities target = euclideanToWheel(v);
	wheel_left_motor_->set_velocity(target.wheel_left * VELOCITY_TO_PWM);
	wheel_right_motor_->set_velocity(target.wheel_right * VELOCITY_TO_PWM);
	wheel_back_motor_->set_velocity(target.wheel_back * VELOCITY_TO_PWM);

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
  // WHEEL LEFT // │  \\ WHEEL RIGHT \\
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
		   == WHEEL BACK ==
			[Back Wheel]
			  (270°)
				 │
				 ▼ -Y
 *
 */
WheelVelocities MotorController::euclideanToWheel(RobotVelocity v)
{
	// v_i = -sin(θ_i)*vx + cos(θ_i)*vy + R*ω
	// θ measured CCW from +Y: wheel_left=150°, wheel_right=30°, wheel_back=270°
	double wheel_left = -std::sin(5.0*M_PI/6) * v.x + std::cos(5.0*M_PI/6) * v.y + WHEEL_DISTANCE_FROM_CENTER_M * v.omega;
	double wheel_right = -std::sin(M_PI/6)     * v.x + std::cos(M_PI/6)     * v.y + WHEEL_DISTANCE_FROM_CENTER_M * v.omega;
	double wheel_back = -std::sin(3.0*M_PI/2) * v.x + std::cos(3.0*M_PI/2) * v.y + WHEEL_DISTANCE_FROM_CENTER_M * v.omega;

	return WheelVelocities{wheel_left, wheel_right, wheel_back};
}


RobotVelocity MotorController::wheelToEuclidean(WheelVelocities v)
{
    double vx = (2.0/3.0) * (
        -std::sin(5.0*M_PI/6) * v.wheel_left +
        -std::sin(M_PI/6)     * v.wheel_right +
        -std::sin(3.0*M_PI/2) * v.wheel_back
    );
    double vy = (2.0/3.0) * (
         std::cos(5.0*M_PI/6) * v.wheel_left +
         std::cos(M_PI/6)     * v.wheel_right +
         std::cos(3.0*M_PI/2) * v.wheel_back
    );
    double omega = (1.0/3.0) * (
        v.wheel_left + v.wheel_right + v.wheel_back
    ) / WHEEL_DISTANCE_FROM_CENTER_M;

    return RobotVelocity{vx, vy, omega};
}

void MotorController::tickMotorSpeeds(){
	wheel_left_motor_->tickSpeed();
	wheel_right_motor_->tickSpeed();
	wheel_back_motor_->tickSpeed();
	RobotVelocity v = wheelToEuclidean({wheel_left_motor_->getSpeed(), wheel_right_motor_->getSpeed(), wheel_back_motor_->getSpeed()});
	Serial.printf("wheel_left_velocity: %.5f\n",wheel_left_motor_->getSpeed());
	Serial.printf("wheel_right_velocity: %.5f\n",wheel_right_motor_->getSpeed());
	Serial.printf("wheel_back_velocity: %.5f\n",wheel_back_motor_->getSpeed());

	Serial.printf("v_x: %.5f\n", v.x);
	Serial.printf("v_y: %.5f\n", v.y);
	Serial.printf("omega: %.5f\n", v.omega);

}

void MotorController::startRotation(){
	accumulated_rotation_distance_m_ = 0.0;
}

double MotorController::calculateRotationDegrees(){
	// Reuses the delta each MotorDriver already computed this tick in tickSpeed()
	// (called via tickMotorSpeeds() earlier in the same control loop step) instead
	// of independently re-diffing getEncoderCount().
	WheelVelocities delta_wheel{
		wheel_left_motor_->getCurrentDeltaCount() * ENCODER_RESOLUTION_DISTANCE_M,
		wheel_right_motor_->getCurrentDeltaCount() * ENCODER_RESOLUTION_DISTANCE_M,
		wheel_back_motor_->getCurrentDeltaCount() * ENCODER_RESOLUTION_DISTANCE_M
	};

	// wheelToEuclidean's omega term is (w_left+w_right+w_back)/(3R) — the same kiwi-drive
	// kinematics used for accumulated_angle_ in applyVelocity_. Feeding it per-tick
	// wheel distance deltas (instead of velocities) yields the per-tick angle delta
	// directly, correctly combining all three wheels instead of taking their max.
	double omega_delta = wheelToEuclidean(delta_wheel).omega;
	accumulated_rotation_distance_m_ += omega_delta * WHEEL_DISTANCE_FROM_CENTER_M;

	return accumulated_rotation_distance_m_/WHEEL_DISTANCE_FROM_CENTER_M;
}


