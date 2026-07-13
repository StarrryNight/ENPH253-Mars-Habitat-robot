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
	  accumulated_rotation_distance_m_(0),
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


RobotVelocity MotorController::wheelToEuclidean(WheelVelocities v)
{
    double vx = (2.0/3.0) * (
        -std::sin(5.0*M_PI/6) * v.wheel_1 +
        -std::sin(M_PI/6)     * v.wheel_2 +
        -std::sin(3.0*M_PI/2) * v.wheel_3
    );
    double vy = (2.0/3.0) * (
         std::cos(5.0*M_PI/6) * v.wheel_1 +
         std::cos(M_PI/6)     * v.wheel_2 +
         std::cos(3.0*M_PI/2) * v.wheel_3
    );
    double omega = (1.0/3.0) * (
        v.wheel_1 + v.wheel_2 + v.wheel_3
    ) / WHEEL_DISTANCE_FROM_CENTER_M;

    return RobotVelocity{vx, vy, omega};
}

void MotorController::tickMotorSpeeds(){
	wheel_1_motor_->tickSpeed();
	wheel_2_motor_->tickSpeed();
	wheel_3_motor_->tickSpeed();
	RobotVelocity v = wheelToEuclidean({wheel_1_motor_->getSpeed(), wheel_2_motor_->getSpeed(), wheel_3_motor_->getSpeed()});
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
		wheel_1_motor_->getCurrentDeltaCount() * ENCODER_RESOLUTION_DISTANCE_M,
		wheel_2_motor_->getCurrentDeltaCount() * ENCODER_RESOLUTION_DISTANCE_M,
		wheel_3_motor_->getCurrentDeltaCount() * ENCODER_RESOLUTION_DISTANCE_M
	};

	// wheelToEuclidean's omega term is (w1+w2+w3)/(3R) — the same kiwi-drive
	// kinematics used for accumulated_angle_ in applyVelocity_. Feeding it per-tick
	// wheel distance deltas (instead of velocities) yields the per-tick angle delta
	// directly, correctly combining all three wheels instead of taking their max.
	double omega_delta = wheelToEuclidean(delta_wheel).omega;
	accumulated_rotation_distance_m_ += omega_delta * WHEEL_DISTANCE_FROM_CENTER_M;

	return accumulated_rotation_distance_m_/WHEEL_DISTANCE_FROM_CENTER_M;
}


