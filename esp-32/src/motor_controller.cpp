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
	  current_rotation_rad_(0),
	  wheel_left_pid_(PidController(WHEEL_LEFT_PID_P, WHEEL_LEFT_PID_I, WHEEL_LEFT_PID_D, WHEEL_LEFT_PID_MAX_I)),
	  wheel_right_pid_(PidController(WHEEL_RIGHT_PID_P, WHEEL_RIGHT_PID_I, WHEEL_RIGHT_PID_D, WHEEL_RIGHT_PID_MAX_I)),
	  wheel_back_pid_(PidController(WHEEL_BACK_PID_P, WHEEL_BACK_PID_I, WHEEL_BACK_PID_D, WHEEL_BACK_PID_MAX_I))
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
}


void MotorController::applyVelocity()
{
	if (!wheel_left_motor_ || !wheel_right_motor_ || !wheel_back_motor_) return;

	double delta_t = MOTOR_CONTROL_LOOP_PERIOD;


	// Accumulate heading from kiwi-drive kinematics: omega = (w_left+w_right+w_back)/(3R)
	accumulated_angle_ += (current_wheel_velocities_.wheel_left +
	                       current_wheel_velocities_.wheel_right +
	                       current_wheel_velocities_.wheel_back) /
	                      (3.0 * WHEEL_DISTANCE_FROM_CENTER_M) * delta_t;

	WheelVelocities target_wheel_velocity = euclideanToWheel(current_target_velocity_);

	// Feedforward (VELOCITY_TO_PWM * target) + PID trim on the residual error.
	// Using the current target as the baseline — instead of accumulating the
	// PID output onto the previous PWM — means PWM tracks the target directly:
	// it jumps to roughly the right duty the instant a new target is set
	// (fixing slow-to-start-from-0) and collapses to 0 the instant target
	// goes back to 0 (fixing slow-to-stop), with no separate kickstart term
	// needed. MotorDriver::primeForRestart (called from stopImmediate) still
	// forces that from-stop jump through the H-bridge coast/dead-time guard.
	// PID error is in m/s; multiply output by VELOCITY_TO_PWM to get PWM counts.
int wheel_left_pwm = static_cast<int>(
    target_wheel_velocity.wheel_left * VELOCITY_TO_PWM +
    wheel_left_pid_.step(target_wheel_velocity.wheel_left - current_wheel_velocities_.wheel_left, delta_t)
);

int wheel_right_pwm = static_cast<int>(
    target_wheel_velocity.wheel_right * VELOCITY_TO_PWM +
    wheel_right_pid_.step(target_wheel_velocity.wheel_right - current_wheel_velocities_.wheel_right, delta_t)
);

int wheel_back_pwm = static_cast<int>(
    target_wheel_velocity.wheel_back * WHEEL_BACK_VELOCITY_TO_PWM +
    wheel_back_pid_.step(target_wheel_velocity.wheel_back - current_wheel_velocities_.wheel_back, delta_t)
);

	wheel_left_motor_->set_velocity(wheel_left_pwm);
	wheel_right_motor_->set_velocity(wheel_right_pwm);
	wheel_back_motor_->set_velocity(wheel_back_pwm);
}

void MotorController::driveOpenLoop(RobotVelocity v)
{
	if (!wheel_left_motor_ || !wheel_right_motor_ || !wheel_back_motor_) return;

	WheelVelocities target = euclideanToWheel(v);
	wheel_left_motor_->set_velocity(target.wheel_left  * VELOCITY_TO_PWM);
	wheel_right_motor_->set_velocity(target.wheel_right * VELOCITY_TO_PWM);
	wheel_back_motor_->set_velocity(target.wheel_back  * WHEEL_BACK_VELOCITY_TO_PWM);

}

void MotorController::stopImmediate()
{
	if (!wheel_left_motor_ || !wheel_right_motor_ || !wheel_back_motor_) return;

	current_target_velocity_ = {0, 0, 0};
	wheel_left_pid_.reset();
	wheel_right_pid_.reset();
	wheel_back_pid_.reset();
	driveOpenLoop({0, 0, 0});
	// So the next start — even in the same direction as before this stop —
	// still coasts one control cycle before applying the kickstart PWM in
	// applyVelocity(), instead of jumping straight to full duty. See
	// MotorDriver::primeForRestart.
	wheel_left_motor_->primeForRestart();
	wheel_right_motor_->primeForRestart();
	wheel_back_motor_->primeForRestart();
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

void MotorController::resetSpeedBaselines(){
	if (!wheel_left_motor_ || !wheel_right_motor_ || !wheel_back_motor_) return;
	wheel_left_motor_->resetSpeedBaseline();
	wheel_right_motor_->resetSpeedBaseline();
	wheel_back_motor_->resetSpeedBaseline();
}

void MotorController::tickMotorSpeeds(){
	wheel_left_motor_->tickSpeed();
	wheel_right_motor_->tickSpeed();
	wheel_back_motor_->tickSpeed();

	current_wheel_velocities_ = {wheel_left_motor_->getSpeed(),wheel_right_motor_->getSpeed(),wheel_back_motor_->getSpeed()};
	RobotVelocity v = wheelToEuclidean(current_wheel_velocities_) ;
	current_robot_velocity_ = v;

	//print
	//Serial.printf("wheel_left_velocity: %.5f\n",wheel_left_motor_->getSpeed());
	//Serial.printf("wheel_right_velocity: %.5f\n",wheel_right_motor_->getSpeed());
	//Serial.printf("wheel_back_velocity: %.5f\n",wheel_back_motor_->getSpeed());
///	Serial.printf("v_x: %.5f\n", v.x);
///	Serial.printf("v_y: %.5f\n", v.y);
///	Serial.printf("omega: %.5f\n", v.omega);

}

void MotorController::startRotation(){
	current_rotation_rad_ = 0.0;
	// A new/changed rotation target means whatever the wheel velocity PIDs
	// accumulated chasing the previous target (or previous drive mode) is
	// stale — don't let it carry into this one.
	wheel_left_pid_.reset();
	wheel_right_pid_.reset();
	wheel_back_pid_.reset();
}
void MotorController::updateRotationTracking(){
	// current_wheel_velocities_ is set by tickMotorSpeeds() (must run first
	// this tick — see MrKrabs::motorStepControl) from each MotorDriver's
	// getSpeed(), which is already smoothed by that driver's internal
	// VELOCITY_BUFFER_SIZE-sample moving average. Integrating that filtered
	// per-wheel velocity is much less quantization-noisy tick-to-tick than
	// diffing raw encoder counts directly. wheelToEuclidean's omega term
	// averages all three wheels via the same kiwi-drive kinematics used
	// elsewhere (omega = (w_left+w_right+w_back)/(3R)).
	double omega = wheelToEuclidean(current_wheel_velocities_).omega;
	current_rotation_rad_ += omega * MOTOR_CONTROL_LOOP_PERIOD;
}

double MotorController::getElapsedRotation() const{
	return current_rotation_rad_;
}

RobotVelocity MotorController::getCurrentRobotVelocity(){
	return current_robot_velocity_;
}

