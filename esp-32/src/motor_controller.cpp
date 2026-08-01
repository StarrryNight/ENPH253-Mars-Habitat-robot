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
	  wheel_left_rotation_count_(0),
	  wheel_right_rotation_count_(0),
	  wheel_back_rotation_count_(0),
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
	                       WHEEL_LEFT_ENCODER_0, WHEEL_LEFT_PWM_OFFSET);
	wheel_right_motor_.emplace(2, WHEEL_RIGHT_PWM_CHANNEL_0, WHEEL_RIGHT_PWM_CHANNEL_1,
	                       WHEEL_RIGHT_PWM_PIN_0, WHEEL_RIGHT_PWM_PIN_1,
	                       WHEEL_RIGHT_ENCODER_0, WHEEL_RIGHT_PWM_OFFSET);
	wheel_back_motor_.emplace(3, WHEEL_BACK_PWM_CHANNEL_0, WHEEL_BACK_PWM_CHANNEL_1,
	                       WHEEL_BACK_PWM_PIN_0, WHEEL_BACK_PWM_PIN_1,
	                       WHEEL_BACK_ENCODER_0, WHEEL_BACK_PWM_OFFSET);
	prev_step_time_us_ = esp_timer_get_time();
}

void MotorController::setVelocity(RobotVelocity target_velocity)
{
	current_target_velocity_ = target_velocity;
	open_loop_active_ = false;
}


void MotorController::applyVelocity()
{
	if (!wheel_left_motor_ || !wheel_right_motor_ || !wheel_back_motor_) return;
	// While driveOpenLoop() is active, it's already writing PWM directly each
	// tick — running the PID here too would immediately clobber that with a
	// stale current_target_velocity_ (see open_loop_active_).
	if (open_loop_active_) return;

	double delta_t = MOTOR_CONTROL_LOOP_PERIOD;


	// Accumulate heading from kiwi-drive kinematics: omega = (w_left+w_right+w_back)/(3R)
	accumulated_angle_ += (current_wheel_velocities_.wheel_left +
	                       current_wheel_velocities_.wheel_right +
	                       current_wheel_velocities_.wheel_back) /
	                      (3.0 * WHEEL_DISTANCE_FROM_CENTER_M) * delta_t;

	WheelVelocities target_wheel_velocity = euclideanToWheel(current_target_velocity_);

	// Hold a wheel's PID in reset while it's sitting out MotorDriver's
	// shoot-through guard after a direction reversal. PWM is forced to zero for
	// that window and the wheel is coasting, so the error the PID would be
	// integrating is error it cannot act on — and with the velocity buffer wiped
	// by the reversal, measured speed reads ~0, making that error the full
	// commanded speed. Left to accumulate, it dumps into the first PWM write
	// after the guard clears, kicking the wheel past its target. Per-wheel
	// because a strafe or a yaw trim flips one wheel while the others keep
	// driving; stopImmediate()/startRotation() cover the deliberate-stop cases.
	if (wheel_left_motor_->isDirectionChangePending())  wheel_left_pid_.reset();
	if (wheel_right_motor_->isDirectionChangePending()) wheel_right_pid_.reset();
	if (wheel_back_motor_->isDirectionChangePending())  wheel_back_pid_.reset();

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
    (target_wheel_velocity.wheel_left   +
    wheel_left_pid_.step(target_wheel_velocity.wheel_left - current_wheel_velocities_.wheel_left, delta_t))*VELOCITY_TO_PWM
);

int wheel_right_pwm = static_cast<int>(
    (target_wheel_velocity.wheel_right  +
    wheel_right_pid_.step(target_wheel_velocity.wheel_right - current_wheel_velocities_.wheel_right, delta_t))*VELOCITY_TO_PWM
);

int wheel_back_pwm = static_cast<int>(
    (target_wheel_velocity.wheel_back +
    wheel_back_pid_.step(target_wheel_velocity.wheel_back - current_wheel_velocities_.wheel_back, delta_t))*VELOCITY_TO_PWM
);

	wheel_left_motor_->set_velocity(wheel_left_pwm);
	wheel_right_motor_->set_velocity(wheel_right_pwm);
	wheel_back_motor_->set_velocity(wheel_back_pwm);

	// Rate-limited (every 20 calls =~ 200 ms at MOTOR_CONTROL_LOOP_PERIOD_US)
	// closed-loop diagnostic — the counterpart to the open-loop [FF] print, but
	// running on the real state machine with the PIDs live.
	//
	// Reading it: wheel_target vs actual is the tracking error the PIDs are
	// working on; pwm is what applyVelocity() asked for (feedforward + PID trim,
	// pre-deadzone/offset/clamp); duty is what MotorDriver actually wrote to the
	// pins. pwm nonzero with duty zero means the command was swallowed — the
	// deadzone, a direction flip, or the shoot-through guard — rather than the
	// motor failing to turn.
	static int debug_print_counter = 0;
	if (++debug_print_counter >= 20){
		debug_print_counter = 0;
		Serial.printf("[Motor] target(x=%.2f y=%.2f w=%.2f) wheel_target(L=%.3f R=%.3f B=%.3f) actual(L=%.3f R=%.3f B=%.3f) pwm(L=%d R=%d B=%d) duty(L=%d R=%d B=%d)\n",
			current_target_velocity_.x, current_target_velocity_.y, current_target_velocity_.omega,
			target_wheel_velocity.wheel_left, target_wheel_velocity.wheel_right, target_wheel_velocity.wheel_back,
			current_wheel_velocities_.wheel_left, current_wheel_velocities_.wheel_right, current_wheel_velocities_.wheel_back,
			wheel_left_pwm, wheel_right_pwm, wheel_back_pwm,
			wheel_left_motor_->getLastDutyCycle(), wheel_right_motor_->getLastDutyCycle(),
			wheel_back_motor_->getLastDutyCycle());
	}
}

void MotorController::driveOpenLoop(RobotVelocity v)
{
	if (!wheel_left_motor_ || !wheel_right_motor_ || !wheel_back_motor_) return;

	open_loop_active_ = true;
	WheelVelocities target = euclideanToWheel(v);
	wheel_left_motor_->set_velocity(target.wheel_left  * VELOCITY_TO_PWM);
	wheel_right_motor_->set_velocity(target.wheel_right * VELOCITY_TO_PWM);
	wheel_back_motor_->set_velocity(target.wheel_back  * VELOCITY_TO_PWM);

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
	wheel_left_rotation_count_ = 0;
	wheel_right_rotation_count_ = 0;
	wheel_back_rotation_count_ = 0;
	// A new/changed rotation target means whatever the wheel velocity PIDs
	// accumulated chasing the previous target (or previous drive mode) is
	// stale — don't let it carry into this one.
	wheel_left_pid_.reset();
	wheel_right_pid_.reset();
	wheel_back_pid_.reset();
	// Same argument for the velocity measurement: the samples in the moving
	// average describe the leg that just ended, and legs that call this without
	// stopping first (startStrafingTilHabitat, startHoldingAndMoving,
	// startBackingUp) would otherwise spend up to VELOCITY_BUFFER_SIZE ticks
	// with getSpeed() reporting the previous leg's motion — the PIDs above would
	// correct against a measurement of something that is no longer happening.
	// resetSpeedFilter() rather than primeForRestart(): the wheels may still be
	// turning here, so forgetting last_direction_ would cost a needless tick of
	// dead-time coast on the next command.
	wheel_left_motor_->resetSpeedFilter();
	wheel_right_motor_->resetSpeedFilter();
	wheel_back_motor_->resetSpeedFilter();
}
void MotorController::updateRotationTracking(){
	// Accumulate exact signed encoder pulses since startRotation() instead of
	// integrating (filtered) velocity every tick. getCurrentDeltaCount() is
	// this tick's raw, unsmoothed pulse delta (must run after
	// tickMotorSpeeds() — see MrKrabs::motorStepControl); the encoder itself
	// is single-phase/unsigned, so getIntendedDirection() supplies the sign.
	// Summing exact integers and converting to angle only when read means no
	// per-tick floating-point rounding/smoothing error compounds over a long
	// rotation, unlike repeatedly adding velocity*dt.
	wheel_left_rotation_count_  += static_cast<int64_t>(wheel_left_motor_->getCurrentDeltaCount())  * wheel_left_motor_->getIntendedDirection();
	wheel_right_rotation_count_ += static_cast<int64_t>(wheel_right_motor_->getCurrentDeltaCount()) * wheel_right_motor_->getIntendedDirection();
	wheel_back_rotation_count_  += static_cast<int64_t>(wheel_back_motor_->getCurrentDeltaCount())  * wheel_back_motor_->getIntendedDirection();

	// During a pure in-place rotation (vx=vy=0, the only way this is ever
	// driven — see startRotation/orientation_controller_), every wheel's
	// surface distance should equal R*angle individually, so each wheel
	// gives an independent angle estimate. Take the largest-magnitude one
	// rather than averaging all three: a slipping wheel reads low, and
	// averaging it in would drag the estimate down, making rotation look
	// smaller than it really is and risking overshoot before
	// reachedTarget() catches up.
	double angle_left  = (wheel_left_rotation_count_  * ENCODER_RESOLUTION_DISTANCE_M) / WHEEL_DISTANCE_FROM_CENTER_M;
	double angle_right = (wheel_right_rotation_count_ * ENCODER_RESOLUTION_DISTANCE_M) / WHEEL_DISTANCE_FROM_CENTER_M;
	double angle_back  = (wheel_back_rotation_count_  * ENCODER_RESOLUTION_DISTANCE_M) / WHEEL_DISTANCE_FROM_CENTER_M;

	double angle = angle_left;
	if (std::abs(angle_right) > std::abs(angle)) angle = angle_right;
	if (std::abs(angle_back)  > std::abs(angle)) angle = angle_back;

	current_rotation_rad_ = angle;
}

double MotorController::getYawDriftRad() const
{
	// omega = (v_left + v_right + v_back) / (3R), applied to accumulated counts
	// instead of velocities — see wheelToEuclidean() for the same expression on
	// the velocity side. Summing first is what cancels translation: a pure
	// strafe commands (-0.5, -0.5, +1.0) * vx and a pure drive (-c, +c, 0) * vy,
	// both of which add to zero here, so only genuine yaw is left.
	int64_t count_sum = wheel_left_rotation_count_ + wheel_right_rotation_count_ + wheel_back_rotation_count_;
	return (count_sum * ENCODER_RESOLUTION_DISTANCE_M) / (3.0 * WHEEL_DISTANCE_FROM_CENTER_M);
}

double MotorController::getElapsedRotation() const{
	return current_rotation_rad_;
}

void MotorController::updateTranslationTracking(){
	// Integrates the measured (filtered) robot velocity each tick instead of
	// accumulating exact encoder-count deltas — the exact-count approach
	// proved less reliable in practice, so this trades back some
	// long-run compounding error for steadier per-tick readings. Must run
	// after tickMotorSpeeds() (see its header comment), which is what keeps
	// current_robot_velocity_ fresh for this tick.
	total_translation_m_ += std::hypot(current_robot_velocity_.x, current_robot_velocity_.y) * MOTOR_CONTROL_LOOP_PERIOD;

	// Same measured velocity again, but integrated as a vector rather than a
	// magnitude: rotate this tick's robot-frame (x, y) into the frame the robot
	// started in, so what accumulates is displacement relative to the start
	// instead of distance travelled. A robot that drives out and comes back
	// reads a large total_translation_m_ and a field pose near zero.
	//
	// Heading is advanced first, so the rotation uses this tick's heading rather
	// than the previous one — the two are within omega*dt of each other, well
	// inside the encoder resolution either way, but taking the newer one keeps
	// this consistent with the velocity it's rotating.
	field_heading_rad_ += current_robot_velocity_.omega * MOTOR_CONTROL_LOOP_PERIOD;
	double cos_h = std::cos(field_heading_rad_);
	double sin_h = std::sin(field_heading_rad_);
	field_x_m_ += (current_robot_velocity_.x * cos_h - current_robot_velocity_.y * sin_h) * MOTOR_CONTROL_LOOP_PERIOD;
	field_y_m_ += (current_robot_velocity_.x * sin_h + current_robot_velocity_.y * cos_h) * MOTOR_CONTROL_LOOP_PERIOD;
}

MotorController::FieldPose MotorController::getFieldPose() const{
	return {field_x_m_, field_y_m_, field_heading_rad_};
}

double MotorController::getTotalTranslationM() const{
	return total_translation_m_;
}

void MotorController::resetTranslationTracking(){
	total_translation_m_ = 0.0;
}

RobotVelocity MotorController::getCurrentRobotVelocity(){
	return current_robot_velocity_;
}

