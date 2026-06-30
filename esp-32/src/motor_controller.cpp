#include <cmath>
#include "motor_controller.h"
#include "motor_driver.h"
#include "constants.h"

<<<<<<< HEAD
MotorController::MotorController() : current_position_(0),
									 current_wheel_velocities_({0, 0, 0}),
									 prev_step_time_(std::chrono::steady_clock::now()),
									 wheel_1_pid_(PidController(0, 1, 2, 3)),
									 wheel_2_pid_(PidController(0, 1, 2, 3)),
									 wheel_3_pid_(PidController(0, 1, 2, 3)),
									 wheel_1_motor_(MotorDriver(1, WHEEL_1_PWM_CHANNEL_0, WHEEL_1_PWM_CHANNEL_1, WHEEL_1_PWM_PIN_0, WHEEL_1_PWM_PIN_1, 0, 0)),
									 wheel_2_motor_(MotorDriver(2, WHEEL_2_PWM_CHANNEL_0, WHEEL_2_PWM_CHANNEL_1, WHEEL_2_PWM_PIN_0, WHEEL_2_PWM_PIN_1, 0, 0)),
									 wheel_3_motor_(MotorDriver(3, WHEEL_3_PWM_CHANNEL_0, WHEEL_3_PWM_CHANNEL_1, WHEEL_3_PWM_PIN_0, WHEEL_3_PWM_PIN_1, 0, 0))
{
=======
MotorController::MotorController(): current_position_(0),
	current_wheel_velocities_({0,0,0}),
	wheel_1_pid_(PidController(1,0,0,3)),
	wheel_2_pid_(PidController(1,0,0,3)),
	wheel_3_pid_(PidController(1,0,0,3)),
	wheel_1_motor_(1, WHEEL_1_PWM_CHANNEL_0, WHEEL_1_PWM_CHANNEL_1, WHEEL_1_PWM_PIN_0, WHEEL_1_PWM_PIN_1, WHEEL_1_ENCODER_0, 0),
	wheel_2_motor_(MotorDriver(2,  WHEEL_2_PWM_CHANNEL_0,WHEEL_2_PWM_CHANNEL_1 ,WHEEL_2_PWM_PIN_0, WHEEL_2_PWM_PIN_1,0,0)),
	wheel_3_motor_(MotorDriver(3,  WHEEL_3_PWM_CHANNEL_0,WHEEL_3_PWM_CHANNEL_1 ,WHEEL_3_PWM_PIN_0, WHEEL_3_PWM_PIN_1,0,0))
{}

void MotorController::begin(){
	wheel_1_motor_.begin();
	wheel_2_motor_.begin();
	wheel_3_motor_.begin();
>>>>>>> 028a5a6c2b69952a0f99a2081b56e3d5a0a858a7
}

<<<<<<< HEAD
void MotorController::setup()
{
}
void MotorController::setVelocity(RobotVelocity target_velocity)
{
	RobotVelocity current_target_velocity = target_velocity;
	WheelVelocities target_wheel_velocities = euclideanToWheel(target_velocity);

	// Corrected syntax using duration cast for fractional seconds
	auto now = std::chrono::steady_clock::now();
	auto delta_t = std::chrono::duration<double>(now - prev_step_time_.value()).count();
	current_wheel_velocities_ = WheelVelocities{wheel_1_motor_.tickVelocity(), wheel_2_motor_.tickVelocity(), wheel_3_motor_.tickVelocity()};

	// Use PID to calculate correction needed for each wheel
	double wheel_1_correction = wheel_1_pid_.step(target_wheel_velocities.wheel_1 - current_wheel_velocities_.wheel_1, delta_t);
	double wheel_2_correction = wheel_2_pid_.step(target_wheel_velocities.wheel_2 - current_wheel_velocities_.wheel_2, delta_t);
	double wheel_3_correction = wheel_3_pid_.step(target_wheel_velocities.wheel_3 - current_wheel_velocities_.wheel_3, delta_t);

	// Use correction to set velocity
	wheel_1_motor_.set_velocity(wheel_1_motor_.get_current_motor_speed + wheel_1_correction);
	wheel_2_motor_.set_velocity(wheel_2_motor_.get_current_motor_speed + wheel_2_correction);
	wheel_3_motor_.set_velocity(wheel_3_motor_.get_current_motor_speed + wheel_3_correction);

	prev_step_time_ = std::chrono::steady_clock::now();
=======
	WheelVelocities target_wheel_velocities = euclideanToWheel(target_velocity_x, target_velocity_y, target_angular_velocity);
	if (target_wheel_velocities.wheel_1 > 0){
		wheel_1_motor_.rotateClockwise(target_wheel_velocities.wheel_1 * 100);
	}
	else{
		wheel_1_motor_.rotateCounterClockwise(-target_wheel_velocities.wheel_1 * 100);
	}

>>>>>>> 028a5a6c2b69952a0f99a2081b56e3d5a0a858a7
}

void MotorController::addVelocity(RobotVelocity correction_velocity)
{
	double target_velocity = current_target_velocity + correction_velocity;
	current_target_velocity = target_velocity;
	WheelVelocities target_wheel_velocities = euclideanToWheel(target_velocity);

	// Corrected syntax using duration cast for fractional seconds
	auto now = std::chrono::steady_clock::now();
	auto delta_t = std::chrono::duration<double>(now - prev_step_time_.value()).count();
	current_wheel_velocities_ = WheelVelocities{wheel_1_motor_.tickVelocity(), wheel_2_motor_.tickVelocity(), wheel_3_motor_.tickVelocity()};

	// Use PID to calculate correction needed for each wheel
	double wheel_1_correction = wheel_1_pid_.step(target_wheel_velocities.wheel_1 - current_wheel_velocities_.wheel_1, delta_t);
	double wheel_2_correction = wheel_2_pid_.step(target_wheel_velocities.wheel_2 - current_wheel_velocities_.wheel_2, delta_t);
	double wheel_3_correction = wheel_3_pid_.step(target_wheel_velocities.wheel_3 - current_wheel_velocities_.wheel_3, delta_t);

	// Use correction to set velocity
	wheel_1_motor_.set_velocity(wheel_1_motor_.get_current_motor_speed + wheel_1_correction);
	wheel_2_motor_.set_velocity(wheel_2_motor_.get_current_motor_speed + wheel_2_correction);
	wheel_3_motor_.set_velocity(wheel_3_motor_.get_current_motor_speed + wheel_3_correction);

	prev_step_time_ = std::chrono::steady_clock::now();
}
/*
 *
	   ▲ +Y (Forward)
				 │
				 │
  // WHEEL 1 //  │  \\ WHEEL 2 \\
  [Front-Left]   │   [Front-Right]
	 (150°)      │      (30°)
		\        │        /
		 \       │       /
		  \      │      /
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
WheelVelocities MotorController::euclideanToWheel(RobotVelocity target_velocity)
{
	double wheel_1 = -std::sin(30) * target_velocity.x + cos(30) * target_velocity.y + WHEEL_DISTANCE_FROM_CENTER_M * target_velocity.omega;
	double wheel_2 = std::sin(30) * target_velocity.x + cos(30) * target_velocity.y + WHEEL_DISTANCE_FROM_CENTER_M * target_velocity.omega;
	double wheel_3 = std::sin(90) * target_velocity.x + WHEEL_DISTANCE_FROM_CENTER_M * target_velocity.omega;

	return WheelVelocities{wheel_1, wheel_2, wheel_3};
}

WheelVelocities MotorController::getWheelVelocities()
{
	return {};
}
