#include <cmath>
#include "motor_controller.h"
#include "motor_driver.h"
#include "constants.h"

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
}
void MotorController::setVelocity(double target_velocity_x, double target_velocity_y, double target_angular_velocity){

	WheelVelocities target_wheel_velocities = euclideanToWheel(target_velocity_x, target_velocity_y, target_angular_velocity);
	if (target_wheel_velocities.wheel_1 > 0){
		wheel_1_motor_.rotateClockwise(target_wheel_velocities.wheel_1 * 100);
	}
	else{
		wheel_1_motor_.rotateCounterClockwise(-target_wheel_velocities.wheel_1 * 100);
	}

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
WheelVelocities MotorController::euclideanToWheel(double velocity_x, double velocity_y, double angular_velocity){
	double wheel_1 = -std::sin(30)*velocity_x + cos(30)*velocity_y+ WHEEL_DISTANCE_FROM_CENTER_M*angular_velocity; 
	double wheel_2 = std::sin(30)*velocity_x + cos(30)*velocity_y+ WHEEL_DISTANCE_FROM_CENTER_M*angular_velocity; 
	double wheel_3 = std::sin(90)*velocity_x + WHEEL_DISTANCE_FROM_CENTER_M*angular_velocity; 

	return WheelVelocities{wheel_1, wheel_2, wheel_3};
}


WheelVelocities MotorController::getWheelVelocities(){
	return {};
}
