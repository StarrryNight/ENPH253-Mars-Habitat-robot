#include <cmath>
#include <optional>
#include "motor_controller.h"

MotorController::MotorController(): current_position_(0), current_wheel_velocities_({0,0,0}), prev_step_time_{},{}

void MotorController::setup(){
	prev_step_time_ = std::chrono::steady_clock::now();
}
void MotorController::setVelocity(double target_velocity_x, double target_velocity_y, double target_angular_velocity){

	WheelVelocities target_wheel_velocities = euclideanToWheel(target_velocity_x, target_velocity_y, target_angular_velocity);

	auto delta_t = (std::chrono::steady_clock::now()-prev_step_time_).count();
	wheel_1_pid_.step(target_wheel_velocities.wheel_1 -current_wheel_velocities_.wheel_1, delta_t);
	wheel_2_pid_.step(target_wheel_velocities.wheel_2 -current_wheel_velocities_.wheel_2, delta_t);
	wheel_3_pid_.step(target_wheel_velocities.wheel_3 -current_wheel_velocities_.wheel_3, delta_t);
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
WheelVelocities MotorController::euclideanToWheel(double velocity_x, double velocity_y, double angular_velocity){
	double wheel_1 = -std::sin(30)*velocity_x + cos(30)*velocity_y+ WHEEL_DISTANCE_FROM_CENTER_M*angular_velocity; 
	double wheel_2 = std::sin(30)*velocity_x + cos(30)*velocity_y+ WHEEL_DISTANCE_FROM_CENTER_M*angular_velocity; 
	double wheel_3 = std::sin(90)*velocity_x + WHEEL_DISTANCE_FROM_CENTER_M*angular_velocity; 

	return WheelVelocities{wheel_1, wheel_2, wheel_3};
}


WheelVelocities MotorController::getWheelVelocities(){
	return {};
}
