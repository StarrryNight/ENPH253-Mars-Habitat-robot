#include "motor_controller.h"

MotorController::MotorController(): current_position_(0), current_velocity_(0), prev_step(0){}
MotorController::setup(){
	motor_pid_ = PidController(3,3,3,999);

}
void MotorController::setVelocity(double target_velocity){
	double error = motor_pid_.step( target_velocity-current_velocity_, current_time - prev_step_);
			
	
}
