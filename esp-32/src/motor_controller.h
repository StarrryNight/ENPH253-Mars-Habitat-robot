#include "pid_controller.h"

class MotorController{
	public:
		MotorController();
		void setup();	
		void setVelocity(double target_velocity);

	private:
		PidController motor_pid_;
		double current_position_;
		double current_velocity_;
		double prev_step;
		



};
