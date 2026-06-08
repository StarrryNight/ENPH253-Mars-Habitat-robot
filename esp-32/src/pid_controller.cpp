#include "pid_controller.h"

PidController::PidController(double p, double i, double d, double max_i): k_p_(p), k_d_(d), k_i_(i), max_integral_(max_i) {}

double PidController::step(double error, double delta_t){

	current_integral_ = delta_t * k_i_ * error;
	if (current_integral_ > max_integral_){
		current_integral_ = max_integral_;
    }

	return error * k_p_ + error/k_d_ + current_integral_;
}

void PidController::reset(){
	current_integral_ = 0;
}

	

