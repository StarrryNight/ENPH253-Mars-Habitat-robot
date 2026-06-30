#include "pid_controller.h"

PidController::PidController(double p, double i, double d, double max_i)
	: k_p_(p), k_i_(i), k_d_(d), max_integral_(max_i), current_integral_(0), prev_error_(0), first_step_(true) {}

double PidController::step(double error, double delta_t){
	current_integral_ += delta_t * k_i_ * error;
	if (current_integral_ > max_integral_) current_integral_ = max_integral_;
	if (current_integral_ < -max_integral_) current_integral_ = -max_integral_;

	double derivative = 0.0;
	if (!first_step_ && delta_t > 0)
		derivative = (error - prev_error_) / delta_t;
	first_step_ = false;
	prev_error_ = error;

	return k_p_ * error + k_d_ * derivative + current_integral_;
}

void PidController::reset(){
	current_integral_ = 0;
	prev_error_ = 0;
	first_step_ = true;
}
