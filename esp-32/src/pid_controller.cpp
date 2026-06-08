#include "pid_controller.h"

class PidController::PidController{

	public:

	PidController(double p, double i, double d, double max_i): k_p(p), k_i(i), k_d(d), max_inte



	void step();

	void reset();

	private:

	double k_p;
	double k_d;
	double k_i;
	double current_integral;
	

};
