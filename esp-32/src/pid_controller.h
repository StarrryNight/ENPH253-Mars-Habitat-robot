#pragma once

class PidController{

	public:

	PidController(double p, double i, double d, double max_i);

	double step(double error, double delta_t);

	void reset();

	private:

	double k_p_;
	double k_d_;
	double k_i_;
	double max_integral_;
	double current_integral_;
	

};
