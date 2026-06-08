class PidController{

	public:

	PidController(double p, double i, double d, double max_i);

	void step();

	void reset();

	private:

	double k_p;
	double k_d;
	double k_i;
	double max_integral;
	double current_integral;
	

};
