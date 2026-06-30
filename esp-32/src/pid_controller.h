#pragma once

// Discrete PID controller with integral clamping.
// Call step() at a fixed or measured interval; call reset() on state transitions
// to prevent integral wind-up from carrying across modes.
class PidController{

	public:

	// max_i clamps the running integral to prevent wind-up.
	PidController(double p, double i, double d, double max_i);

	// Advances the controller by one timestep.
	// error: setpoint minus measurement (positive = under target).
	// delta_t: seconds since last call.
	// Returns the control output to add to the actuator command.
	double step(double error, double delta_t);

	// Zeroes the accumulated integral. Call on mode/state transitions.
	void reset();

	private:

	double k_p_;
	double k_d_;
	double k_i_;
	double max_integral_; // integral is clamped to [-max_integral_, max_integral_]
	double current_integral_;


};
