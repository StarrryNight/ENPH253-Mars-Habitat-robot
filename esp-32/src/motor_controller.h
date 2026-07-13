#pragma once
#include "motor_driver.h"
#include "pid_controller.h"
#include <optional>
#include <cstdint>

// Per-wheel speed targets in m/s (positive = forward for that wheel's orientation).
struct WheelVelocities
{
	double wheel_1; // front-left (150°)
	double wheel_2; // front-right (30°)
	double wheel_3; // back (270°)
};

// Robot-frame velocity in SI units.
// x: rightward (m/s), y: forward (m/s), omega: counter-clockwise (rad/s).
struct RobotVelocity
{
	double x;
	double y;
	double omega;

	RobotVelocity operator+(const RobotVelocity &other) const
	{
		return {x + other.x, y + other.y, omega + other.omega};
	}
};

// Closed-loop velocity controller for the kiwi (3-omni-wheel) drivetrain.
// Accepts robot-frame velocities, converts to per-wheel targets via inverse
// kinematics, then runs independent PID loops per wheel each call to setVelocity.
//
// Call setup() once in Arduino setup() — that is when MotorDriver objects are
// constructed and encoder interrupts are attached.
class MotorController
{
public:
	MotorController();

	// Constructs MotorDrivers and attaches encoder interrupts. Must be called
	// from Arduino setup() after the hardware is initialised.
	void setup();

	// Replaces the current velocity target and runs one PID step.
	void setVelocity(RobotVelocity target_velocity);

	// Applies a one-shot correction on top of the last setVelocity target.
	// Does NOT modify the stored base target; the next setVelocity() resets it.
	void addVelocity(RobotVelocity correction_velocity);

	// Open-loop drive: converts v directly to wheel PWM via euclidean-to-wheel,
	// no PID. Still calls tickVelocity() each call to keep encoder counts live.
	void driveOpenLoop(RobotVelocity v);

	// Returns accumulated heading (rad) estimated from wheel encoder deltas.
	// Uses kiwi-drive kinematics: omega = (w1+w2+w3) / (3*R).
	double computeAngle();

	void tickMotorSpeeds();

	// Initialize rotation control, activated once when we start rotating in place.
	// Resets the accumulated rotation distance; per-tick deltas come from each
	// MotorDriver's tickSpeed() (must run before this is called each tick — see
	// MrKrabs::stepControl).
	void startRotation();
	// Returns total rotation (degrees) accumulated since startRotation(), computed by
	// adding each wheel's most recent tickSpeed() delta every time this is called,
	// then converting distance to angle via wheel-to-euclidean.
	double calculateRotationDegrees();
private:

	// Kiwi-drive inverse kinematics: maps robot-frame (x, y, omega) to wheel speeds (m/s).
	// Wheel angles from +Y (forward): wheel_1=150°, wheel_2=30°, wheel_3=270°.
	WheelVelocities euclideanToWheel(RobotVelocity v);
	RobotVelocity wheelToEuclidean(WheelVelocities v);

	// Runs one PID step for all three wheels and writes PWM output.
	void applyVelocity_(RobotVelocity target);

	PidController wheel_1_pid_;
	PidController wheel_2_pid_;
	PidController wheel_3_pid_;

	// Motors are held in optionals so the MotorController object can be
	// constructed safely before Arduino init; setup() emplaces them.
	std::optional<MotorDriver> wheel_1_motor_;
	std::optional<MotorDriver> wheel_2_motor_;
	std::optional<MotorDriver> wheel_3_motor_;

	RobotVelocity current_target_velocity_;
	WheelVelocities current_wheel_velocities_;
	uint64_t prev_step_time_us_;  // µs from esp_timer_get_time(); avoids std::chrono unreliability
	double accumulated_angle_;    // radians; updated each applyVelocity_ call

	// Running total (m) of signed rotation distance accumulated one control-loop
	// delta at a time since startRotation().
	double accumulated_rotation_distance_m_;
	// Distance from robot center to each wheel contact point (m).
	static constexpr double WHEEL_DISTANCE_FROM_CENTER_M = 0.3;
};
