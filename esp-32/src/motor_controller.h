#pragma once
#include "motor_driver.h"
#include "pid_controller.h"
#include "pins.h"
#include <optional>
#include <cstdint>

// Motor PWM channels (LEDC channel numbers, not pins — see pins.h for the wiring)
static constexpr int WHEEL_LEFT_PWM_CHANNEL_0 = 0;
static constexpr int WHEEL_LEFT_PWM_CHANNEL_1 = 1;
// "LEFT" on altium

static constexpr int WHEEL_RIGHT_PWM_CHANNEL_0 = 2;
static constexpr int WHEEL_RIGHT_PWM_CHANNEL_1 = 3;
// "RIGHT" on altium

static constexpr int WHEEL_BACK_PWM_CHANNEL_0 = 4;
static constexpr int WHEEL_BACK_PWM_CHANNEL_1 = 5;
// "BACK" on altium

// Per-wheel speed targets in m/s (positive = forward for that wheel's orientation).
struct WheelVelocities
{
	double wheel_left; // front-left (150°)
	double wheel_right; // front-right (30°)
	double wheel_back; // back (270°)
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

	// Replaces the current velocity target. Does not write PWM directly —
	// the next applyVelocity() call (driven by motor_control_loop_timer_)
	// picks it up.
	void setVelocity(RobotVelocity target_velocity);


	// Runs one PID step for all three wheels and writes PWM output.
	void applyVelocity();

	// Open-loop drive: converts v directly to wheel PWM via euclidean-to-wheel,
	// no PID. Still calls tickVelocity() each call to keep encoder counts live.
	void driveOpenLoop(RobotVelocity v);

	// Immediately zeroes PWM output and clears the wheel PIDs' accumulated
	// state. Unlike setVelocity({0,0,0}), which only changes the target and
	// lets applyVelocity's incremental PID (pwm[n] = pwm[n-1] + Kp*error) wind
	// itself down over several ticks, this snaps the wheels to a stop right
	// away — used when teleop releases all keys.
	void stopImmediate();

	// Returns accumulated heading (rad) estimated from wheel encoder deltas.
	// Uses kiwi-drive kinematics: omega = (w_left+w_right+w_back) / (3*R).
	double computeAngle();

	void tickMotorSpeeds();

	// Resyncs all three wheels' speed baselines to their current encoder counts.
	// Call once, right before the motor control loop timer starts — see
	// MotorDriver::resetSpeedBaseline.
	void resetSpeedBaselines();

	// Initialize rotation control, activated once when we start rotating in place.
	// Resets the accumulated rotation distance; per-tick deltas come from each
	// MotorDriver's tickSpeed() (must run before this is called each tick — see
	// MrKrabs::stepControl).
	void startRotation();

	// Folds this tick's per-wheel encoder deltas (from tickMotorSpeeds(), which
	// must run first this tick) into the cached elapsed-rotation angle. Called
	// once per motor-control tick by MrKrabs::motorStepControl(), so rotation
	// tracking stays synchronized with the same 10 ms cadence the deltas are
	// computed on rather than being recomputed whenever some other timer's
	// callback happens to ask.
	void updateRotationTracking();

	// Returns the cached rotation (rad) accumulated since startRotation(), as
	// of the most recent updateRotationTracking() call. Pure getter — safe to
	// call from any tick (e.g. a different timer than motorStepControl's)
	// without perturbing the accumulation.
	//
	// Valid ONLY for a pure in-place rotation: it takes the largest-magnitude
	// per-wheel estimate, each of which assumes that wheel's whole travel is
	// R*angle. Any translation in the command makes it read that translation as
	// rotation. Use getYawDriftRad() while translating.
	double getElapsedRotation() const;

	// Rotation (rad) since startRotation() computed from the kiwi drive's
	// forward kinematics — omega = (v_left + v_right + v_back) / (3R) — over the
	// same signed encoder counts getElapsedRotation() reads. Because the three
	// wheel contributions of any pure translation sum to zero, what survives is
	// yaw alone, so this stays valid while the robot is also driving/strafing.
	// That makes it the estimate to feed OrientationController::holdCorrection.
	//
	// Same caveats as the counts themselves: sign comes from each wheel's
	// commanded direction (single-phase encoders), so a slipping or back-driven
	// wheel reads as if it turned the commanded way, and resolution is
	// ENCODER_RESOLUTION_DISTANCE_M / (3R) = 0.0297 rad (1.7 deg) per tick.
	double getYawDriftRad() const;

	// Folds this tick's measured (filtered) robot velocity into the running
	// translation odometer (total_translation_m_) via velocity * dt —
	// unlike updateRotationTracking()'s exact-encoder-count approach for
	// heading, this integrates getCurrentRobotVelocity() instead, since the
	// exact-count approach proved less reliable in practice. Must run after
	// tickMotorSpeeds() each tick (see MrKrabs::motorStepControl).
	void updateTranslationTracking();

	// Monotonic total path length (m) traveled since construction/last reset,
	// integrated from measured velocity (see updateTranslationTracking()).
	// Callers diff successive reads to get incremental distance — see
	// MrKrabs::driveCurrentMode.
	double getTotalTranslationM() const;

	// Zeros the translation odometer. Called when leaving LINE_FOLLOWING for
	// a rotation/arm-pose sequence, so it starts fresh at 0 the next time
	// line-following resumes instead of carrying forward stale distance.
	void resetTranslationTracking();

	// Displacement from where the robot started, in the frame it started in:
	// x rightward and y forward *as of power-on*, not the current robot frame.
	// heading_rad is the accumulated yaw (CCW positive) those two are
	// integrated against. Distinct from getTotalTranslationM(), which is a
	// scalar path length that gets reset per leg — this is a whole-run
	// position and is never reset. See updateTranslationTracking().
	struct FieldPose {
		double x_m;
		double y_m;
		double heading_rad;
	};
	FieldPose getFieldPose() const;

	RobotVelocity getCurrentRobotVelocity();
private:

	// Kiwi-drive inverse kinematics: maps robot-frame (x, y, omega) to wheel speeds (m/s).
	// Wheel angles from +Y (forward): wheel_left=150°, wheel_right=30°, wheel_back=270°.
	WheelVelocities euclideanToWheel(RobotVelocity v);
	RobotVelocity wheelToEuclidean(WheelVelocities v);


	PidController wheel_left_pid_;
	PidController wheel_right_pid_;
	PidController wheel_back_pid_;

	// Motors are held in optionals so the MotorController object can be
	// constructed safely before Arduino init; setup() emplaces them.
	std::optional<MotorDriver> wheel_left_motor_;
	std::optional<MotorDriver> wheel_right_motor_;
	std::optional<MotorDriver> wheel_back_motor_;

	RobotVelocity current_target_velocity_;
	WheelVelocities current_wheel_velocities_;
	uint64_t prev_step_time_us_;  // µs from esp_timer_get_time(); avoids std::chrono unreliability
	double accumulated_angle_;    // radians; updated each applyVelocity_ call

	// Rotation (rad) since startRotation(), recomputed each tick by
	// updateRotationTracking() directly from the signed cumulative encoder
	// counts below rather than integrated tick-by-tick from filtered
	// velocity — avoids compounding the velocity moving-average's smoothing/
	// rounding error over a long rotation. External callers should read
	// getElapsedRotation() instead of this directly.
	double current_rotation_rad_;

	// Signed cumulative encoder pulses per wheel since startRotation() —
	// exact integer running totals (not distance/angle yet), incremented
	// each tick in updateRotationTracking() by
	// getCurrentDeltaCount() * getIntendedDirection() (the encoder itself is
	// single-phase/unsigned, so direction has to come from the commanded
	// sign). Converted to distance/angle only when read, so there's no
	// repeated floating-point accumulation to drift.
	int64_t wheel_left_rotation_count_;
	int64_t wheel_right_rotation_count_;
	int64_t wheel_back_rotation_count_;

	// Monotonic path length (m) accumulated by updateTranslationTracking().
	double total_translation_m_ = 0.0;

	// Field-frame displacement (m) and heading (rad) since construction — see
	// getFieldPose(). Never reset, unlike total_translation_m_: the distance
	// legs need a per-leg odometer, this is a whole-run position. The heading
	// is integrated here in updateTranslationTracking() rather than reusing
	// accumulated_angle_, which is only advanced by applyVelocity() — that
	// returns early while open-loop drive is active, which would freeze the
	// heading and silently rotate the rest of the run's displacement.
	double field_x_m_ = 0.0;
	double field_y_m_ = 0.0;
	double field_heading_rad_ = 0.0;
	// Distance from robot center to each wheel contact point (m). Measured
	// on hardware as 10 cm — was previously 0.3 (3x too large), which made
	// current_rotation_rad_ (∝ 1/R) systematically underestimate true
	// rotation, so the rotate-to-angle loop kept driving well past the real
	// target before its own deflated estimate caught up.
	static constexpr double WHEEL_DISTANCE_FROM_CENTER_M = 0.1087;

	RobotVelocity current_robot_velocity_;

	// True while driveOpenLoop() is the active source of truth for wheel PWM.
	// applyVelocity() is called unconditionally every 10ms by an independent
	// timer (see MrKrabs::motorStepControl) — without this flag it would
	// keep re-driving the wheels from the stale current_target_velocity_ via
	// PID and undo whatever driveOpenLoop() just wrote directly. setVelocity()
	// clears it to hand control back to the closed-loop PID path.
	bool open_loop_active_ = false;
};
