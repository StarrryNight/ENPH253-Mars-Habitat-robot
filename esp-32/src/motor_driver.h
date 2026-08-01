#pragma once
#include <Arduino.h>
#include <cstdint>
#include <vector>

// Moving-average window (ticks, at MOTOR_CONTROL_LOOP_PERIOD_US each) for
// getSpeed(). Bigger = smoother velocity reading (also consumed by
// MotorController::updateRotationTracking() for elapsed-rotation tracking)
// but more lag in the wheel velocity PID's feedback in applyVelocity() —
// don't push this much further without checking the PID still responds
// crisply to target changes.
static constexpr int VELOCITY_BUFFER_SIZE = 7;

// Below this commanded PWM magnitude, treat speed as zero instead of applying
// the deadband floor below. Prevents FP/PID noise around a 0 target (e.g. the
// back wheel while driving straight) from producing a small nonzero duty cycle.
static constexpr double MOTOR_SPEED_DEADZONE = 5.0;

// Fallback for MotorDriver's pwm_offset constructor argument, used only when a
// caller doesn't pass one. Real values live in pid_config.h next to the rest of
// the per-wheel tuning (WHEEL_*_PWM_OFFSET) — this just preserves the behavior
// the offset had when it was hardcoded in speedToDutyCycle().
static constexpr int MOTOR_PWM_OFFSET_DEFAULT = 30;

// Controls a single DC motor with quadrature encoder feedback.
// PWM is driven via ESP32 LEDC channels. Velocity is measured by tickVelocity(),
// which is called once per control loop tick (every 10 ms) by MotorController —
// no separate per-motor timer is used.
// All setup happens in the constructor — no separate begin() call needed.
class MotorDriver
{

public:
	// pwm_channel_0/1: LEDC channels (must be unique per motor).
	// pwm_pin_0 drives forward, pwm_pin_1 drives reverse.
	// encoder_pin_0: single-phase encoder input (RISING edge counted).
	// pwm_offset: static duty added to every nonzero command for this motor —
	//   see pwm_offset_ below. Per-motor because the three drivetrains don't
	//   break loose at the same duty. Tuned in pid_config.h
	//   (WHEEL_*_PWM_OFFSET); MOTOR_PWM_OFFSET_DEFAULT if omitted.
	MotorDriver(int wheel_number, int pwm_channel_0, int pwm_channel_1, int pwm_pin_0, int pwm_pin_1, int encoder_pin_0,
	            int pwm_offset = MOTOR_PWM_OFFSET_DEFAULT);

	// Direct PWM control. Zeroes the opposite channel before applying speed.
	// These do NOT include a direction-change delay; use set_velocity for the
	// closed-loop path which handles H-bridge protection automatically.
	void rotateClockwise(int speed);
	void rotateCounterClockwise(int speed);

	// Signed wrapper: speed > 0 = clockwise, speed < 0 = counter-clockwise, 0 = stop.
	// Applies a 10 ms coast-to-stop delay only when direction actually reverses,
	// protecting the H-bridge from shoot-through without blocking every tick.
	void set_velocity(double speed);

	// Returns the last raw speed passed to set_velocity (pre-deadzone, pre-quantization).
	double getCurrentTargetSpeed();

	// The duty actually written to the LEDC channels by the most recent
	// set_velocity(), signed by the direction it was written in (+ = channel 0,
	// - = channel 1). This is the number after the deadzone, after +pwm_offset_
	// and after the clamp to 255 — i.e. what the H-bridge really got, which is
	// what getCurrentTargetSpeed() deliberately does NOT tell you. Zero whenever
	// the write was suppressed: deadzone, a direction flip, or waiting out the
	// shoot-through guard.
	int getLastDutyCycle();

	// Returns the raw cumulative encoder count since construction (monotonic, unsigned,
	// wraps at 2^32 — callers must diff with unsigned arithmetic to handle wraparound).
	uint32_t getEncoderCount();

	// Computes wheel surface speed (m/s) from encoder count delta since last call.
	// Must be called once per control loop tick (10 ms) by MotorController.
	void tickSpeed();

	// Resyncs the speed baseline to the current encoder count. Call once, right
	// before the periodic timer that drives tickSpeed() starts, so pulses
	// accumulated between construction and that point aren't misread as a huge
	// velocity on the first tick.
	void resetSpeedBaseline();

	double getSpeed();

	// Encoder count delta computed by the most recent tickSpeed() call, i.e.
	// ticks accumulated over the last control loop period (10 ms). Callers that
	// tick on the same 10 ms cadence (e.g. rotation tracking) can reuse this
	// instead of independently diffing getEncoderCount().
	uint32_t getCurrentDeltaCount();

	// Sign of the most recent set_velocity() command (-1, 0, or 1) — the same
	// unsmoothed direction signal tickSpeed() uses internally to sign raw_speed
	// before it goes into the velocity moving average. Callers that want to
	// sign getCurrentDeltaCount() themselves (e.g. rotation tracking, to avoid
	// depending on getSpeed()'s smoothing) should use this rather than
	// re-deriving direction from getSpeed() or getCurrentTargetSpeed().
	int getIntendedDirection();

	// True while this motor is sitting out the shoot-through guard after a
	// commanded direction reversal — PWM forced to zero, waiting for
	// SHOOTTHROUGH_GUARD_THRESHOLD_US before the new direction is applied.
	// MotorController::applyVelocity() uses it to hold that wheel's velocity PID
	// in reset for the duration: the wheel physically cannot follow a command
	// during the coast, so the error accumulating into the integral is error the
	// controller had no way to act on, and it would otherwise dump into the
	// first PWM write after the guard clears.
	bool isDirectionChangePending() const;

	// Forgets the last driven direction, so the next set_velocity() call —
	// whichever direction it's for — is treated as a direction change and
	// goes through the coast/dead-time guard in set_velocity() instead of
	// jumping straight to full duty cycle. Call this on a deliberate full
	// stop (MotorController::stopImmediate) so a kickstart PWM applied on
	// the next start still coasts one control cycle first, same as a real
	// direction reversal. Distinct from the ordinary new_direction==0 path
	// in set_velocity, which intentionally leaves last_direction_ alone to
	// preserve tickSpeed()'s sign fallback during deadzone dithering. Also
	// clears the velocity moving-average buffer (same as a real direction
	// reversal), so getSpeed() reads 0 immediately instead of decaying down
	// from stale pre-stop samples.
	void primeForRestart();

	// Drops the velocity moving-average samples without touching last_direction_.
	// Use when a new leg begins and the old samples describe a different motion
	// (MotorController::startRotation) — getSpeed() would otherwise report the
	// previous leg's velocity for up to VELOCITY_BUFFER_SIZE ticks and the wheel
	// PID would correct against it. Deliberately NOT primeForRestart(): that also
	// forgets the driven direction, which sends the next command through the
	// shoot-through guard for a tick of coast. That's right after a real stop,
	// wrong when the wheels are still turning.
	void resetSpeedFilter();

private:
	int speedToDutyCycle(int speed);

	const int wheel_number_;


	int intended_direction_; // Tracks the immediate sign of incoming commands
	volatile double wheel_speed_;
	double motor_target_speed_; // last raw speed passed to set_velocity
	int last_direction_;                  // -1, 0, or 1 — tracks last direction for H-bridge protection and speed tracking
	int last_duty_cycle_ = 0;             // signed duty actually written; see getLastDutyCycle()

	const int pwm_channel_0_;
	const int pwm_channel_1_;

	const int pwm_pin_0_;
	const int pwm_pin_1_;

	const int encoder_pin_0_;

	// Static duty added to the magnitude of every nonzero command before the
	// clamp to 255 (see speedToDutyCycle). Covers the duty a motor needs just
	// to overcome stiction/gearbox drag, so a small commanded speed produces
	// actual rotation instead of a stalled hum — the feedforward gain alone
	// can't express that, being proportional to speed. Consequences to keep in
	// mind when tuning: the smallest nonzero speed this motor can produce is
	// whatever this duty spins it at, and the headroom above it shrinks to
	// (255 - offset), so the command rails at a lower speed the higher it goes.
	const int pwm_offset_;

	// uint32_t: wraps instead of overflowing at ~2 billion counts. Wraparound is
	// handled correctly as long as deltas are computed with unsigned subtraction.
	volatile uint32_t encoder_count_;
	uint32_t prev_measurement_encoder_count_; // snapshot taken each tickVelocity call
	uint32_t last_delta_count_;               // delta computed by the most recent tickSpeed() call
	uint64_t prev_pwm_reset_time_;
	bool pending_direction_change_;


	double velocity_count_buffer_[VELOCITY_BUFFER_SIZE];
	int buffer_index_;
	int buffer_filled_count_; // number of valid samples so far, caps at VELOCITY_BUFFER_SIZE
	static constexpr uint64_t SHOOTTHROUGH_GUARD_THRESHOLD_US = 10000;

};
