#pragma once
#include <fcntl.h>
#include "constants.h"
#include "pid_config.h"
#include "pid_controller.h"
#include "motor_controller.h"
#include "pins.h"
#include <array>

// Raw 12-bit ADC threshold (0–4095). Calibrate on your surface.
static constexpr double LIGHT_THRESHOLD_ADC = 2000;
static constexpr double TURNING_RADIUS = 0.13;
static constexpr double SMALL_ERROR_VALUE = FORWARD_SPEED*1.4;
static constexpr double BIG_ERROR_VALUE =FORWARD_SPEED/(TURNING_RADIUS);

// IR photoresistor line follower with PID-based lateral correction.
// Uses a 4-sensor array: [left, mid-left, mid-right, right].
// The two middle sensors drive small corrections; when both mids lose the line,
// the previous mid state is used as memory to pick a large correction direction.
// calculateCorrection() returns an x-axis RobotVelocity nudge; pass it to
// MotorController::setVelocity({correction.x, FORWARD_SPEED, 0}) each control tick.
class LineFollower
{

public:
	LineFollower();

	// Reads photoresistors and returns a lateral (x-axis) velocity correction.
	// Positive x = steer right, negative x = steer left.
	double calculateCorrection();

	// True once both middle sensors detect the line — used by
	// AI::tickReacquiringLine to tell when a search rotation has reoriented
	// the robot back onto the line.
	bool bothMidSensorsOnLine();

	// True once both outer (left, right) sensors detect the line — the line
	// crossing the habitat marker's perpendicular strip triggers both side
	// sensors at once. Used by AI::tickLineFollowing to hand off to
	// RobotState::HABITAT_FIND.
	bool bothSideSensorsOnLine();

	// True once the right sensor alone detects the line — the habitat place
	// marker triggers just this one. Used by AI::tickLineFollowingReverse to
	// hand off to RobotState::HABITAT_PLACE.
	bool rightSensorOnLine();

private:
	// Returns binarised sensor readings (1 = on line, 0 = off line)
	// ordered as {left, mid-left, mid-right, right}.
	std::array<double, 4> readPhotoresistors();

	std::array<double, 4> prev_state_; // last reading; used when both mids lose the line
	PidController line_followng_pid_;
};
