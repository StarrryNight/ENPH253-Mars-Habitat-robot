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

// Distance (m) between the two OUTER photoresistors — the FULL left-centre to
// right-centre span, not the half-span from the middle of the array.
//
// Used to turn the travel measured between their two triggers on the habitat
// strip into a heading error. The two sensors are the ends of a segment of this
// length; yawing it by theta offsets the ends along the direction of travel by
// span * sin(theta), so the segment is the hypotenuse and the measured travel
// the opposite side: theta = asin(travel / span). (Not atan — that would make
// this the adjacent side, i.e. the array's projection onto the strip, which is
// span * cos(theta), not span.)
//
// Measured on the robot.
static constexpr double SIDE_SENSOR_SPACING_M = 0.13;
static constexpr double TURNING_RADIUS = 0.13;
static constexpr double SMALL_ERROR_VALUE = FORWARD_SPEED*0.8;
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

	// Samples the photoresistor array once and latches the result, updating the
	// prev_state_ memory the big-correction branch of calculateCorrection()
	// depends on. Must be called once per control loop tick by MrKrabs,
	// unconditionally — including while rotating, strafing, settling or running
	// an arm sequence. Those are exactly the manoeuvres that sweep the line
	// across the array, so skipping them would leave prev_state_ describing
	// wherever the line sat before the manoeuvre, and the first big correction
	// after line-following resumes would steer off that stale side.
	//
	// Everything below reads the latched values rather than sampling again, so a
	// single tick's decisions are all made against the same reading.
	void tick();

	// Returns a lateral (x-axis) velocity correction from this tick's latched
	// reading. Positive x = steer right, negative x = steer left.
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

	// True while the left outer sensor detects the line. Paired with
	// rightSensorOnLine() by AI::tickLineFollowing to tell *which* side reached
	// the habitat strip first, which is the direction information
	// bothSideSensorsOnLine() throws away — see RobotState::HABITAT_SQUARE_UP.
	bool leftSensorOnLine();

private:
	// Returns binarised sensor readings (1 = on line, 0 = off line)
	// ordered as {left, mid-left, mid-right, right}.
	std::array<double, 4> readPhotoresistors();

	// This tick's reading, latched by tick() and read by everything else.
	std::array<double, 4> current_state_;
	// Last reading that still had a mid sensor on the line, so it records which
	// side the line was on — used when both mids lose it. Deliberately not
	// overwritten by an all-mids-off reading; see tick().
	std::array<double, 4> prev_state_;
	PidController line_followng_pid_;
};
