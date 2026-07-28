#pragma once
#include <cstddef>

enum class RobotState {
	FINDING_ROCK,
	METAL_DETECTING,
	// Entered from METAL_DETECTING when the RPi reports a Teletubby instead
	// of (or in addition to) a metal hit — see AI::tickMetalDetecting/
	// AI::notifyTeletubbyDetected. Runs kTeletubbySequence (robot_poses.h),
	// then falls back to PICKUP_ROCK if metal was also detected, otherwise
	// nextRockOrDone() — see AI::tickTeletubbying.
	TELETUBBYING,
	SCANNING_ROCK,
	PICKUP_ROCK,
	LINE_FOLLOWING,
	HABITAT_PICKUP,
	LINE_FOLLOWING_REVERSE,
	HABITAT_PLACE,
	// Rotates in place (fixed small omega, one direction) until the line
	// follower's two middle sensors both detect the line, then hands off to
	// AI::post_reacquire_state_. Entered by any sequence-driven state once
	// its arm sequence completes, since rotating to arbitrary pose headings
	// can leave the robot facing off the line. See AI::tickReacquiringLine.
	REACQUIRING_LINE,
	DONE,
};

static constexpr size_t kNumRobotStates = 10;

// Debug/serial-print helper — not used by any control-flow logic.
inline const char* robotStateName(RobotState s) {
	switch (s) {
		case RobotState::FINDING_ROCK: return "FINDING_ROCK";
		case RobotState::METAL_DETECTING: return "METAL_DETECTING";
		case RobotState::TELETUBBYING: return "TELETUBBYING";
		case RobotState::SCANNING_ROCK: return "SCANNING_ROCK";
		case RobotState::PICKUP_ROCK: return "PICKUP_ROCK";
		case RobotState::LINE_FOLLOWING: return "LINE_FOLLOWING";
		case RobotState::HABITAT_PICKUP: return "HABITAT_PICKUP";
		case RobotState::LINE_FOLLOWING_REVERSE: return "LINE_FOLLOWING_REVERSE";
		case RobotState::HABITAT_PLACE: return "HABITAT_PLACE";
		case RobotState::REACQUIRING_LINE: return "REACQUIRING_LINE";
		case RobotState::DONE: return "DONE";
	}
	return "?";
}
