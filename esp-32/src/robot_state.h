#pragma once
#include <cstddef>

enum class RobotState {
	FINDING_ROCK,
	// Rotates in place (fixed omega, direction from the current rock
	// checkpoint's heading sign) until the sonar reads a distance in the
	// rock-detection range, then caches that distance for the pickup XY
	// sequence. See AI::tickRotatingTilRock.
	ROTATING_TIL_ROCK,
	METAL_DETECTING,
	// Entered from METAL_DETECTING when the RPi reports a Teletubby instead
	// of (or in addition to) a metal hit — see AI::tickMetalDetecting/
	// AI::notifyTeletubbyDetected. Runs kTeletubbySequence (robot_poses.h),
	// then falls back to PICKUP_ROCK if metal was also detected, otherwise
	// nextRockOrDone() — see AI::tickTeletubbying.
	TELETUBBYING,
	// Runs kPickupRockXYSequence: arm retracts slightly (pose 1, right after
	// METAL_DETECTING's probe pose 0), then closes the claw, retracts, and
	// places the rock. See AI::tickPickupRock.
	PICKUP_ROCK,
	LINE_FOLLOWING,
	HABITAT_PICKUP,
	LINE_FOLLOWING_REVERSE,
	HABITAT_PLACE,
	// First rotates a fixed 45° in place, then continues rotating the same
	// direction (reactive, no fixed target) until the line follower's two
	// middle sensors both detect the line, then hands off to
	// AI::post_reacquire_state_. Entered by any sequence-driven state once
	// its arm sequence completes, since rotating to arbitrary pose headings
	// (or, for HABITAT_PICKUP/HABITAT_PLACE, not rotating at all) can leave
	// the robot facing off the line. See AI::tickReacquiringLine.
	REACQUIRING_LINE,
	HABITAT_FIND,
	// Drives straight backward a fixed distance (kHabitatBackupDistanceM)
	// after HABITAT_FIND locates the slot, before HABITAT_PICKUP runs its arm
	// sequence — clears the habitat wall the sonar just read close-range.
	// See AI::tickHabitatBackup.
	HABITAT_BACKUP,
	// Entered once HABITAT_PICKUP's arm sequence completes. Rotates 180°
	// the same way REACQUIRING_LINE does (fixed 45° turn, then reactive
	// continuous spin), but simultaneously strafes opposite the direction
	// HABITAT_FIND strafed to reach this slot, undoing that lateral offset
	// while it turns back toward the line. See AI::tickHabitatHoldAndMove.
	HABITAT_HOLD_AND_MOVE,
	DONE,
};

static constexpr size_t kNumRobotStates = 14;

// Debug/serial-print helper — not used by any control-flow logic.
inline const char* robotStateName(RobotState s) {
	switch (s) {
		case RobotState::FINDING_ROCK: return "FINDING_ROCK";
		case RobotState::ROTATING_TIL_ROCK: return "ROTATING_TIL_ROCK";
		case RobotState::METAL_DETECTING: return "METAL_DETECTING";
		case RobotState::TELETUBBYING: return "TELETUBBYING";
		case RobotState::PICKUP_ROCK: return "PICKUP_ROCK";
		case RobotState::LINE_FOLLOWING: return "LINE_FOLLOWING";
		case RobotState::HABITAT_PICKUP: return "HABITAT_PICKUP";
		case RobotState::LINE_FOLLOWING_REVERSE: return "LINE_FOLLOWING_REVERSE";
		case RobotState::HABITAT_PLACE: return "HABITAT_PLACE";
		case RobotState::REACQUIRING_LINE: return "REACQUIRING_LINE";
		case RobotState::HABITAT_FIND: return "HABITAT_FIND";
		case RobotState::HABITAT_BACKUP: return "HABITAT_BACKUP";
		case RobotState::HABITAT_HOLD_AND_MOVE: return "HABITAT_HOLD_AND_MOVE";
		case RobotState::DONE: return "DONE";
	}
	return "?";
}
