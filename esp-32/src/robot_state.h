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
	// Line-following leg that ends at the habitat: the only line-following state
	// watching for the habitat marker's perpendicular strip (both side sensors
	// -> HABITAT_APPROACH_BACKUP, one side sensor -> HABITAT_SQUARE_UP). Named
	// for that rather than called plain LINE_FOLLOWING, since the rock legs
	// line-follow under FINDING_ROCK and must NOT divert on a side sensor
	// clipping tape. Drives with DriveMode::LINE_FOLLOWING like any other leg.
	// See AI::tickHabitatLineFollowing.
	HABITAT_LINE_FOLLOWING,
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
	// Like REACQUIRING_LINE, but the direction is forced clockwise instead of
	// derived from last_rotation_sign_ — first rotates a fixed 30° clockwise,
	// then continues spinning clockwise (reactive, no fixed target) until
	// both middle sensors detect the line, then hands off to
	// AI::post_reacquire_state_. Guarantees a full ~180° reversal regardless
	// of which way the robot last turned. See AI::tickReverse180.
	REVERSE_180,
	// Entered from LINE_FOLLOWING the moment ONE side sensor triggers on the
	// habitat marker's perpendicular strip — which means the robot met the strip
	// at an angle, since a square approach trips both at once. Stops translating
	// and rotates in place, in the direction that swings the sensor that hasn't
	// triggered yet forward onto the strip, until both read it. That squares the
	// robot to the strip before the habitat legs (which are all fixed distances
	// and headings) start relying on its heading. See AI::tickHabitatSquareUp.
	HABITAT_SQUARE_UP,
	// Entered from LINE_FOLLOWING once both side sensors trigger on the
	// habitat marker. Drives straight backward a fixed distance
	// (kHabitatApproachBackupDistanceM) before HABITAT_FIND starts its
	// sonar-based strafe search, to clear the wall it just line-followed up
	// to. See AI::tickHabitatApproachBackup.
	HABITAT_APPROACH_BACKUP,
	HABITAT_FIND,
	// Drives straight backward a fixed distance (kHabitatBackupDistanceM)
	// after HABITAT_FIND locates the slot, before HABITAT_PICKUP runs its arm
	// sequence — clears the habitat wall the sonar just read close-range.
	// See AI::tickHabitatBackup.
	HABITAT_BACKUP,
	// Entered once HABITAT_PICKUP's arm sequence completes. Drives straight
	// backward a fixed distance (kHabitatPostPickupBackupDistanceM) before
	// HABITAT_HOLD_AND_MOVE strafes back onto the line. See
	// AI::tickHabitatPostPickupBackup.
	HABITAT_POST_PICKUP_BACKUP,
	// Entered once HABITAT_POST_PICKUP_BACKUP's backward leg completes.
	// Strafes opposite the direction HABITAT_FIND strafed to reach this
	// slot, undoing that lateral offset, until both mid line sensors are on
	// the line. See AI::tickHabitatHoldAndMove.
	HABITAT_HOLD_AND_MOVE,
	DONE,
};

static constexpr size_t kNumRobotStates = 18;

// Debug/serial-print helper — not used by any control-flow logic.
inline const char* robotStateName(RobotState s) {
	switch (s) {
		case RobotState::FINDING_ROCK: return "FINDING_ROCK";
		case RobotState::ROTATING_TIL_ROCK: return "ROTATING_TIL_ROCK";
		case RobotState::METAL_DETECTING: return "METAL_DETECTING";
		case RobotState::TELETUBBYING: return "TELETUBBYING";
		case RobotState::PICKUP_ROCK: return "PICKUP_ROCK";
		case RobotState::HABITAT_LINE_FOLLOWING: return "HABITAT_LINE_FOLLOWING";
		case RobotState::HABITAT_PICKUP: return "HABITAT_PICKUP";
		case RobotState::LINE_FOLLOWING_REVERSE: return "LINE_FOLLOWING_REVERSE";
		case RobotState::HABITAT_PLACE: return "HABITAT_PLACE";
		case RobotState::REACQUIRING_LINE: return "REACQUIRING_LINE";
		case RobotState::REVERSE_180: return "REVERSE_180";
		case RobotState::HABITAT_SQUARE_UP: return "HABITAT_SQUARE_UP";
		case RobotState::HABITAT_APPROACH_BACKUP: return "HABITAT_APPROACH_BACKUP";
		case RobotState::HABITAT_FIND: return "HABITAT_FIND";
		case RobotState::HABITAT_BACKUP: return "HABITAT_BACKUP";
		case RobotState::HABITAT_POST_PICKUP_BACKUP: return "HABITAT_POST_PICKUP_BACKUP";
		case RobotState::HABITAT_HOLD_AND_MOVE: return "HABITAT_HOLD_AND_MOVE";
		case RobotState::DONE: return "DONE";
	}
	return "?";
}
