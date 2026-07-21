#pragma once
#include <cstddef>

enum class RobotState {
	// Hardware validation state: rotate 90 degrees then -90 degrees, arm
	// neutral throughout. Temporary — set as AI's initial state while the
	// drivetrain/rotation pipeline is being bring-up tested; not part of the
	// real CLAUDE.md state machine.
	TEST_ROTATION,
	FINDING_ROCK,
	METAL_DETECTING,
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
