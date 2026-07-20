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
	DONE,
};

static constexpr size_t kNumRobotStates = 9;
