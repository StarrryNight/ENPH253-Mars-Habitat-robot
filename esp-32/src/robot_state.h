#pragma once
#include <cstddef>

enum class RobotState {
	FINDING_ROCK,
	METAL_DETECTING,
	PICKUP_ROCK,
	LINE_FOLLOWING,
	HABITAT_PICKUP,
	LINE_FOLLOWING_REVERSE,
	HABITAT_PLACE,
	DONE,
};

static constexpr size_t kNumRobotStates = 8;
