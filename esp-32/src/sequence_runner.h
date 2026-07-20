#pragma once
#include <cstddef>
#include "arm.h"
#include "robot_poses.h"

// Steps through a RobotSequence one pose at a time: exposes the current
// pose's target heading for the drivetrain to rotate toward, and applies the
// pose to the arm once that heading is reached. Used by AI for every
// sequence-driven state (METAL_DETECTING, PICKUP_ROCK, HABITAT_PICKUP,
// HABITAT_PLACE). See MrKrabs::driveCurrentMode for the rotate/apply/settle
// cycle this participates in.
class SequenceRunner {
public:
	void start(const RobotSequence& sequence);

	// True once every pose in the sequence has been applied (or start() was
	// never called).
	bool complete() const;

	double targetRotationDegrees() const;

	// Applies the current pose to the arm and advances to the next one.
	// Returns true iff a pose was actually written (caller should start a
	// settle delay); false if the sequence is already complete.
	bool onRotationReached(Arm& arm);

private:
	const RobotSequence* sequence_ = nullptr;
	size_t index_ = 0;
};
