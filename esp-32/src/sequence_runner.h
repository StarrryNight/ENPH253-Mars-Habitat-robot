#pragma once
#include <cstddef>
#include "arm.h"
#include "robot_poses.h"

// Steps through an ArmPoseSequence one pose at a time: exposes the
// sequence's single target heading for the drivetrain to rotate toward
// once, and applies each pose to the arm in turn once that heading is
// reached. Used by AI for every sequence-driven state (METAL_DETECTING,
// PICKUP_ROCK, HABITAT_PICKUP, HABITAT_PLACE). See MrKrabs::driveCurrentMode
// for the rotate-once/apply-each/settle cycle this participates in.
class SequenceRunner {
public:
	void start(const ArmPoseSequence& sequence);

	// True once every pose in the sequence has been applied (or start() was
	// never called).
	bool complete() const;

	double targetRotationDegrees() const;

	// Applies the current pose to the arm and advances to the next one.
	// Returns true iff a pose was actually written (caller should start a
	// settle delay); false if the sequence is already complete.
	bool onRotationReached(Arm& arm);

private:
	const ArmPoseSequence* sequence_ = nullptr;
	size_t index_ = 0;
};
