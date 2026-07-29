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

	// This sequence's pose_settle_us (see robot_poses.h) — how long the
	// caller should hold still after onRotationReached() applies a pose
	// before applying the next. 0 if start() was never called.
	uint64_t poseSettleUs() const;

	// Applies the current pose to the arm and advances to the next one.
	// Returns true iff a pose was actually written (caller should start a
	// settle delay); false if the sequence is already complete.
	bool onRotationReached(Arm& arm);

private:
	const ArmPoseSequence* sequence_ = nullptr;
	size_t index_ = 0;
};

// Same rotate-once/apply-each cadence as SequenceRunner, but for
// ArmXYSequence (robot_poses.h): poses are ArmCoordinate (XY), applied via
// Arm::setPoseXY, and any pose with x_is_sonar_relative=true has its x_pos
// resolved as an offset from a runtime sonar reading rather than an absolute
// position. Used for METAL_DETECTING/PICKUP_ROCK (sonar-relative, no
// rotation) and HABITAT_PICKUP/HABITAT_PLACE (fixed rotation, absolute
// poses) — see AI::onRotationReached.
class XySequenceRunner {
public:
	void start(const ArmXYSequence& sequence, double sonar_x_origin_m);

	// True once every pose in the sequence has been applied (or start() was
	// never called).
	bool complete() const;

	// How many poses have been applied so far (0 if start() was never
	// called). Lets callers gate a RobotState transition on a specific pose
	// having actually been written — see AI::tickBackingUpRock.
	size_t currentIndex() const;

	double targetRotationDegrees() const;

	uint64_t poseSettleUs() const;

	// Resolves the current pose's x (offset + cached sonar origin, or the
	// pose's own x_pos if not sonar-relative), applies it to the arm, and
	// advances to the next one. Returns true iff a pose was actually written.
	bool onRotationReached(Arm& arm);

private:
	const ArmXYSequence* sequence_ = nullptr;
	size_t index_ = 0;
	double sonar_x_origin_m_ = 0.0;
};
