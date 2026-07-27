#pragma once
#include <cstdint>
#include <vector>
#include "arm.h"
#include "robot_state.h"

// A single relative rotation applied once, then a sequence of arm poses
// applied one at a time at that heading (settling between each) — see
// SequenceRunner. Unlike the old per-pose-rotation RobotPose/RobotSequence,
// every pose in `poses` is applied at the SAME heading; there is no way to
// rotate again partway through. Sequences that used to rotate back to a
// different heading for their last pose (e.g. HABITAT_PICKUP) now stay at
// the sequence's one rotation and rely on REACQUIRING_LINE's reverse-spin
// (see AI::nextRockOrDone/tickHabitatPickup) to return toward the line
// afterward — same pattern METAL_DETECTING already used.
struct ArmPoseSequence {
	double rotation_degrees;
	std::vector<ArmPose> poses;
	// How long to hold/settle after each pose in this sequence is applied,
	// before the next one is written (see MrKrabs::driveCurrentMode's
	// APPLYING_SEQUENCE case). Defaults to the same 2.5 s used everywhere
	// else (ACTION_TRANSITION_DELAY_US in mr_krabs.h); sequences that don't
	// need poses to fully settle (e.g. a fast wave/dance) can override this
	// to move through their poses quicker.
	uint64_t pose_settle_us = 2500000;
};

// Placeholder poses — need real tuning against the named poses in CLAUDE.md
// (home, sweep, rock pickup, rock deposit, habitat grab, habitat place) once
// the arm is characterized on hardware.

// Per-rock scan sequences (rotation + sweep poses) live in ai.cpp as
// kRockCheckpoints, alongside the per-rock progress trigger — see
// AI::transitionTo. This file only holds sequences that are the same
// every time their state is entered.

const ArmPoseSequence kPickupRockSequence = {
	0.0, // grab from wherever METAL_DETECTING left off, no extra rotation
	{
		{40, 55, Arm::WRIST_CENTER, Arm::CLAW_OPEN},
		{45, 45, Arm::WRIST_CENTER, Arm::CLAW_OPEN},
		{45, 45, Arm::WRIST_CENTER, Arm::CLAW_CLOSE},
		{45, 20, Arm::WRIST_CENTER, Arm::CLAW_CLOSE},
		{-5, 20, Arm::WRIST_CENTER, Arm::CLAW_CLOSE},
		{-5, 20, Arm::WRIST_PLACE, Arm::CLAW_CLOSE},
		{-5, 20, Arm::WRIST_PLACE, Arm::CLAW_OPEN},
		{-5, 25, 120, 55}, // reset position
	}
};

const ArmPoseSequence kHabitatPickupSequence = {
	90.0,
	{
		{45, 60, 0, 0},  // reach toward habitat, claw open
		{45, 60, 0, 90}, // grab, claw closed
		{90, 90, 0, 90}, // retract, holding item
	}
};

const ArmPoseSequence kHabitatPlaceSequence = {
	90.0,
	{
		{45, 60, 0, 90}, // reach into habitat, still holding
		{45, 60, 0, 0},  // release, claw open
		{0, 90, 0, 0},   // retract, arm home
	}
};
// Faster settle (0.4 s vs. the 2.5 s default) — this is a wave/dance, not a
// precise grab, so poses can be strung together quickly.
const ArmPoseSequence kTeletubbySequence = {
	0,
	{
		{40, 55, 160, Arm::CLAW_OPEN},
		{40, 55, 80, Arm::CLAW_OPEN},
		{40, 55, 160, Arm::CLAW_OPEN},
		{40, 55, 80, Arm::CLAW_OPEN},
		{40, 55, 160, Arm::CLAW_OPEN},
		{40, 55, 80, Arm::CLAW_OPEN},
	},
	700000,
};

// Looks up the ArmPoseSequence to run when entering state. Returns nullptr
// for states with no fixed arm sequence — the line-following states, DONE,
// and METAL_DETECTING (which is per-rock; see AI::transitionTo) — see
// AI::desiredDriveMode.
inline const ArmPoseSequence* sequenceForState(RobotState state) {
	switch (state) {
		case RobotState::TELETUBBYING: return &kTeletubbySequence;
		case RobotState::PICKUP_ROCK: return &kPickupRockSequence;
		case RobotState::HABITAT_PICKUP: return &kHabitatPickupSequence;
		case RobotState::HABITAT_PLACE: return &kHabitatPlaceSequence;
		default: return nullptr;
	}
}
