#pragma once
#include <vector>
#include "arm.h"
#include "robot_state.h"

// A target heading (deg) plus the arm pose to apply once that heading is
// reached. See SequenceRunner.
struct RobotPose {
	double rotation_degrees;
	ArmPose arm_pose;
};

// An ordered list of poses applied one at a time: rotate to rotation_degrees,
// apply arm_pose, settle, then move to the next entry. See SequenceRunner.
using RobotSequence = std::vector<RobotPose>;

// Placeholder poses — need real tuning against the named poses in CLAUDE.md
// (home, sweep, rock pickup, rock deposit, habitat grab, habitat place) once
// the arm is characterized on hardware.

const RobotSequence kMetalDetectingSequence = {
	{0.0, {90, 90, 90, 90}}, // sweep arm out to scan for metal
	{0.0, {0, 0, 0, 0}},     // rest
};

const RobotSequence kPickupRockSequence = {
	{0.0, {45, 30, 0, 0}},  // lower arm, claw open
	{0.0, {45, 30, 0, 90}}, // close claw around rock
	{0.0, {90, 90, 0, 90}}, // lift, holding rock
};

const RobotSequence kHabitatPickupSequence = {
	{90.0, {45, 60, 0, 0}},  // reach toward habitat, claw open
	{90.0, {45, 60, 0, 90}}, // grab, claw closed
	{0.0, {90, 90, 0, 90}},  // retract, holding item
};

const RobotSequence kHabitatPlaceSequence = {
	{90.0, {45, 60, 0, 90}}, // reach into habitat, still holding
	{90.0, {45, 60, 0, 0}},  // release, claw open
	{0.0, {0, 90, 0, 0}},    // retract, arm home
};

// Looks up the RobotSequence to run when entering state. Returns nullptr for
// states with no arm sequence (the line-following states, and DONE) — see
// AI::desiredDriveMode.
inline const RobotSequence* sequenceForState(RobotState state) {
	switch (state) {
		case RobotState::METAL_DETECTING: return &kMetalDetectingSequence;
		case RobotState::PICKUP_ROCK: return &kPickupRockSequence;
		case RobotState::HABITAT_PICKUP: return &kHabitatPickupSequence;
		case RobotState::HABITAT_PLACE: return &kHabitatPlaceSequence;
		default: return nullptr;
	}
}
