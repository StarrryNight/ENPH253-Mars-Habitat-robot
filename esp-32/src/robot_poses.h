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

// Same rotate-once/apply-each-pose shape as ArmPoseSequence, but for
// ArmCoordinate (XY) poses instead of ArmPose (degrees) — see
// XySequenceRunner (sequence_runner.h). x_pos on a pose with
// x_is_sonar_relative=true is an offset (m) added to a runtime sonar
// reading rather than an absolute position (see kPickupRockXYSequence);
// rotation_degrees is 0.0 for sequences that don't need to rotate (the
// rock states are already facing the target after ROTATING_TIL_ROCK's
// free-spin).
struct ArmXYSequence {
	double rotation_degrees = 0.0;
	std::vector<ArmCoordinate> poses;
	uint64_t pose_settle_us = 1300000;
};

// Runs during METAL_DETECTING (pose 0) and PICKUP_ROCK (poses 1-6). x_pos for
// poses with x_is_sonar_relative=true is an offset (m) added to the sonar
// reading cached by AI::tickRotatingTilRock, resolved at apply time by
// XySequenceRunner (sequence_runner.h) — not an absolute position. Ported
// verbatim from mr_krabs.cpp's kArmTestPoses bench test, including its
// per-pose sonar offsets and servo speeds.
const ArmXYSequence kPickupRockXYSequence = {
	0.0,
	{
		{0.08,  -0.03, 120, 50, 500, 500, true},   // [0] METAL_DETECTING: reach in, probe for metal
		{-0.02,  0.05, 120, 50, 500, 500, true},   // [1] PICKUP_ROCK: retract slightly
		{0.0,   -0.06, 120, 50, 250, 1000, true},   // [2] PICKUP_ROCK: position around rock, still open
		{0.0,   -0.055,120,  0, 500, 500, true},   // [3] PICKUP_ROCK: close claw
		{0.24,   0.060,120,  0, 500, 500, false},  // [4] PICKUP_ROCK: retract, claw closed
		{0.24,   0.060,Arm::WRIST_PLACE, 0, 500, 500, false}, // [5] PICKUP_ROCK: rotate to place
		{0.24,   0.060,Arm::WRIST_PLACE, 50,500, 500, false}, // [6] PICKUP_ROCK: release
		{0.24,   0.060,120, 50,500, 500, false}, // [6] PICKUP_ROCK: release
	},
	1500000,
};

// No rotation — HABITAT_PICKUP runs in place at whatever heading the robot's
// already at (HABITAT_PLACE does rotate first; see below).
const ArmXYSequence kHabitatPickupXYSequence = {
	0.0,
	{
		{0.30,  0.03, 120, 50, 500, 500},
		{0.31, -0.05, 120, 50, 250, 500},
		{0.36, -0.05, 120, 50, 250, 500},
		{0.36, -0.05, 120,  0, 250, 500}, // close claw — grabbed
		{0.22,0.06, 120,  0, 250, 500}, // retract, holding item
	},
};

// Rotates 38° before the first pose. HABITAT_PLACE is entered from the
// LINE_FOLLOWING_REVERSE leg the instant the right sensor crosses the place
// marker (see AI::tickLineFollowingReverse), which leaves the robot still
// aligned along the line rather than facing the slot — this turn is what
// squares it up. Positive is counter-clockwise (see RobotVelocity::omega);
// negate it to turn the other way.
const ArmXYSequence kHabitatPlaceXYSequence = {
	-55.0,
	{
		{0.28, 0.04, 120,  0, 250, 500}, // reach in, still holding
		{0.18, -0.00, 120,  0, 250, 500}, // reach in, still holdin
		{0.18, -0.00, 120, 50, 250, 500}, // open claw — release
		{0.18, -0.00, 120, 50, 250, 500}, // pull back, open
		{0.21,  0.03, 120, 50, 500, 500}, // retract to home
		{0.23,  0.06, 120, 50, 500, 500},
	},
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

// Single pose (joint angles, not XY) written once on entering REVERSE_180 — see
// AI::transitionTo. Not an ArmPoseSequence: REVERSE_180 has its own DriveMode,
// so sequenceForState() below is never consulted for it and no runner steps
// through poses; the arm just gets this one command and holds it through the
// spin. MrKrabs::startReverse180's ACTION_TRANSITION_DELAY_US settle runs after
// the write, so the servos have that long to arrive before the robot turns.
//
// Claw stays closed: REVERSE_180 is entered mid-carry, holding the item picked
// up at the habitat (HABITAT_HOLD_AND_MOVE -> REVERSE_180), so opening it here
// drops the load. The pitch angles are a starting point — base_pitch is degrees
// from vertical (usable range [0,70]) and elbow_pitch degrees down from
// horizontal ([23,70]), so this pulls the load up and in, close to the turn
// axis. Tune on hardware.
const ArmPose kReverse180ArmPose = {-2, 25, Arm::WRIST_CENTER, Arm::CLAW_CLOSE};

// Looks up the ArmPoseSequence to run when entering state. Returns nullptr
// for states with no fixed arm sequence — the line-following states, DONE,
// and METAL_DETECTING/PICKUP_ROCK/HABITAT_PICKUP/HABITAT_PLACE (all driven by
// ArmXYSequence/XySequenceRunner instead; see AI::transitionTo) — see
// AI::desiredDriveMode.
inline const ArmPoseSequence* sequenceForState(RobotState state) {
	switch (state) {
		case RobotState::TELETUBBYING: return &kTeletubbySequence;
		default: return nullptr;
	}
}
