#pragma once
#include <vector>
#include "arm.h"

struct RobotPose{
	double rotation_degrees;
	ArmPose arm_pose;
};

enum RobotState{LINE_FOLLOWING, METAL_DETECTING, ROCK_PICKING, TELETUBBYING, HABITAT_PICKING};

struct RobotEvent{
	RobotState robot_state;
	double robot_progress;
	RobotPose robot_pose;
};

// Placeholder poses/progress thresholds — need real tuning against the named
// poses in CLAUDE.md (home, sweep, rock pickup, rock deposit, habitat grab,
// habitat place) once the arm is characterized on hardware.
const std::vector<RobotEvent> DUMMY_ROBOT_EVENTS = {
	// Metal detecting: sweep the arm out to scan for metal.
	{METAL_DETECTING, 0.0, {0.0, {90, 90, 90, 90}}},
	{METAL_DETECTING, 0.15, {0, {0, 0, 0, 0}}},

	// Rock picking: lower arm, open claw, close claw around rock, lift.
	{ROCK_PICKING, 0.0, {0.0, {45, 30, 0, 0}}},
	{ROCK_PICKING, 0.05, {0.0, {45, 30, 0, 90}}},
	{ROCK_PICKING, 0.10, {0.0, {90, 90, 0, 90}}},

	// Teletubbying: point flashlight/arm toward the Teletubby.
	{TELETUBBYING, 0.0, {90.0, {60, 45, 90, 0}}},

	// Habitat picking: reach into habitat, place rock, retract.
	{HABITAT_PICKING, 0.0, {90.0, {45, 60, 0, 90}}},
	{HABITAT_PICKING, 0.05, {90.0, {45, 60, 0, 0}}},
	{HABITAT_PICKING, 0.10, {0.0, {0, 90, 0, 0}}},
};
