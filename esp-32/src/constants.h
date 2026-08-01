#pragma once
#include <cstdint>

static constexpr int CONTROL_LOOP_PERIOD_US = 10000;        
static constexpr double CONTROL_LOOP_PERIOD = CONTROL_LOOP_PERIOD_US * 1e-6; 

static constexpr int MOTOR_CONTROL_LOOP_PERIOD_US = 10000;  
static constexpr double MOTOR_CONTROL_LOOP_PERIOD = MOTOR_CONTROL_LOOP_PERIOD_US * 1e-6; 

static constexpr double ENCODER_RESOLUTION_DISTANCE_M = 0.009687; // at 74 mm diameter (37 mm measured radius), 24 ticks-per-rev

// Forward speed during line following (m/s). Tune empirically.
// Forward speed (m/s) for the ordinary line-following legs — the rock phase,
// the ramp, the plain LINE_FOLLOWING run. See AI::lineFollowingSpeed().
static constexpr double FORWARD_SPEED = 0.20;

// Forward speed (m/s) for the habitat line-following legs
// (HABITAT_LINE_FOLLOWING, LINE_FOLLOWING_REVERSE). Slower because those legs
// end on a sensor event that has to be caught precisely — the marker strip and
// the place marker — and everything downstream of them is fixed distances and
// angles measured from wherever the robot stopped. The rock legs have no such
// constraint, so they run at FORWARD_SPEED.
static constexpr double HABITAT_FORWARD_SPEED = 0.1;

