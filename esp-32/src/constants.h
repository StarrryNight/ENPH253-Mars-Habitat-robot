#pragma once
#include <cstdint>

static constexpr int CONTROL_LOOP_PERIOD_US = 10000;        
static constexpr double CONTROL_LOOP_PERIOD = CONTROL_LOOP_PERIOD_US * 1e-6; 

static constexpr int MOTOR_CONTROL_LOOP_PERIOD_US = 10000;  
static constexpr double MOTOR_CONTROL_LOOP_PERIOD = MOTOR_CONTROL_LOOP_PERIOD_US * 1e-6; 

static constexpr double ENCODER_RESOLUTION_DISTANCE_M = 0.009687; // at 74 mm diameter (37 mm measured radius), 24 ticks-per-rev

// Forward speed during line following (m/s). Tune empirically.
static constexpr double FORWARD_SPEED = 0.12;

