#pragma once
#include <cstdint>

static constexpr int CONTROL_LOOP_PERIOD_US = 10000;        
static constexpr double CONTROL_LOOP_PERIOD = CONTROL_LOOP_PERIOD_US * 1e-6; 

static constexpr int MOTOR_CONTROL_LOOP_PERIOD_US = 10000;  
static constexpr double MOTOR_CONTROL_LOOP_PERIOD = MOTOR_CONTROL_LOOP_PERIOD_US * 1e-6; 

static constexpr double ENCODER_RESOLUTION_DISTANCE_M = 0.009163; // at 70 mm diameter, 24 ticks-per-rev

