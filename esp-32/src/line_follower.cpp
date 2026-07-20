#include "line_follower.h"
#include <Arduino.h>
#include "constants.h"
#include "pid_config.h"

LineFollower::LineFollower() : line_followng_pid_(PidController(LINE_PID_P, LINE_PID_I, LINE_PID_D, LINE_PID_MAX_I)), prev_state_({0, 0, 0, 0})
{
    pinMode(LEFT_PHOTORESISTOR, INPUT);
    pinMode(MID_LEFT_PHOTORESISTOR, INPUT);
    pinMode(MID_RIGHT_PHOTORESISTOR, INPUT);
    pinMode(RIGHT_PHOTORESISTOR, INPUT);
}

double LineFollower::calculateCorrection()
{
    std::array<double, 4> current_state = readPhotoresistors();
    double correction = 0;
    // steer left if mid-left is on the line and mid-right isn't (robot drifted right)
    if (current_state[1] == 1 && current_state[2] == 0)
    {
        correction = line_followng_pid_.step(-SMALL_ERROR_VALUE, CONTROL_LOOP_PERIOD);
    prev_state_ = current_state;
    }
    // steer right if mid-right is on the line and mid-left isn't (robot drifted left)
    else if (current_state[1] == 0 && current_state[2] == 1)
    {
        correction = line_followng_pid_.step(SMALL_ERROR_VALUE, CONTROL_LOOP_PERIOD);
    prev_state_ = current_state;
    }
    // both mids lost the line — use previous mid state as memory for big correction
    else if (current_state[1] == 0 && current_state[2] == 0)
    {
        if (prev_state_[1] == 1 && prev_state_[2] == 0)
        {
            correction = line_followng_pid_.step(-BIG_ERROR_VALUE, CONTROL_LOOP_PERIOD);
        }
        else if (prev_state_[1] == 0 && prev_state_[2] == 1)
        {
            correction = line_followng_pid_.step(BIG_ERROR_VALUE, CONTROL_LOOP_PERIOD);
        }
    }
    return correction;
}

// Issue: For metal detecting, we will need to spin the robot arm while following line to be efficient. But our robot cannot support both actions at once.
std::array<double, 4> LineFollower::readPhotoresistors()
{
    std::array<double, 4> current_state = {};
    current_state[0] = analogRead(LEFT_PHOTORESISTOR);
    current_state[1] = analogRead(MID_LEFT_PHOTORESISTOR);
    current_state[2] = analogRead(MID_RIGHT_PHOTORESISTOR);
    current_state[3] = analogRead(RIGHT_PHOTORESISTOR);

    for (int i = 0; i < 4; i++)
    {
        current_state[i] = (current_state[i] > LIGHT_THRESHOLD_ADC) ? 1.0 : 0.0;
    }
    return current_state;
}
