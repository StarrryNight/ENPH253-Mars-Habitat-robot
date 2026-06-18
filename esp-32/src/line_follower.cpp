#include "line_follower.h"
#include <Arduino.h>
#include "constants.h"

LineFollower::LineFollower() : line_followng_pid_(PidController(1, 0, 2, 2))
{
    pinMode(LEFT_PHOTORESISTOR, INPUT);
    pinMode(MID_LEFT_PHOTORESISTOR, INPUT);
    pinMode(MID_RIGHT_PHOTORESISTOR, INPUT);
    pinMode(RIGHT_PHOTORESISTOR, INPUT);
}

void LineFollower::followLine()
{
    std::array<double, 4> current_state = readPhotoresistors();
    // steer left slightly if left photoresistor is on line and right isn't
    if (current_state[1] == 1 && current_state[2] == 0)
    {
        line_followng_pid_.step(-SMALL_ERROR_VALUE, CONTROL_LOOP_PERIOD);
    }
    // steer right slightly if right photoresistor is on line and left isn't
    if (current_state[1] == 0 && current_state[2] == 1)
    {
        line_followng_pid_.step(SMALL_ERROR_VALUE, CONTROL_LOOP_PERIOD);
    }
    // big changes
    if (current_state[1] == 0 && current_state[2] == 0)
    {
        // steer left greatly if left photoresistor is on line and right isn't
        if (prev_state_[0] == 1 && prev_state_[1] == 0)
        {
            line_followng_pid_.step(-BIG_ERROR_VALUE, CONTROL_LOOP_PERIOD);
        }
        // steer right greatly if right photoresistor is on line and left isn't
        elif (prev_state_[0] == 0 && prev_state_[1] == 1)
        {
            line_followng_pid_.step(BIG_ERROR_VALUE, CONTROL_LOOP_PERIOD);
        }
    }
    prev_state_ = current_state;
}

std::array<double, 4> LineFollower::readPhotoresistors()
{
    std::array<double, 4> current_state = {};
    current_state[0] = analogRead(LEFT_PHOTORESISTOR);
    current_state[1] = analogRead(MID_LEFT_PHOTORESISTOR);
    current_state[2] = analogRead(MID_RIGHT_PHOTORESISTOR);
    current_state[3] = analogRead(RIGHT_PHOTORESISTOR);

    for (int i = 0 : i < 4; i++)
    {
        current_state[i] = 1 ? current_state[i] > LIGHT_THRESHOLD_V : 0;
    }
    return currentState;
}