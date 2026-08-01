#include "line_follower.h"
#include <Arduino.h>
#include "constants.h"
#include "pid_config.h"

LineFollower::LineFollower() : line_followng_pid_(PidController(LINE_PID_P, LINE_PID_I, LINE_PID_D, LINE_PID_MAX_I)), current_state_({0, 0, 0, 0}), prev_state_({0, 0, 0, 0})
{
    pinMode(LEFT_PHOTORESISTOR, INPUT);
    pinMode(MID_LEFT_PHOTORESISTOR, INPUT);
    pinMode(MID_RIGHT_PHOTORESISTOR, INPUT);
    pinMode(RIGHT_PHOTORESISTOR, INPUT);
}

void LineFollower::tick()
{
    current_state_ = readPhotoresistors();
    // Only latch a reading that still has a mid sensor on the line. prev_state_
    // exists purely to remember which side the line was last seen on, so
    // storing an all-mids-off reading would erase the one thing the big-error
    // branch below needs to pick a direction.
    if (current_state_[1] == 1 || current_state_[2] == 1)
    {
        prev_state_ = current_state_;
    }
}

double LineFollower::calculateCorrection(double forward_speed)
{
    const std::array<double, 4>& current_state = current_state_;
    // Sized off the speed being driven, not off FORWARD_SPEED — see
    // smallErrorValue/bigErrorValue in the header.
    const double small_error = smallErrorValue(forward_speed);
    const double big_error = bigErrorValue(forward_speed);
    double correction = 0;
    // Both mids on the line: dead center, no drift to correct. Reset the PID
    // so stale integral/derivative from the last correction doesn't leak
    // into the next one.
    if (current_state[1] == 1 && current_state[2] == 1)
    {
        line_followng_pid_.reset();
    }
    // steer left if mid-left is on the line and mid-right isn't (robot drifted right)
    else if (current_state[1] == 1 && current_state[2] == 0)
    {
        correction = line_followng_pid_.step(-small_error, CONTROL_LOOP_PERIOD);
    }
    // steer right if mid-right is on the line and mid-left isn't (robot drifted left)
    else if (current_state[1] == 0 && current_state[2] == 1)
    {
        correction = line_followng_pid_.step(small_error, CONTROL_LOOP_PERIOD);
    }
    // both mids lost the line — use previous mid state as memory for big correction
    else if (current_state[1] == 0 && current_state[2] == 0)
    {
        if (prev_state_[1] == 1 && prev_state_[2] == 0)
        {
            correction = line_followng_pid_.step(-big_error, CONTROL_LOOP_PERIOD);
        }
        else if (prev_state_[1] == 0 && prev_state_[2] == 1)
        {
            correction = line_followng_pid_.step(big_error, CONTROL_LOOP_PERIOD);
        }
    }
	//Serial.printf("%f.2\n", correction);
    return correction;
}

//currently changed
bool LineFollower::bothMidSensorsOnLine()
{
    return current_state_[1] == 1 || current_state_[2] == 1;
}

bool LineFollower::bothSideSensorsOnLine()
{
    return current_state_[0] == 1 && current_state_[3] == 1;
}

bool LineFollower::rightSensorOnLine()
{
    return current_state_[3] == 1;
}

bool LineFollower::leftSensorOnLine()
{
    return current_state_[0] == 1;
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
