#include "pid_controller.h"
#include "motor_controller.h"
#include <array>
class LineFollower
{

public:
    /*
     * Constructor of the line follower class.
     * Uses PID
     */
    LineFollower();

    /*
     *
     */
    void calculateCorrection();

private:
    // formatted as {left, midleft, midright, right}
    std::array<double, 4> readPhotoresistors();
    std::array<double, 4> prev_state_;
    PidController line_followng_pid_;
};