#include "orientation_controller.h"
#include "constants.h"
#include "pid_config.h"
#include <cmath>

OrientationController::OrientationController()
    : orientation_pid(PidController(ORIENTATION_PID_P, ORIENTATION_PID_I, ORIENTATION_PID_D, ORIENTATION_PID_MAX_I)),
      target_angle_(0) {}

double OrientationController::calculateCorrection(double current_angle)
{
    return orientation_pid.step(target_angle_ - current_angle, CONTROL_LOOP_PERIOD);
}

bool OrientationController::reachedTarget(double current_angle) const
{
    return std::abs(target_angle_ - current_angle) < ROTATION_TOLERANCE_RAD;
}

void OrientationController::startRotation(double target_angle)
{
    target_angle_ = target_angle;
    reset();
}

void OrientationController::reset()
{
    orientation_pid.reset();
}
