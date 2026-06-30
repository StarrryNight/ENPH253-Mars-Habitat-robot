#include "orientation_controller.h"
#include "constants.h"

OrientationController::OrientationController()
    : orientation_pid(PidController(1, 0, 0, 1)),
      target_angle_(0) {}

double OrientationController::calculateCorrection(double current_angle)
{
    return orientation_pid.step(target_angle_ - current_angle, CONTROL_LOOP_PERIOD);
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
