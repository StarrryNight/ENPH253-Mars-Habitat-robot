#include "orientation_controller.h"
#include "constants.h"
#include "pid_config.h"
#include <algorithm>
#include <cmath>
#include <Arduino.h>

OrientationController::OrientationController()
    : orientation_pid(PidController(ORIENTATION_PID_P, ORIENTATION_PID_I, ORIENTATION_PID_D, ORIENTATION_PID_MAX_I)),
      hold_pid_(PidController(STRAFE_HOLD_PID_P, STRAFE_HOLD_PID_I, STRAFE_HOLD_PID_D, STRAFE_HOLD_PID_MAX_I)),
      target_angle_(0) {}

double OrientationController::calculateCorrection(double current_angle)
{
	Serial.printf("current rot = %.5f\n", current_angle);
    double correction = orientation_pid.step(target_angle_ - current_angle, CONTROL_LOOP_PERIOD);
    if (std::abs(correction) < MIN_ROTATION_OMEGA_RAD_S) {
        correction = std::copysign(MIN_ROTATION_OMEGA_RAD_S, correction);
    }
    return correction;
}

double OrientationController::holdCorrection(double current_angle)
{
    double error = target_angle_ - current_angle;
    if (std::abs(error) < HOLD_DEADBAND_RAD) {
        // Straight enough. Reset so the next real excursion starts from clean
        // state rather than inheriting an integral wound up by encoder noise
        // while the robot was tracking fine.
        hold_pid_.reset();
        return 0.0;
    }
    double correction = hold_pid_.step(error, CONTROL_LOOP_PERIOD);
    return std::clamp(correction, -MAX_HOLD_OMEGA_RAD_S, MAX_HOLD_OMEGA_RAD_S);
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
    hold_pid_.reset();
}
