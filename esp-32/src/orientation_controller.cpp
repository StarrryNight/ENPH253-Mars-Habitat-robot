#include "orientation_controller.h"

OrientationController::OrientationController() : orientation_pid(PidController(1, 0, 0, 1)),
                                                 wheel_1_init_encoder_count_(0),
                                                 wheel_2_init_encoder_count_(0),
                                                 wheel_3_init_encoder_count_(0) {};

double OrientationController::calculateCorrection(double target_angle)
{
    double current_angle = encoderToPosition();
    return orientation_pid.step(target_angle - current_angle, 10);
}

double OrientationController::encoderToPosition()
{
    return 0;
}

void OrientationController::reset()
{
    wheel_1_init_encoder_count_ = 0;
    wheel_2_init_encoder_count_ = 0;
    wheel_3_init_encoder_count_ = 0;
    orientation_pid.reset();
}