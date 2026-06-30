#include "pid_controller.h"

class OrientationController
{
public:
    OrientationController();

    double calculateCorrection(double target_angle);
    void startRotation();
    void reset();

private:
    double encoderToPosition();

    PidController orientation_pid;
    int wheel_1_init_encoder_count_;
    int wheel_2_init_encoder_count_;
    int wheel_3_init_encoder_count_;
};