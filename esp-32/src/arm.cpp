#include "arm.h"
#include "constants.h"


Arm::Arm()
	: bus_servo_(BASE_ELBOW_PITCH_SERIAL_PIN),
	  wrist_servo_(WRIST_YAW_SERVO_PIN),
	  claw_servo_(CLAW_OPEN_SERVO_PIN),
	  current_pose_({0,0,0,0})
{
	setPose(current_pose_);
}


void Arm::setPose(ArmPose pose)
{
	current_pose_ = pose;
	bus_servo_.WritePosEx(/*ID=*/1, /*Position=*/basePitchPositionConversion(pose.base_pitch_servo_degrees), /*Speed=*/2400, /*ACC=*/50); // ~150 deg
	bus_servo_.WritePosEx(/*ID=*/2, /*Position=*/elbowPitchPositionConversion(pose.elbow_pitch_servo_degrees), /*Speed=*/34900, /*ACC=*/50); // ~150 deg
	wrist_servo_.setAngleDeg(pose.wrist_yaw_servo_degrees);
	claw_servo_.setAngleDeg(pose.claw_servo_degrees);
}

int Arm::basePitchPositionConversion(double degrees){
	return degrees/180*(BASE_PITCH_SERVO_MAX - BASE_PITCH_SERVO_MIN)+BASE_PITCH_SERVO_MIN;
}

int Arm::elbowPitchPositionConversion(double degrees){
	return degrees/180*(ELBOW_PITCH_SERVO_MAX - ELBOW_PITCH_SERVO_MIN)+ELBOW_PITCH_SERVO_MIN;
}
