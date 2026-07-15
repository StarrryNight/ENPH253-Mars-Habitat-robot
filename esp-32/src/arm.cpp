#include "arm.h"


Arm::Arm()
	: bus_servo_(),
	  wrist_servo_(WRIST_YAW_SERVO_PIN),
	  claw_servo_(CLAW_OPEN_SERVO_PIN),
	  current_pose_({0,0,0,0})
{
}

void Arm::begin()
{
	bus_servo_.pSerial = &Serial2;
	setPose(current_pose_);
}


void Arm::setPose(ArmPose pose)
{
	current_pose_ = pose;
	bus_servo_.WritePosEx(/*ID=*/1, /*Position=*/ServoPositionConversion(pose.base_pitch_servo_degrees), /*Speed=*/2400, /*ACC=*/50); // ~150 deg
	bus_servo_.WritePosEx(/*ID=*/2, /*Position=*/ServoPositionConversion(pose.elbow_pitch_servo_degrees), /*Speed=*/34900, /*ACC=*/50); // ~150 deg
	wrist_servo_.setAngleDeg(pose.wrist_yaw_servo_degrees);
	claw_servo_.setAngleDeg(pose.claw_servo_degrees);
}

int Arm::ServoPositionConversion(double degrees){
	return degrees/180*(SERIAL_SERVO_MAX - SERIAL_SERVO_MIN)+SERIAL_SERVO_MIN;
}

