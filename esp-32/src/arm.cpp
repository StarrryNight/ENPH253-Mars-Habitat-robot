#include <Arduino.h>
#include "arm.h"
#include "pins.h"


Arm::Arm()
	: bus_servo_(6),
	  wrist_servo_(WRIST_YAW_SERVO_PIN),
	  claw_servo_(CLAW_OPEN_SERVO_PIN),
	  current_pose_({0,0,0,0})
{
}

void Arm::begin()
{
	// Single-wire half-duplex UART: rx and tx are the same physical pin.
	setPose(current_pose_);
}


void Arm::setPose(ArmPose pose)
{
	current_pose_ = pose;
	wrist_servo_.setAngleDeg(pose.wrist_yaw_servo_degrees);
	claw_servo_.setAngleDeg(pose.claw_servo_degrees);
	bus_servo_.setAngle(pose.base_pitch_servo_degrees, pose.elbow_pitch_servo_degrees);
}


