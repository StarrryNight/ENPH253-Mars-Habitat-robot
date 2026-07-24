#include <Arduino.h>
#include "arm.h"
#include "pins.h"


Arm::Arm()
	: bus_servo_(6),
	  wrist_servo_(WRIST_YAW_SERVO_PIN),
	  claw_servo_(CLAW_OPEN_SERVO_PIN),
	  // Home pose (base=0deg, elbow=70deg) — the only combination guaranteed
	  // in range for both bus servos (base: [0,70], elbow: [23,70], see
	  // STSServo's SERVO_1_*/SERVO_2_* calibration in servo.h). Applied fresh
	  // by begin() on every setup(), not just used as a default.
	  current_pose_({0, 25, 0, 0})
{
}

void Arm::begin()
{
	// Single-wire half-duplex UART: rx and tx are the same physical pin.
	// Always reset to the home pose on startup, regardless of whatever
	// current_pose_ might otherwise hold.
	setPose({-5, 25, 120, 55});
}


void Arm::setPose(ArmPose pose)
{
	
	current_pose_ = pose;
	wrist_servo_.setAngleDeg(pose.wrist_yaw_servo_degrees);
	claw_servo_.setAngleDeg(pose.claw_servo_degrees);
	bus_servo_.setAngle(pose.base_pitch_servo_degrees, pose.elbow_pitch_servo_degrees);
}


