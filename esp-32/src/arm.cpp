#include "arm.h"
#include "constants.h"

const Arm::PoseAngles Arm::kPoseAngles[] = {
	{90, 90, 90, 90},
	{90, 90, 90, 90},
	{90, 90, 90, 90},
	{90, 90, 90, 90},
};

Arm::Arm()
	: base_servo_(BASE_ELBOW_PITCH_SERIAL_PIN),
	  elbow_servo_(BASE_ELBOW_PITCH_SERIAL_PIN),
	  wrist_servo_(yaw_wrist),
	  claw_servo_(open_claw),
	  current_pose_(METAL)
{
	servo_controller_.addServo(&base_servo_);
	servo_controller_.addServo(&elbow_servo_);
	servo_controller_.addServo(&wrist_servo_);
	servo_controller_.addServo(&claw_servo_);
	setPose(current_pose_);
}

void Arm::tickArm()
{
	servo_controller_.update();
}

void Arm::setPose(ArmPoses pose)
{
	current_pose_ = pose;
	const PoseAngles &angles = kPoseAngles[pose];
	base_servo_.setAngleDeg(angles.base_pitch_deg);
	elbow_servo_.setAngleDeg(angles.elbow_pitch_deg);
	wrist_servo_.setAngleDeg(angles.wrist_yaw_deg);
	claw_servo_.setAngleDeg(angles.claw_deg);
}
