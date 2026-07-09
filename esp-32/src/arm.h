#pragma once
#include "servo.h"
#include "servo_controller.h"

class Arm
{

public:
	enum ArmPoses {METAL, GRAB_HABITAT, HOLD_HABITAT, PLACE_HABITAT};

	Arm();

	void tickArm();
	void setPose(ArmPoses pose);

private:
	struct PoseAngles
	{
		double base_pitch_deg;
		double elbow_pitch_deg;
		double wrist_yaw_deg;
		double claw_deg;
	};

	static const PoseAngles kPoseAngles[];

	Servo base_servo_;
	Servo elbow_servo_;
	Servo wrist_servo_;
	Servo claw_servo_;
	ServoController servo_controller_;

	ArmPoses current_pose_;
};
