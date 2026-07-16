#pragma once
#include "SCServo.h"
#include "servo.h"
#include "servo_controller.h"
#include "pins.h"

struct ArmPose{
	double base_pitch_servo_degrees;
	double elbow_pitch_servo_degrees;
	double wrist_yaw_servo_degrees;
	double claw_servo_degrees;
};

class Arm
{

public:

	Arm();

	// Binds the SCServo bus to Serial2 and drives the arm to its home pose.
	// Must be called after Serial2.begin() has run (Arduino init + UART
	// config), so it can't happen in the constructor.
	void begin();

	void setPose(ArmPose pose);

private:
	int ServoPositionConversion(double degrees);


	STSServo bus_servo_;
	Servo wrist_servo_;
	Servo claw_servo_;

	ArmPose current_pose_;

};
