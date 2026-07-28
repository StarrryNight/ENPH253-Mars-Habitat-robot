#pragma once
#include "SCServo.h"
#include "servo.h"
#include "servo_controller.h"
#include "pins.h"
#include "arm.h"

struct ArmPose{
	double base_pitch_servo_degrees;
	double elbow_pitch_servo_degrees;
	double wrist_yaw_servo_degrees;
	double claw_servo_degrees;
	// Bus servo (STSServo) speeds for this pose — omit to fall back to
	// DEFAULT_SPEED, so existing 4-field {a,b,c,d} pose literals still compile.
	double base_pitch_servo_speed = DEFAULT_SPEED;
	double elbow_pitch_servo_speed = DEFAULT_SPEED;

	static constexpr double DEFAULT_SPEED = 500;
};

/*
(x_pos=0, y_pos=0) := back shoulder joint
(x_pos, y_pos) := front of claw body
*/
struct ArmCoordinate{
	double x_pos;
	double y_pos;
	double wrist_yaw_servo_degrees;
	double claw_servo_degrees;
	// Mirrors ArmPose's speed fields — coordinateToDegrees() forwards these
	// through so a caller can request a fast/slow move for a given target.
	double base_pitch_servo_speed = ArmPose::DEFAULT_SPEED;
	double elbow_pitch_servo_speed = ArmPose::DEFAULT_SPEED;
};

class Arm
{

public:

	static constexpr double WRIST_CENTER = 120;
	static constexpr double WRIST_PLACE = 15;
	static constexpr double CLAW_OPEN = 50;
	static constexpr double CLAW_CLOSE = 0;
	static constexpr double UPPERARM_LENGTH = 0.2;
	static constexpr double FOREARM_LENGTH = 0.15;
	Arm();

	// Binds the SCServo bus to Serial2 and drives the arm to its home pose.
	// Must be called after Serial2.begin() has run (Arduino init + UART
	// config), so it can't happen in the constructor.
	void begin();

	void setPose(ArmPose pose);
	void setPoseXY(ArmCoordinate pose);
	ArmPose coordinateToDegrees(ArmCoordinate pose);



private:
	int ServoPositionConversion(double degrees);


	STSServo bus_servo_;
	Servo wrist_servo_;
	Servo claw_servo_;

	ArmPose current_pose_;
	ArmCoordinate current_poseXY_;

};
