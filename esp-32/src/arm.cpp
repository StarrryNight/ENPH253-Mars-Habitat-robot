#include <Arduino.h>
#include "arm.h"
#include "pins.h"
#include <math.h>


Arm::Arm()
	: bus_servo_(6),
	  wrist_servo_(WRIST_YAW_SERVO_PIN),
	  claw_servo_(CLAW_OPEN_SERVO_PIN),
	  // Home pose (base=0deg, elbow=70deg) — the only combination guaranteed
	  // in range for both bus servos (base: [0,70], elbow: [23,70], see
	  // STSServo's SERVO_1_*/SERVO_2_* calibration in servo.h). Applied fresh
	  // by begin() on every setup(), not just used as a default.
	  current_pose_({45, 29, 0, 0})
{
}

void Arm::begin()
{
	// Single-wire half-duplex UART: rx and tx are the same physical pin.
	// Always reset to the home pose on startup, regardless of whatever
	// current_pose_ might otherwise hold.
	setPose({-5, 30, 120, 55});
}


void Arm::setPose(ArmPose pose)
{
	
	current_pose_ = pose;
	wrist_servo_.setAngleDeg(pose.wrist_yaw_servo_degrees);
	claw_servo_.setAngleDeg(pose.claw_servo_degrees);
	bus_servo_.setAngle(pose.base_pitch_servo_degrees, pose.elbow_pitch_servo_degrees, pose.base_pitch_servo_speed, pose.elbow_pitch_servo_speed);
}

void Arm::setPoseXY(ArmCoordinate pose)
{
	current_poseXY_ = pose;
	Arm::setPose(Arm::coordinateToDegrees(pose));
}

ArmPose Arm::coordinateToDegrees(ArmCoordinate pose)
{
	Serial.printf("xypo:  %f.2, ypose:  %f.2, \n",pose.x_pos, pose.y_pos);
	pose.x_pos -= 92.73/1000;
	pose.y_pos += 72.5/1000;
	double q2 = -acos((pow(pose.x_pos, 2) + pow(pose.y_pos, 2) - pow(UPPERARM_LENGTH, 2) - pow(FOREARM_LENGTH, 2))/(2*UPPERARM_LENGTH*FOREARM_LENGTH));
	double q1 = atan(pose.y_pos/ pose.x_pos) + atan(FOREARM_LENGTH*sin(-q2)/( UPPERARM_LENGTH + FOREARM_LENGTH*cos(-q2)));
	Serial.printf("q1:  %f.2, q2:  %f.2, \n",q1* RAD_TO_DEG
, q2* RAD_TO_DEG
);

	double base_pitch_servo_degrees = 90 - q1 * RAD_TO_DEG;
	double elbow_pitch_servo_degrees = -(q1+q2) * RAD_TO_DEG;
	Serial.printf("%f.2, %f.2, \n",base_pitch_servo_degrees, elbow_pitch_servo_degrees);
	return {base_pitch_servo_degrees, elbow_pitch_servo_degrees, pose.wrist_yaw_servo_degrees, pose.claw_servo_degrees, pose.base_pitch_servo_speed, pose.elbow_pitch_servo_speed};
}
