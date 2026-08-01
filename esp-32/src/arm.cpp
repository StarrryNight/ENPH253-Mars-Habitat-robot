#include <Arduino.h>
#include "arm.h"
#include "pins.h"
#include <math.h>


Arm::Arm()
	: bus_servo_(6),
	  wrist_servo_(WRIST_YAW_SERVO_PIN),
	  claw_servo_(CLAW_OPEN_SERVO_PIN, servoPulseUsForDeg(CLAW_INITIAL_ANGLE_DEG)),
	  // Home pose (base=0deg, elbow=70deg) — the only combination guaranteed
	  // in range for both bus servos (base: [0,70], elbow: [23,70], see
	  // STSServo's SERVO_1_*/SERVO_2_* calibration in servo.h). Applied fresh
	  // by begin() on every setup(), not just used as a default.
	  current_pose_({0, 23, 0, 30})
{
}

void Arm::begin()
{
	// Single-wire half-duplex UART: rx and tx are the same physical pin.
	// Always reset to the home pose on startup, regardless of whatever
	// current_pose_ might otherwise hold.
	setPose({0, 23, Arm::WRIST_CENTER, 30});
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
	pose.x_pos -= 110.73/1000;
	pose.y_pos += 62.5/1000;
	// Second frame shift on top of the offsets above: callers pass x 113.5 mm
	// lower and y 53.29 mm higher than the values this solver works in, and
	// every ArmCoordinate literal (robot_poses.h, and kArmTestPoses in
	// mr_krabs.cpp) was re-based by those amounts. The extra 5 mm on x beyond
	// the 113.5 is a real reach adjustment — it pushes every commanded pose
	// 5 mm further out.
	pose.x_pos += 118.5/1000;
	pose.y_pos -= 53.29/1000;
	// Measured on hardware: the claw lands 30 mm above the commanded y, so every
	// pose is driven that much lower. Kept as its own line rather than folded
	// into the 53.29 above because that number is a frame shift (geometry) and
	// this one is a calibration error (the real arm vs. the solver's model) —
	// they get re-measured for different reasons.
	pose.y_pos -= 30.0/1000;
	double q2 = -acos((pow(pose.x_pos, 2) + pow(pose.y_pos, 2) - pow(UPPERARM_LENGTH, 2) - pow(FOREARM_LENGTH, 2))/(2*UPPERARM_LENGTH*FOREARM_LENGTH));
	double q1 = atan(pose.y_pos/ pose.x_pos) + atan(FOREARM_LENGTH*sin(-q2)/( UPPERARM_LENGTH + FOREARM_LENGTH*cos(-q2)));

	double base_pitch_servo_degrees = 90 - q1 * RAD_TO_DEG;
	double elbow_pitch_servo_degrees = -(q1+q2) * RAD_TO_DEG;
	return {base_pitch_servo_degrees, elbow_pitch_servo_degrees, pose.wrist_yaw_servo_degrees, pose.claw_servo_degrees, pose.base_pitch_servo_speed, pose.elbow_pitch_servo_speed};
}
