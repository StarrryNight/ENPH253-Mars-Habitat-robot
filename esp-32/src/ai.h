#pragma once
#include "arm.h"
#include "robot_event.h"

struct RobotPose{
	double rotation_degrees;
	ArmPose arm_pose;
};


enum RobotState{LINE_FOLLOWING, METAL_DETECTING, ROCK_PICKING, TELETUBBYING, HABITAT_PICKING};


class AI{
	public:
		AI();

		void tickAI();

	private:
		void applyRobotPose();
		RobotState current_robot_state_;
		// State of robot in meters
		double current_state_progress_m;
		
		
};
