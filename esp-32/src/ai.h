#pragma once
#include <cstddef>
#include "arm.h"
#include "robot_events.h"

// A RobotEvent is considered close enough to trigger once AI::current_state_progress_m
// is within this distance (m) of the event's robot_progress. See AI::tickAI.
static constexpr double POSE_EVENT_PROGRESS_TOLERANCE_M = 0.02;

class AI{
	public:
		AI();

		void tickAI();

		// Called by MrKrabs once the drivetrain has reached the current target
		// heading. Only then does the arm pose actually get written to the
		// servos. Returns true iff this call just wrote a new pose (so the
		// caller knows to start a settle delay).
		bool onRotationReached();

		void setArm(Arm* arm);

		double targetRotationDegrees() const;
		void setAIProgress(double prog);

		enum class DriveCommand { LINE_FOLLOWING, APPLYING_ROBOT_POSE };
		DriveCommand desiredDriveCommand() const;

	private:
		void applyRobotPose(const RobotPose& pose);

		static constexpr size_t kNoEvent = SIZE_MAX;

		RobotState current_robot_state_;
		// State of robot in meters
		double current_state_progress_m;

		Arm* arm_;

		size_t pending_event_index_;
		bool pending_pose_applied_;
		double current_target_rotation_degrees_;

		DriveCommand desired_drive_command_;
};
