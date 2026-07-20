#include "ai.h"
#include <cmath>
#include "robot_events.h"

AI::AI():
	current_robot_state_(TEST_ROTATION),
	current_state_progress_m(0),
	arm_(nullptr),
	pending_event_index_(kNoEvent),
	pending_pose_applied_(true),
	current_target_rotation_degrees_(0),
	desired_drive_command_(DriveCommand::LINE_FOLLOWING)
{}

void AI::setArm(Arm* arm){
	arm_ = arm;
}

void AI::tickAI(){
	for (size_t i = 0; i < DUMMY_ROBOT_EVENTS.size(); ++i){
		const RobotEvent& event = DUMMY_ROBOT_EVENTS[i];
		bool close_enough = event.robot_state == current_robot_state_ &&
			std::abs(event.robot_progress - current_state_progress_m) < POSE_EVENT_PROGRESS_TOLERANCE_M;
		if (close_enough && i != pending_event_index_){
			pending_event_index_ = i;
			current_target_rotation_degrees_ = event.robot_pose.rotation_degrees;
			pending_pose_applied_ = false;
			break;
		}
	}

	desired_drive_command_ = (current_robot_state_ == LINE_FOLLOWING)
		? DriveCommand::LINE_FOLLOWING
		: DriveCommand::APPLYING_ROBOT_POSE;
}

void AI::setAIProgress(double progress){
	current_state_progress_m = progress ;

}

bool AI::onRotationReached(){
	if (pending_event_index_ == kNoEvent || pending_pose_applied_){
		return false;
	}
	applyRobotPose(DUMMY_ROBOT_EVENTS[pending_event_index_].robot_pose);
	pending_pose_applied_ = true;
	return true;
}

double AI::targetRotationDegrees() const{
	return current_target_rotation_degrees_;
}

AI::DriveCommand AI::desiredDriveCommand() const{
	return desired_drive_command_;
}

void AI::applyRobotPose(const RobotPose& pose){
	if (arm_){
		Serial.print(pose.rotation_degrees);
		arm_->setPose(pose.arm_pose);
	}
}
