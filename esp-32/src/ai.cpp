#include "ai.h"
#include <array>

// Dummy tuning constants — placeholders pending real localization/testing,
// same status as the arm pose numbers in robot_poses.h.
namespace {
	constexpr int kNumRocks = 6;
	constexpr int kNumHabitatCycles = 2;

	// Distance (m) into FINDING_ROCK's Nth visit at which rock N's checkpoint
	// is reached. Indexed by AI::visits(FINDING_ROCK) - 1.
	constexpr std::array<double, kNumRocks> kRockCheckpointsM = {
		0.3, 0.6, 0.9, 1.2, 1.5, 1.8,
	};

	// Distance (m) into LINE_FOLLOWING's Nth visit at which the habitat is
	// reached. Indexed by AI::visits(LINE_FOLLOWING) - 1.
	constexpr std::array<double, kNumHabitatCycles> kHabitatApproachDistancesM = {
		1.0, 1.0,
	};

	// Distance (m) to drive in reverse before HABITAT_PLACE.
	constexpr double kLineFollowingReverseDistanceM = 1.0;
}

AI::AI():
	arm_(nullptr),
	current_state_(RobotState::TEST_ROTATION),
	current_state_progress_m_(0)
{
	transitionTo(RobotState::TEST_ROTATION);
}

void AI::setArm(Arm* arm){
	arm_ = arm;
}

void AI::addProgress(double delta_m){
	current_state_progress_m_ += delta_m;
}

void AI::transitionTo(RobotState next){
	current_state_ = next;
	state_visit_count_[idx(next)]++;
	current_state_progress_m_ = 0;

	const RobotSequence* sequence = sequenceForState(next);
	if (sequence){
		sequence_runner_.start(*sequence);
	}
}

void AI::tickAI(){
	RobotState next = tickCurrentState();
	if (next != current_state_){
		transitionTo(next);
	}
}

RobotState AI::tickCurrentState(){
	switch (current_state_){
		case RobotState::TEST_ROTATION: return tickTestRotation();
		case RobotState::FINDING_ROCK: return tickFindingRock();
		case RobotState::METAL_DETECTING: return tickMetalDetecting();
		case RobotState::PICKUP_ROCK: return tickPickupRock();
		case RobotState::LINE_FOLLOWING: return tickLineFollowing();
		case RobotState::HABITAT_PICKUP: return tickHabitatPickup();
		case RobotState::LINE_FOLLOWING_REVERSE: return tickLineFollowingReverse();
		case RobotState::HABITAT_PLACE: return tickHabitatPlace();
		case RobotState::DONE: return RobotState::DONE;
	}
	return current_state_;
}

RobotState AI::tickTestRotation(){
	// Hardware validation only — stay here running the sequence, then hand
	// off to DONE (forcing IDLE) once complete instead of falling through:
	// SequenceRunner::targetRotationDegrees() reverts to 0.0 once complete(),
	// which handleDriveTransition() would otherwise read as a fresh target
	// and rotate back to heading 0.
	return sequence_runner_.complete() ? RobotState::DONE : RobotState::TEST_ROTATION;
}

RobotState AI::nextRockOrDone(){
	return visits(RobotState::FINDING_ROCK) >= kNumRocks
		? RobotState::LINE_FOLLOWING
		: RobotState::FINDING_ROCK;
}

RobotState AI::tickFindingRock(){
	int n = visits(RobotState::FINDING_ROCK); // 1-based
	if (n > kNumRocks){
		return RobotState::LINE_FOLLOWING;
	}
	if (current_state_progress_m_ >= kRockCheckpointsM[n - 1]){
		return RobotState::METAL_DETECTING;
	}
	return RobotState::FINDING_ROCK;
}

RobotState AI::tickMetalDetecting(){
	if (!sequence_runner_.complete()){
		return RobotState::METAL_DETECTING;
	}
	return metal_detector_.getMetalDetectorState() ? RobotState::PICKUP_ROCK : nextRockOrDone();
}

RobotState AI::tickPickupRock(){
	if (!sequence_runner_.complete()){
		return RobotState::PICKUP_ROCK;
	}
	return nextRockOrDone();
}

RobotState AI::tickLineFollowing(){
	int n = visits(RobotState::LINE_FOLLOWING); // 1-based
	double trigger_m = kHabitatApproachDistancesM[n - 1];
	return current_state_progress_m_ >= trigger_m ? RobotState::HABITAT_PICKUP : RobotState::LINE_FOLLOWING;
}

RobotState AI::tickHabitatPickup(){
	if (!sequence_runner_.complete()){
		return RobotState::HABITAT_PICKUP;
	}
	return RobotState::LINE_FOLLOWING_REVERSE;
}

RobotState AI::tickLineFollowingReverse(){
	return current_state_progress_m_ >= kLineFollowingReverseDistanceM
		? RobotState::HABITAT_PLACE
		: RobotState::LINE_FOLLOWING_REVERSE;
}

RobotState AI::tickHabitatPlace(){
	if (!sequence_runner_.complete()){
		return RobotState::HABITAT_PLACE;
	}
	return visits(RobotState::HABITAT_PLACE) >= kNumHabitatCycles ? RobotState::DONE : RobotState::LINE_FOLLOWING;
}

AI::DriveMode AI::desiredDriveMode() const{
	if (current_state_ == RobotState::DONE){
		return DriveMode::IDLE;
	}
	return sequenceForState(current_state_) ? DriveMode::APPLYING_SEQUENCE : DriveMode::LINE_FOLLOWING;
}

double AI::lineFollowingDirection() const{
	return current_state_ == RobotState::LINE_FOLLOWING_REVERSE ? -1.0 : 1.0;
}

double AI::targetRotationDegrees() const{
	return sequence_runner_.targetRotationDegrees();
}

bool AI::onRotationReached(){
	if (!arm_){
		return false;
	}
	return sequence_runner_.onRotationReached(*arm_);
}

RobotState AI::currentState() const{
	return current_state_;
}

int AI::visits(RobotState state) const{
	return state_visit_count_[idx(state)];
}
