#include "ai.h"
#include <Arduino.h>
#include <array>

// Dummy tuning constants — placeholders pending real localization/testing,
// same status as the arm pose numbers in robot_poses.h.
namespace {
	constexpr int kNumRocks = 6;
	constexpr int kNumHabitatCycles = 2;

	// Distance (m) into FINDING_ROCK's Nth visit at which rock N's checkpoint
	// is reached. Indexed by AI::visits(FINDING_ROCK) - 1. First entry
	// shortened to 0.15 for bench testing the rock-finding sequence
	// (line-follow -> stop -> METAL_DETECTING -> reacquire) without needing
	// the full field run.
	constexpr std::array<double, kNumRocks> kRockCheckpointsM = {
		0.15, 0.6, 0.9, 1.2, 1.5, 1.8,
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
	line_follower_(nullptr),
	current_state_(RobotState::FINDING_ROCK),
	current_state_progress_m_(0),
	post_reacquire_state_(RobotState::LINE_FOLLOWING)
{
	Serial.printf("[AI] constructed, initial state: %s\n", robotStateName(current_state_));
	// Set the counter directly rather than routing through transitionTo(),
	// which would also print a self-transition.
	state_visit_count_[idx(current_state_)] = 1;
	// FINDING_ROCK has no arm sequence (it's line-following, tracked by
	// current_state_progress_m_ against kRockCheckpointsM) — only start the
	// sequence runner for states that actually have one, unlike the
	// sequence-driven states (METAL_DETECTING, etc.) which get theirs loaded
	// via transitionTo() when entered normally.
	if (const RobotSequence* initial_sequence = sequenceForState(current_state_)) {
		sequence_runner_.start(*initial_sequence);
	}
}

void AI::setArm(Arm* arm){
	arm_ = arm;
}

void AI::setLineFollower(LineFollower* line_follower){
	line_follower_ = line_follower;
}

void AI::addProgress(double delta_m){
	current_state_progress_m_ += delta_m;
}

void AI::transitionTo(RobotState next){
	Serial.printf("[AI] %s -> %s\n", robotStateName(current_state_), robotStateName(next));
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
		case RobotState::REACQUIRING_LINE: return tickReacquiringLine();
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
	// off to REACQUIRING_LINE once complete instead of falling through:
	// SequenceRunner::targetRotationDegrees() reverts to 0.0 once complete(),
	// which handleDriveTransition() would otherwise read as a fresh target
	// and rotate back to heading 0.
	if (!sequence_runner_.complete()){
		return RobotState::TEST_ROTATION;
	}
	post_reacquire_state_ = RobotState::LINE_FOLLOWING;
	return RobotState::REACQUIRING_LINE;
}

RobotState AI::tickReacquiringLine(){
	if (!line_follower_ || !line_follower_->bothMidSensorsOnLine()){
		return RobotState::REACQUIRING_LINE;
	}
	return post_reacquire_state_;
}

RobotState AI::nextRockOrDone(){
	post_reacquire_state_ = visits(RobotState::FINDING_ROCK) >= kNumRocks
		? RobotState::LINE_FOLLOWING
		: RobotState::FINDING_ROCK;
	return RobotState::REACQUIRING_LINE;
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
	// Hardware bring-up: stay in LINE_FOLLOWING indefinitely instead of
	// handing off to HABITAT_PICKUP, so line-following can be tested in
	// isolation without the rest of the state machine kicking in.
	return RobotState::LINE_FOLLOWING;
}

RobotState AI::tickHabitatPickup(){
	if (!sequence_runner_.complete()){
		return RobotState::HABITAT_PICKUP;
	}
	post_reacquire_state_ = RobotState::LINE_FOLLOWING_REVERSE;
	return RobotState::REACQUIRING_LINE;
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
	if (visits(RobotState::HABITAT_PLACE) >= kNumHabitatCycles){
		return RobotState::DONE; // finished — no need to reacquire, robot goes IDLE
	}
	post_reacquire_state_ = RobotState::LINE_FOLLOWING;
	return RobotState::REACQUIRING_LINE;
}

AI::DriveMode AI::desiredDriveMode() const{
	if (current_state_ == RobotState::DONE){
		return DriveMode::IDLE;
	}
	if (current_state_ == RobotState::REACQUIRING_LINE){
		return DriveMode::SEARCHING_FOR_LINE;
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
