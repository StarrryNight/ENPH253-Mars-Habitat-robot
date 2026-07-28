#include "ai.h"
#include "sonar.h"
#include <Arduino.h>
#include <array>

// Dummy tuning constants — placeholders pending real localization/testing,
// same status as the arm pose numbers in robot_poses.h.
namespace {
	constexpr int kNumRocks = 6;
	constexpr int kNumHabitatCycles = 2;
	// Only 2 teletubbies exist on the field — see AI::notifyTeletubbyDetected.
	constexpr int kMaxTeletubbies = 2;

	// Per-rock: distance (m) into FINDING_ROCK's Nth visit at which rock N's
	// checkpoint is reached, the one-time rotation to face the scan
	// direction, and the sweep poses to apply there. Indexed by
	// AI::visits(FINDING_ROCK) - 1 — see AI::tickFindingRock/transitionTo.
	// NOT constexpr: ArmPoseSequence holds a std::vector, a non-literal type.
	struct RockCheckpoint {
		double trigger_progress_m;
		ArmPoseSequence scan;
	};
	const std::array<RockCheckpoint, kNumRocks> kRockCheckpoints = {{
		// First entry shortened to 0.15 for bench testing the rock-finding
		// sequence (line-follow -> stop -> METAL_DETECTING -> reacquire)
		// without needing the full field run.
		{0.1111, {0.0, 	{{50, 20, Arm::WRIST_CENTER, Arm::CLAW_OPEN},{20, 50, Arm::WRIST_CENTER, Arm::CLAW_OPEN}}}},
		{0.5, {-47.0, 	{{50, 20, Arm::WRIST_CENTER, Arm::CLAW_OPEN},{20, 50, Arm::WRIST_CENTER, Arm::CLAW_OPEN}}}},
		{0.5, {48.0, 	{{50, 20, Arm::WRIST_CENTER, Arm::CLAW_OPEN},{20, 50, Arm::WRIST_CENTER, Arm::CLAW_OPEN}}}},
		{0.4, {-45.0, 	{{50, 20, Arm::WRIST_CENTER, Arm::CLAW_OPEN},{20, 50, Arm::WRIST_CENTER, Arm::CLAW_OPEN}}}},
		{0.25, {-45.0, 	{{50, 20, Arm::WRIST_CENTER, Arm::CLAW_OPEN},{20, 50, Arm::WRIST_CENTER, Arm::CLAW_OPEN}}}},
	}};

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
	// which would also print a self-transition. FINDING_ROCK has no arm
	// sequence, so there's nothing to start on the sequence runner.
	state_visit_count_[idx(current_state_)] = 1;
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

void AI::notifyTeletubbyDetected(){
	if (current_state_ == RobotState::METAL_DETECTING && visits(RobotState::TELETUBBYING) < kMaxTeletubbies){
		teletubby_detected_ = true;
	}
}

void AI::transitionTo(RobotState next){
	//Serial.printf("[AI] %s -> %s\n", robotStateName(current_state_), robotStateName(next));
	current_state_ = next;
	state_visit_count_[idx(next)]++;
	current_state_progress_m_ = 0;

	if (next == RobotState::METAL_DETECTING){
		// Per-rock scan sequence, not a fixed sequenceForState() entry —
		// visits(FINDING_ROCK) is still the checkpoint we just reached
		// (FINDING_ROCK's own visit count doesn't bump again until we
		// return to it via nextRockOrDone()).
		int rock_index = visits(RobotState::FINDING_ROCK) - 1;
		sequence_runner_.start(kRockCheckpoints[rock_index].scan);
	} else if (const ArmPoseSequence* sequence = sequenceForState(next)){
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
		case RobotState::REACQUIRING_LINE: return tickReacquiringLine();
		case RobotState::FINDING_ROCK: return tickFindingRock();
		case RobotState::METAL_DETECTING: return tickMetalDetecting();
		case RobotState::TELETUBBYING: return tickTeletubbying();
		case RobotState::PICKUP_ROCK: return tickPickupRock();
		case RobotState::LINE_FOLLOWING: return tickLineFollowing();
		case RobotState::HABITAT_PICKUP: return tickHabitatPickup();
		case RobotState::LINE_FOLLOWING_REVERSE: return tickLineFollowingReverse();
		case RobotState::HABITAT_PLACE: return tickHabitatPlace();
		case RobotState::DONE: return RobotState::DONE;
	}
	return current_state_;
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
	if (current_state_progress_m_ >= kRockCheckpoints[n - 1].trigger_progress_m){
		return RobotState::METAL_DETECTING;
	}
	return RobotState::FINDING_ROCK;
}

RobotState AI::tickMetalDetecting(){
	// Checked every tick regardless of sequence completion — the metal
	// detector flag auto-clears itself after ~1s (see MetalDetector), so a
	// hit mid-sweep would be missed if this only looked once the whole
	// (multi-second, settle-delay-heavy) scan sequence finished.
	if (metal_detector_.getMetalDetectorState()){
		return RobotState::PICKUP_ROCK;
	}
	if (teletubby_detected_){
		teletubby_detected_ = false;
		return RobotState::TELETUBBYING;
	}
	if (!sequence_runner_.complete()){
		return RobotState::METAL_DETECTING;
	}
	return nextRockOrDone();
}


RobotState AI::tickTeletubbying(){
	if (!sequence_runner_.complete()){
		return RobotState::TELETUBBYING;
	}
	// Metal may have been detected while the wave was playing — prefer
	// picking up the rock over moving on, same priority as tickMetalDetecting.
	if (metal_detector_.getMetalDetectorState()){
		return RobotState::PICKUP_ROCK;
	}
	return nextRockOrDone();
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
	if (current_state_ == RobotState::METAL_DETECTING){
		// Per-rock sequence, not a fixed sequenceForState() entry (see
		// transitionTo) — special-cased here the same way.
		return DriveMode::APPLYING_SEQUENCE;
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

uint64_t AI::sequencePoseSettleUs() const{
	return sequence_runner_.poseSettleUs();
}

bool AI::metalDetected(){
	return metal_detector_.getMetalDetectorState();
}

RobotState AI::currentState() const{
	return current_state_;
}

int AI::visits(RobotState state) const{
	return state_visit_count_[idx(state)];
}
