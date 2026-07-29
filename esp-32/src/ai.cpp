#include "ai.h"
#include "HWCDC.h"
#include "sonar.h"
#include <Arduino.h>
#include "esp_timer.h"
#include <array>

// Dummy tuning constants — placeholders pending real localization/testing,
// same status as the arm pose numbers in robot_poses.h.
namespace {
	constexpr int kNumRocks = 6;
	constexpr int kNumHabitatCycles = 2;
	// Only 2 teletubbies exist on the field — see AI::notifyTeletubbyDetected.
	constexpr int kMaxTeletubbies = 2;

	// Per-rock: distance (m) into FINDING_ROCK's Nth visit at which rock N's
	// checkpoint is reached, and the search direction (sign only) for
	// ROTATING_TIL_ROCK — reusing the same sign convention startRotation()
	// derives last_rotation_sign_ from. Indexed by AI::visits(FINDING_ROCK) -
	// 1 — see AI::tickFindingRock/rockSearchOmegaRadS.
	struct RockCheckpoint {
		double trigger_progress_m;
		double rotation_degrees;
	};
	constexpr std::array<RockCheckpoint, kNumRocks> kRockCheckpoints = {{
		{0.0, 1.0},
		{0.5, -47.0},
		{0.5, 48.0},
		{0.4, -45.0},
		{0.25, -45.0},
	}};

	// Distance (m) into LINE_FOLLOWING's Nth visit at which the habitat is
	// reached. Indexed by AI::visits(LINE_FOLLOWING) - 1.
	constexpr std::array<double, kNumHabitatCycles> kHabitatApproachDistancesM = {
		1.0, 1.0,
	};

	// Distance (m) to drive in reverse before HABITAT_PLACE.
	constexpr double kLineFollowingReverseDistanceM = 1.0;

	// Sonar range (cm) that counts as "found the rock" — see
	// AI::tickRotatingTilRock. Placeholder pending hardware tuning.
	constexpr double kRockSonarMinCm = 7.0;
	constexpr double kRockSonarMaxCm = 20.0;
	// Sonar-to-arm-origin mounting offset (m), ported from the mr_krabs.cpp
	// bench test that originally worked out this value on hardware.
	constexpr double kSonarMountOffsetM = 0.18054;
	// Same magnitude as LINE_SEARCH_OMEGA_RAD_S (mr_krabs.h) — kept as a
	// separate constant since rock-search and line-search are independent
	// bench-tuning knobs.
	constexpr double kRockSearchOmegaRadS = 1.2;
	// Minimum time (us) the sonar must stay continuously in range before
	// ROTATING_TIL_ROCK commits to "found the rock". Without this, a single
	// spurious close-range reading (ultrasonic noise/reflections are common
	// right at the start of a rotation) can trip tickRotatingTilRock() on
	// the very first tick after the entry settle delay — before
	// MrKrabs::driveCurrentMode() ever issues a single rotate command, so
	// the robot appears to sit still in ROTATING_TIL_ROCK.
	constexpr uint64_t kRockSonarConfirmDelayUs = 30000; // 0.1 s
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

void AI::setSonar(Sonar* sonar){
	sonar_ = sonar;
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
	Serial.printf("[AI] %s -> %s\n", robotStateName(current_state_), robotStateName(next));
	current_state_ = next;
	state_visit_count_[idx(next)]++;
	current_state_progress_m_ = 0;

	if (next == RobotState::METAL_DETECTING){
		// Starts the one XY runner that spans METAL_DETECTING's probe pose
		// (index 0) and PICKUP_ROCK's retract/grab poses (indices 1-6) —
		// PICKUP_ROCK doesn't restart this; it just keeps advancing from
		// wherever this left off.
		xy_sequence_runner_.start(kPickupRockXYSequence, cached_rock_sonar_x_m_);
		metal_probe_pose_applied_ = false;
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
		case RobotState::ROTATING_TIL_ROCK: return tickRotatingTilRock();
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
		return RobotState::ROTATING_TIL_ROCK;
	}
	return RobotState::FINDING_ROCK;
}

RobotState AI::tickRotatingTilRock(){
	if (!sonar_){
		return RobotState::ROTATING_TIL_ROCK;
	}
	double d = sonar_->queryDistance();
	if (d < kRockSonarMinCm || d > kRockSonarMaxCm){
		// Out of range: forget any in-progress confirmation window so
		// re-entering range later starts a fresh 0.1s wait instead of
		// reusing a stale timestamp from earlier in the rotation.
		rock_sonar_in_range_since_us_ = 0;
		return RobotState::ROTATING_TIL_ROCK;
	}
	uint64_t now = esp_timer_get_time();
	if (rock_sonar_in_range_since_us_ == 0){
		// Just entered range this tick — start the confirmation window.
		rock_sonar_in_range_since_us_ = now;
		return RobotState::ROTATING_TIL_ROCK;
	}
	if (now - rock_sonar_in_range_since_us_ < kRockSonarConfirmDelayUs){
		return RobotState::ROTATING_TIL_ROCK;
	}
	cached_rock_sonar_x_m_ = d / 100.0 + kSonarMountOffsetM;
	rock_sonar_in_range_since_us_ = 0;
	return RobotState::METAL_DETECTING;
}

RobotState AI::tickMetalDetecting(){
	// Bench-test override: go straight to PICKUP_ROCK regardless of the
	// metal detector reading, so the grab sequence can be exercised without
	// a working/present metal hit. Teletubby still takes priority since it's
	// an explicit RPi report, not a sensor poll.
	if (teletubby_detected_){
		teletubby_detected_ = false;
		return RobotState::TELETUBBYING;
	}
	return RobotState::PICKUP_ROCK;
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
	if (!xy_sequence_runner_.complete()){
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
	if (current_state_ == RobotState::ROTATING_TIL_ROCK){
		return DriveMode::ROTATING_TIL_ROCK;
	}
	if (current_state_ == RobotState::METAL_DETECTING || current_state_ == RobotState::PICKUP_ROCK){
		// Driven by xy_sequence_runner_, not a fixed sequenceForState() entry
		// (see transitionTo) — special-cased here the same way.
		return DriveMode::APPLYING_SEQUENCE;
	}
	return sequenceForState(current_state_) ? DriveMode::APPLYING_SEQUENCE : DriveMode::LINE_FOLLOWING;
}

double AI::lineFollowingDirection() const{
	return current_state_ == RobotState::LINE_FOLLOWING_REVERSE ? -1.0 : 1.0;
}

double AI::rockSearchOmegaRadS() const{
	int n = visits(RobotState::FINDING_ROCK); // 1-based, same index tickFindingRock uses
	double heading = (n >= 1 && n <= static_cast<int>(kRockCheckpoints.size()))
		? kRockCheckpoints[n - 1].rotation_degrees
		: 0.0;
	return kRockSearchOmegaRadS * (heading >= 0.0 ? 1.0 : -1.0);
}

double AI::targetRotationDegrees() const{
	if (current_state_ == RobotState::METAL_DETECTING || current_state_ == RobotState::PICKUP_ROCK){
		return 0.0; // xy_sequence_runner_'s poses never change heading
	}
	return sequence_runner_.targetRotationDegrees();
}

bool AI::onRotationReached(){
	if (!arm_){
		return false;
	}
	switch (current_state_){
		case RobotState::METAL_DETECTING:
			if (metal_probe_pose_applied_){
				// Hold the probe pose until tickMetalDetecting() sees a hit
				// and moves the state on — don't auto-advance to the next
				// (retract) pose just because a settle window elapsed.
				return false;
			}
			metal_probe_pose_applied_ = xy_sequence_runner_.onRotationReached(*arm_);
			return metal_probe_pose_applied_;
		case RobotState::PICKUP_ROCK:
			return xy_sequence_runner_.onRotationReached(*arm_);
		default:
			return sequence_runner_.onRotationReached(*arm_);
	}
}

uint64_t AI::sequencePoseSettleUs() const{
	if (current_state_ == RobotState::METAL_DETECTING || current_state_ == RobotState::PICKUP_ROCK){
		return xy_sequence_runner_.poseSettleUs();
	}
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
