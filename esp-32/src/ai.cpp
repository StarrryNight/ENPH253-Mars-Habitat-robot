#include "ai.h"
#include "HWCDC.h"
#include "esp32-hal-gpio.h"
#include "pins.h"
#include "robot_state.h"
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
		{500000, -47.0},
		{0.45, -47.0},
		{0.7, 48.0},
		{0.25, -45.0},
		{0.55, -45.0},
	}};

	// Distance (m) into LINE_FOLLOWING's Nth visit at which the habitat is
	// reached. Indexed by AI::visits(LINE_FOLLOWING) - 1.
	constexpr std::array<double, kNumHabitatCycles> kHabitatApproachDistancesM = {
		1.0, 1.0,
	};

	// Distance (m) driven on the LINE_FOLLOWING_REVERSE leg between the
	// habitat pickup and place spots (in both directions).
	constexpr double kLineFollowingReverseDistanceM = 0.5;

	// Distance (m) driven straight backward on the HABITAT_BACKUP leg, to
	// clear the habitat wall HABITAT_FIND's sonar just read close-range
	// before HABITAT_PICKUP's arm sequence runs.
	constexpr double kHabitatBackupDistanceM = 0.04;

	// Distance (m) driven straight backward on the HABITAT_APPROACH_BACKUP
	// leg, before HABITAT_FIND starts its sonar-based strafe search — clears
	// the wall the robot just line-followed up to.
	constexpr double kHabitatApproachBackupDistanceM = 0.06;

	// Distance (m) driven straight backward on the HABITAT_POST_PICKUP_BACKUP
	// leg, after HABITAT_PICKUP's arm sequence completes — clears the
	// habitat before HABITAT_HOLD_AND_MOVE strafes back onto the line.
	constexpr double kHabitatPostPickupBackupDistanceM = 0.07;

	// Sonar range (cm) that counts as "found the rock" — see
	// AI::tickRotatingTilRock. Placeholder pending hardware tuning.
	constexpr double kRockSonarMinCm = 5.0;
	constexpr double kRockSonarMaxCm = 18.0;
	// Sonar-to-arm-origin mounting offset (m), ported from the mr_krabs.cpp
	// bench test that originally worked out this value on hardware.
	constexpr double kSonarMountOffsetM = 0.18054;
	// Same magnitude as LINE_SEARCH_OMEGA_RAD_S (mr_krabs.h) — kept as a
	// separate constant since rock-search and line-search are independent
	// bench-tuning knobs.
	constexpr double kRockSearchOmegaRadS = 0.9;
	// Minimum time (us) the sonar must stay continuously in range before
	// ROTATING_TIL_ROCK commits to "found the rock". Without this, a single
	// spurious close-range reading (ultrasonic noise/reflections are common
	// right at the start of a rotation) can trip tickRotatingTilRock() on
	// the very first tick after the entry settle delay — before
	// MrKrabs::driveCurrentMode() ever issues a single rotate command, so
	// the robot appears to sit still in ROTATING_TIL_ROCK.
	constexpr uint64_t kRockSonarConfirmDelayUs = 38000; // 0.025 s

	// Progress (m) into LINE_FOLLOWING at which the ramp leg begins — see
	// RobotState::RAMP_LINE_FOLLOWING. Measured from where LINE_FOLLOWING was
	// entered (transitionTo zeroes current_state_progress_m_), i.e. from wherever
	// ROCK_REACQUIRE_LINE put the robot back on the line.
	constexpr double kRampTriggerDistanceM = 0.7;

	// Sonar window (cm) that counts as "second rock ahead" during
	// RAMP_LINE_FOLLOWING. The lower bound is not just a range limit: NewPing
	// returns 0 for no echo (see Sonar::queryDistance), so a bare "closer than
	// 20" test would fire continuously whenever the sensor reads nothing at all.
	// Requiring > 8 excludes that case as well as anything implausibly close.
	constexpr double kSecondRockSonarMinCm = 8.0;
	constexpr double kSecondRockSonarMaxCm = 20.0;

	// Range (cm) SECOND_ROCK_FIND drives forward to before stopping and picking
	// up. The reading taken at that moment is what the pickup's sonar-relative
	// poses are placed from, so this is a real positioning number, not just a
	// trigger — the arm reaches to wherever the sonar last said the rock was.
	constexpr double kSecondRockApproachStopCm = 15.0;

	// Below this the sonar reading is "no echo" rather than "very close":
	// NewPing returns 0 when nothing comes back (see Sonar::queryDistance), and
	// every closing-distance test has to exclude it or it fires immediately.
	constexpr double kSonarNoEchoCm = 2.0;

	// Rocks collected before the run stops looking for more. The three are
	// handled differently: the first from a FINDING_ROCK checkpoint, the second
	// off the ramp (RAMP_LINE_FOLLOWING -> SECOND_ROCK_FIND), and the third back
	// through the ordinary checkpoint machinery. See nextRockOrDone().
	constexpr int kRocksToCollect = 3;

	// How many poses from the front of kPickupRockXYSequence belong to
	// METAL_DETECTING rather than PICKUP_ROCK. The runner is shared across both
	// states — it isn't restarted on the hand-off — so this is the split point:
	// the probe applies poses [0, kMetalProbePoseCount) and then holds, and
	// PICKUP_ROCK resumes from wherever that left the index.
	constexpr size_t kMetalProbePoseCount = 2;

	// How long (us) METAL_DETECTING holds the probe pose waiting for the metal
	// detector before deciding this rock isn't metal and moving on. Measured
	// from when the probe pose is actually applied, not from entering the state,
	// so the arm's travel time doesn't eat the window. Long enough for the
	// detector to settle over the target, short enough that a non-metal rock
	// doesn't cost the run much — every rock checkpoint pays this.
	constexpr uint64_t kMetalProbeTimeoutUs = 500000; // 2 s

	// Grace window, in control ticks, granted to the second side sensor after the
	// first one reads the habitat strip. Only if it hasn't come on by then does
	// the approach count as crooked and divert to HABITAT_SQUARE_UP; otherwise
	// this is a square arrival and the robot carries on to
	// HABITAT_APPROACH_BACKUP. Exists because "both at once" would otherwise mean
	// "within one 10 ms tick", i.e. within 1 mm of travel at FORWARD_SPEED, which
	// no real approach achieves — without a window the square-up would fire on
	// essentially every run, including dead-straight ones.
	//
	// 5 ticks = 50 ms = ~5 mm of approach at FORWARD_SPEED, so the tolerance is
	// whatever yaw offsets the two outer sensors by less than that (2 * half
	// their spacing * sin(theta) < 5 mm). Raise it to tolerate more crookedness,
	// lower to square up more eagerly.
	constexpr int kSideSensorGraceTicks = 9;

	// Longest (us) HABITAT_SQUARE_UP will rotate before giving up and proceeding
	// unsquared. Generous next to the turn it should need — at
	// SQUARE_UP_OMEGA_RAD_S a realistic misalignment is a fraction of a second —
	// so it only ever fires when the second sensor genuinely isn't going to read
	// the strip. See AI::tickHabitatSquareUp.
	constexpr uint64_t kSquareUpTimeoutUs = 1000000; // 1.5 s

	// How long (us) HABITAT_FIND keeps strafing after the sonar first reads
	// closer than HABITAT_SIDE_THRESHOLD, before handing off to HABITAT_BACKUP.
	// Pure extra travel, not a debounce: at HABITAT_STRAFE_SPEED (0.13 m/s)
	// every 10 ms here is ~1.3 mm of strafe past the point the sonar first went
	// close. Raise it to end up further along the habitat, lower to stop nearer
	// to where it was detected.
	constexpr uint64_t kHabitatSideStopDelayUs = 15000; // 0.05 s
}

AI::AI():
	arm_(nullptr),
	line_follower_(nullptr),
	// Bench-test override: start straight in HABITAT_PICKUP to loop the
	// habitat pickup/place cycle in isolation, instead of the real
	// FINDING_ROCK entry point. Flip back to RobotState::FINDING_ROCK to
	// restore the real competition flow.
	current_state_(RobotState::HABITAT_LINE_FOLLOWING),
	current_state_progress_m_(0),
	post_reacquire_state_(RobotState::HABITAT_LINE_FOLLOWING)
{
	Serial.printf("[AI] constructed, initial state: %s\n", robotStateName(current_state_));
	// Set the counter directly rather than routing through transitionTo(),
	// which would also print a self-transition. Unlike FINDING_ROCK,
	// HABITAT_PICKUP does have an arm sequence, so it must be started here
	// explicitly — transitionTo() is the only other place that happens, and
	// it's bypassed for this initial state.
	state_visit_count_[idx(current_state_)] = 1;
	xy_sequence_runner_.start(kHabitatPickupXYSequence, 0.0);
	pinMode(TELETUBBY_LED, OUTPUT);
	digitalWrite(TELETUBBY_LED, LOW);
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

void AI::setLineSearchInitialTurnDone(bool done){
	line_search_initial_turn_done_ = done;
}

bool AI::lineSearchContinuesLastRotation() const{
	return line_search_continues_last_rotation_;
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
		metal_probe_started_us_ = 0;
	} else if (next == RobotState::PICKUP_ROCK){
		// Entered only from a confirmed metal hit (tickMetalDetecting, or
		// tickTeletubbying re-checking after the wave), so this is the one place
		// that knows a rock is being collected. Counting here rather than in the
		// tick functions covers both entries with one line, and the count is what
		// nextRockOrDone() routes the run on.
		rocks_collected_++;
		Serial.printf("[AI] rock %d of %d\n", rocks_collected_, kRocksToCollect);
	} else if (next == RobotState::HABITAT_PICKUP){
		xy_sequence_runner_.start(kHabitatPickupXYSequence, 0.0);
	} else if (next == RobotState::HABITAT_PLACE){
		xy_sequence_runner_.start(kHabitatPlaceXYSequence, 0.0);
	} else if (next == RobotState::HABITAT_SQUARE_UP){
		// Cleared, not stamped: tickHabitatSquareUp() starts the bail-out clock
		// on its own first tick instead. Stamping here would run the clock
		// through MrKrabs::startSquaringUp()'s entry settle delay — during which
		// stepControl() returns before tickAI() and the wheels are deliberately
		// held stopped — so a timeout shorter than that settle would expire
		// before the rotation was ever commanded. The rotation direction itself is
		// latched by tickHabitatLineFollowing() before it returns this state,
		// while the sensor readings that determine it are still valid.
		square_up_entered_us_ = 0;
	} else if (next == RobotState::REVERSE_180){
		// One-shot pose rather than a runner: REVERSE_180's DriveMode drives the
		// spin, so nothing would step a sequence through. Written here, at the
		// transition, so it lands before MrKrabs::startReverse180() starts its
		// settle delay and the turn begins. See kReverse180ArmPose.
		if (arm_){
			arm_->setPose(kReverse180ArmPose);
		}
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
		case RobotState::ROCK_REACQUIRE_LINE: return tickRockReacquireLine();
		case RobotState::REVERSE_180: return tickReverse180();
		case RobotState::FINDING_ROCK: return tickFindingRock();
		case RobotState::ROTATING_TIL_ROCK: return tickRotatingTilRock();
		case RobotState::METAL_DETECTING: return tickMetalDetecting();
		case RobotState::TELETUBBYING: return tickTeletubbying();
		case RobotState::PICKUP_ROCK: return tickPickupRock();
		case RobotState::LINE_FOLLOWING: return tickLineFollowing();
		case RobotState::RAMP_LINE_FOLLOWING: return tickRampLineFollowing();
		case RobotState::SECOND_ROCK_FIND: return tickSecondRockFind();
		case RobotState::HABITAT_LINE_FOLLOWING: return tickHabitatLineFollowing();
		case RobotState::HABITAT_PICKUP: return tickHabitatPickup();
		case RobotState::LINE_FOLLOWING_REVERSE: return tickLineFollowingReverse();
		case RobotState::HABITAT_PLACE: return tickHabitatPlace();
		case RobotState::HABITAT_SQUARE_UP: return tickHabitatSquareUp();
		case RobotState::HABITAT_APPROACH_BACKUP: return tickHabitatApproachBackup();
		case RobotState::HABITAT_FIND: return tickHabitatFind();
		case RobotState::HABITAT_BACKUP: return tickHabitatBackup();
		case RobotState::HABITAT_POST_PICKUP_BACKUP: return tickHabitatPostPickupBackup();
		case RobotState::HABITAT_HOLD_AND_MOVE: return tickHabitatHoldAndMove();
		case RobotState::DONE: return RobotState::DONE;
	}
	return current_state_;
}

RobotState AI::tickReacquiringLine(){
	// Ignore the line sensors until the fixed initial turn (see
	// MrKrabs::startSearchingForLine/driveCurrentMode) has actually
	// completed — otherwise a state that left the robot already sitting on
	// the line (HABITAT_PICKUP/HABITAT_PLACE, which don't rotate) could see
	// bothMidSensorsOnLine() read true a few degrees in, cutting the turn
	// short before it ever reaches the intended amount.
	if (!line_search_initial_turn_done_){
		return RobotState::REACQUIRING_LINE;
	}
	if (!line_follower_ || !line_follower_->bothMidSensorsOnLine()){
		return RobotState::REACQUIRING_LINE;
	}
	return post_reacquire_state_;
}

RobotState AI::tickReverse180(){
	// Same two-phase completion as tickReacquiringLine — hold off on the
	// line sensors until the fixed initial turn has cleared the line, then
	// spin until they reacquire it. The forced-clockwise direction and the
	// smaller initial angle are set by MrKrabs::startReverse180().
	if (!line_search_initial_turn_done_){
		return RobotState::REVERSE_180;
	}
	if (!line_follower_ || !line_follower_->bothMidSensorsOnLine()){
		return RobotState::REVERSE_180;
	}
	return post_reacquire_state_;
}

RobotState AI::nextRockOrDone(){
	// Where the run goes after a rock is resolved depends on how many have
	// actually been collected, because the three are reached differently:
	//
	//   0 — nothing collected yet (this rock was skipped for lack of metal):
	//       carry on through the checkpoint table as before.
	//   1 — first rock done: LINE_FOLLOWING, whose distance trigger starts the
	//       ramp leg that finds the second.
	//   2 — second (ramp) rock done: back to FINDING_ROCK, so the third is
	//       handled by the ordinary checkpoint machinery.
	//  3+ — done collecting; hold on the line.
	//
	// LINE_FOLLOWING rather than HABITAT_LINE_FOLLOWING throughout: the habitat
	// states are deliberately not entered yet.
	if (rocks_collected_ >= kRocksToCollect){
		post_reacquire_state_ = RobotState::LINE_FOLLOWING;
	} else if (rocks_collected_ == 1){
		post_reacquire_state_ = RobotState::LINE_FOLLOWING;
	} else if (rocks_collected_ == 2){
		post_reacquire_state_ = RobotState::FINDING_ROCK;
	} else {
		post_reacquire_state_ = visits(RobotState::FINDING_ROCK) >= kNumRocks
			? RobotState::LINE_FOLLOWING
			: RobotState::FINDING_ROCK;
	}
	// Rock path: retrace the search spin, no blind initial turn.
	line_search_continues_last_rotation_ = false;
	return RobotState::ROCK_REACQUIRE_LINE;
}

RobotState AI::tickRockReacquireLine(){
	// No line_search_initial_turn_done_ gate, unlike tickReacquiringLine: the
	// rock search already rotated the robot off the line, so there is no line
	// under the sensors to clear first and nothing to protect against. Watch from
	// the first tick and stop the moment the line comes back, which keeps the
	// overshoot past it to whatever one control tick of rotation covers.
	if (!line_follower_ || !line_follower_->bothMidSensorsOnLine()){
		return RobotState::ROCK_REACQUIRE_LINE;
	}
	return post_reacquire_state_;
}

RobotState AI::tickFindingRock(){
	// Quota filled: stop stopping for checkpoints and just keep line-following
	// (same speed — both states drive DriveMode::LINE_FOLLOWING at
	// FORWARD_SPEED). Note this is a count now, not a "have we got one" flag:
	// the third rock is reached by coming back into this state after the ramp
	// rock, so an early return on the first pickup would make it unreachable.
	if (rocks_collected_ >= kRocksToCollect){
		return RobotState::LINE_FOLLOWING;
	}
	int n = visits(RobotState::FINDING_ROCK); // 1-based
	if (n > kNumRocks){
		// Out of checkpoints — same destination as a successful pickup while the
		// habitat states are held back (see nextRockOrDone).
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
	// Teletubby takes priority over the sensor poll below: it's an explicit RPi
	// report rather than something we're measuring, and it's only valid while
	// we're sat here at the rock.
	if (teletubby_detected_){
		teletubby_detected_ = false;
		return RobotState::TELETUBBYING;
	}
	// Nothing to read until the probe pose is actually on the rock — the arm is
	// still travelling out until onRotationReached() writes it (see
	// metal_probe_pose_applied_).
	if (!metal_probe_pose_applied_){
		return RobotState::METAL_DETECTING;
	}
	uint64_t now = esp_timer_get_time();
	if (metal_probe_started_us_ == 0){
		// Probe pose just landed. Start the window here, and drop any latched
		// detection so the verdict comes from this rock rather than from a hit
		// still held over from the last one (the latch runs 5 s — see
		// MetalDetector::clear).
		metal_probe_started_us_ = now;
		metal_detector_.clear();
		return RobotState::METAL_DETECTING;
	}
	if (metal_detector_.getMetalDetectorState()){
		Serial.printf("[AI] metal detected after %.0f ms, picking up\n",
			(now - metal_probe_started_us_) / 1000.0);
		digitalWrite(TELETUBBY_LED, HIGH);
		return RobotState::PICKUP_ROCK;
	}
	if (now - metal_probe_started_us_ >= kMetalProbeTimeoutUs){
		// Not metal. Leave the rock where it is and carry on to the next
		// checkpoint. The arm is still extended in the probe pose and the
		// pickup sequence is abandoned mid-run, so retract before the
		// REACQUIRING_LINE spin that nextRockOrDone() hands off to.
		Serial.printf("[AI] no metal after %d ms, skipping this rock\n",
			static_cast<int>(kMetalProbeTimeoutUs / 1000));
		if (arm_){
			arm_->setPose(kArmHomePose);
		}
		return nextRockOrDone();
	}
	return RobotState::METAL_DETECTING;
}

RobotState AI::tickTeletubbying(){
	if (!sequence_runner_.complete()){
		return RobotState::TELETUBBYING;
	}
	// Metal may have been detected while the wave was playing — prefer
	// picking up the rock over moving on, same priority as tickMetalDetecting.
	digitalWrite(TELETUBBY_LED, LOW);
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
	// Drives the line at FORWARD_SPEED (desiredDriveMode() falls through to
	// DriveMode::LINE_FOLLOWING — no sequenceForState entry) until enough
	// distance has passed to be at the ramp, then hands over to the leg that
	// watches the sonar. Nothing else ends this state.
	if (current_state_progress_m_ >= kRampTriggerDistanceM){
		Serial.printf("[AI] %.2f m of line-following done, entering ramp leg\n",
			current_state_progress_m_);
		return RobotState::RAMP_LINE_FOLLOWING;
	}
	return RobotState::LINE_FOLLOWING;
}

RobotState AI::tickRampLineFollowing(){
	// Same driving as LINE_FOLLOWING; the only difference is that this leg
	// watches ahead. Stops as soon as the sonar reads inside the second-rock
	// window — the stop itself happens through the state change, since
	// SECOND_ROCK_FIND's drive mode holds the wheels.
	if (!sonar_){
		return RobotState::RAMP_LINE_FOLLOWING;
	}
	double d = sonar_->queryDistance();
	if (d <= kSecondRockSonarMinCm || d >= kSecondRockSonarMaxCm){
		// Out of the window (0 = no echo lands here too) — reset any part-served
		// confirmation so a fresh one has to accumulate, same as the rock search.
		ramp_sonar_in_range_since_us_ = 0;
		return RobotState::RAMP_LINE_FOLLOWING;
	}
	uint64_t now = esp_timer_get_time();
	if (ramp_sonar_in_range_since_us_ == 0){
		ramp_sonar_in_range_since_us_ = now;
		return RobotState::RAMP_LINE_FOLLOWING;
	}
	if (now - ramp_sonar_in_range_since_us_ < kRockSonarConfirmDelayUs){
		return RobotState::RAMP_LINE_FOLLOWING;
	}
	Serial.printf("[AI] second rock at %.1f cm, stopping\n", d);
	ramp_sonar_in_range_since_us_ = 0;
	return RobotState::SECOND_ROCK_FIND;
}

RobotState AI::tickSecondRockFind(){
	// Keeps line-following straight at the rock the ramp leg spotted, closing
	// until the sonar reads kSecondRockApproachStopCm, then picks up from there.
	// No rotation involved, unlike the checkpoint rocks — the ramp leg already
	// left the robot pointed at it, so this is purely an approach.
	if (!sonar_){
		return RobotState::SECOND_ROCK_FIND;
	}
	double d = sonar_->queryDistance();
	if (d <= kSonarNoEchoCm || d > kSecondRockApproachStopCm){
		// Still closing (or momentarily no echo — driving on is the right
		// response either way, since the rock was seen a moment ago).
		return RobotState::SECOND_ROCK_FIND;
	}
	// Close enough. Cache this reading as the origin for the pickup sequence's
	// sonar-relative poses, exactly as tickRotatingTilRock does for a checkpoint
	// rock, so the arm reaches to where the rock actually is rather than to a
	// fixed distance.
	cached_rock_sonar_x_m_ = d / 100.0 + kSonarMountOffsetM;
	Serial.printf("[AI] second rock reached at %.1f cm, picking up\n", d);
	return RobotState::METAL_DETECTING;
}

RobotState AI::tickHabitatLineFollowing(){
	if (!line_follower_){
		return RobotState::HABITAT_LINE_FOLLOWING;
	}
	// Both on the strip — square enough, whether they arrived together or the
	// second one caught up inside the grace window below. The tick count says
	// which: 0 means they landed on the same tick, 1-4 means the window opened
	// and the trailing sensor arrived inside it.
	if (line_follower_->bothSideSensorsOnLine()){
		Serial.printf("[AI] habitat strip square, no square-up needed (both sides on after %d/%d grace ticks)\n",
			side_sensor_grace_ticks_, kSideSensorGraceTicks);
		side_sensor_grace_ticks_ = 0;
		return RobotState::HABITAT_APPROACH_BACKUP;
	}
	bool left = line_follower_->leftSensorOnLine();
	bool right = line_follower_->rightSensorOnLine();
	if (left != right){
		// One side has reached the strip. Keep driving for kSideSensorGraceTicks
		// to let the other one arrive; only a window that expires one-sided means
		// the approach is genuinely crooked.
		if (side_sensor_grace_ticks_ == 0){
			// Latch the direction from the sensor that got there FIRST, on the
			// tick the window opens — that identifies the leading side, which is
			// the geometry the correction depends on. Reading it later risks the
			// two sensors having swapped over an edge mid-window.
			//
			// Rotating counter-clockwise swings the right side of the array
			// forward (a sensor at (+d, L) moves to y = L*cos + d*sin), so the
			// side that arrived first was already leading and the correction has
			// to bring the *other* one forward: left first -> CCW (+1), right
			// first -> CW (-1).
			square_up_omega_sign_ = left ? 1.0 : -1.0;
		}
		if (++side_sensor_grace_ticks_ < kSideSensorGraceTicks){
			return RobotState::HABITAT_LINE_FOLLOWING;
		}
		Serial.printf("[AI] habitat strip crooked, square-up TRIGGERED (%s side first, other still off after %d ticks, rotating %s)\n",
			square_up_omega_sign_ > 0 ? "left" : "right", kSideSensorGraceTicks,
			square_up_omega_sign_ > 0 ? "CCW" : "CW");
		side_sensor_grace_ticks_ = 0;
		return RobotState::HABITAT_SQUARE_UP;
	}
	// Neither sensor on the strip: whatever one of them clipped wasn't the
	// marker, so drop the window rather than letting a later reading resume a
	// half-elapsed count.
	side_sensor_grace_ticks_ = 0;
	return RobotState::HABITAT_LINE_FOLLOWING;
}

RobotState AI::tickHabitatSquareUp(){
	uint64_t now = esp_timer_get_time();
	// First tick of the state (see transitionTo): start the bail-out clock here,
	// so it measures time actually spent rotating rather than including the entry
	// settle delay that precedes any rotation.
	if (square_up_entered_us_ == 0){
		square_up_entered_us_ = now;
		return RobotState::HABITAT_SQUARE_UP;
	}
	// Done the moment the trailing sensor catches up — both on the strip means
	// the array is parallel to it.
	if (line_follower_ && line_follower_->bothSideSensorsOnLine()){
		Serial.printf("[AI] square-up complete after %.0f ms of rotation\n",
			(now - square_up_entered_us_) / 1000.0);
		return RobotState::HABITAT_APPROACH_BACKUP;
	}
	// Bail-out: a strip the second sensor never reads (missed edge, marginal
	// reflectivity, a sensor that just clipped a corner) would otherwise leave
	// the robot rotating in place indefinitely at the habitat. Give up and carry
	// on unsquared rather than hang — the habitat legs cope worse with a bad
	// heading than with a slightly crooked one, but they cope.
	if (esp_timer_get_time() - square_up_entered_us_ >= kSquareUpTimeoutUs){
		Serial.println("[AI] HABITAT_SQUARE_UP timed out, proceeding unsquared");
		return RobotState::HABITAT_APPROACH_BACKUP;
	}
	return RobotState::HABITAT_SQUARE_UP;
}

RobotState AI::tickHabitatApproachBackup(){
	return current_state_progress_m_ >= kHabitatApproachBackupDistanceM
		? RobotState::HABITAT_FIND
		: RobotState::HABITAT_APPROACH_BACKUP;
}

RobotState AI::tickHabitatFind(){
	if (!sonar_){
		return RobotState::HABITAT_FIND;
	}
	// Strafe until the sonar first reads close (HABITAT_SIDE_THRESHOLD), then
	// keep going for kHabitatSideStopDelayUs before handing off to
	// HABITAT_BACKUP — the extra travel carries the robot past the edge it just
	// detected. The countdown is latched, not re-armed each tick: once the edge
	// has been seen it runs to completion regardless of what the sonar reads
	// next, so the long readings that come back while passing the slot itself
	// can't cancel it.
	if (habitat_side_seen_us_ == 0){
		if (sonar_->queryDistance() >= HABITAT_SIDE_THRESHOLD){
			return RobotState::HABITAT_FIND;
		}
		habitat_side_seen_us_ = esp_timer_get_time();
		return RobotState::HABITAT_FIND;
	}
	if (esp_timer_get_time() - habitat_side_seen_us_ < kHabitatSideStopDelayUs){
		return RobotState::HABITAT_FIND;
	}
	habitat_side_seen_us_ = 0;
	habitat_found_num_+=1;
	if(habitat_found_num_==2){
		current_habitat_find_direction_=-1;
	}
	return RobotState::HABITAT_BACKUP;
}

RobotState AI::tickHabitatBackup(){
	return current_state_progress_m_ >= kHabitatBackupDistanceM
		? RobotState::HABITAT_PICKUP
		: RobotState::HABITAT_BACKUP;
}

RobotState AI::tickHabitatPickup(){
	if (!xy_sequence_runner_.complete()){
		return RobotState::HABITAT_PICKUP;
	}
	post_line_reverse_state_ = RobotState::HABITAT_PLACE;
	return RobotState::HABITAT_POST_PICKUP_BACKUP;
}

RobotState AI::tickHabitatPostPickupBackup(){
	return current_state_progress_m_ >= kHabitatPostPickupBackupDistanceM
		? RobotState::HABITAT_HOLD_AND_MOVE
		: RobotState::HABITAT_POST_PICKUP_BACKUP;
}

RobotState AI::tickHabitatHoldAndMove(){
	if (!line_follower_ || !line_follower_->bothMidSensorsOnLine()){
		return RobotState::HABITAT_HOLD_AND_MOVE;
	}
	// Back on the line, but still facing the habitat we just picked up from —
	// reverse ~180° before driving the LINE_FOLLOWING_REVERSE leg toward the
	// place spot.
	post_reacquire_state_ = RobotState::LINE_FOLLOWING_REVERSE;
	return RobotState::REVERSE_180;
}

RobotState AI::tickLineFollowingReverse(){
	// The post-pickup leg (heading to HABITAT_PLACE) drives until the right
	// sensor crosses the habitat place marker, then hands straight to
	// HABITAT_PLACE — whose sequence rotates by its rotation_degrees before any
	// pose is applied (see kHabitatPlaceXYSequence and MrKrabs::
	// driveCurrentMode's APPLYING_SEQUENCE case), which is what squares the
	// robot up to the slot. That fixed turn replaces the REACQUIRING_LINE
	// (45° + reactive spin) leg this used to route through: the marker is a
	// known geometry, so a known angle beats hunting for the line again.
	if (post_line_reverse_state_ == RobotState::HABITAT_PLACE){
		if (!line_follower_ || !line_follower_->rightSensorOnLine()){
			return RobotState::LINE_FOLLOWING_REVERSE;
		}
		return RobotState::HABITAT_PLACE;
	}
	return current_state_progress_m_ >= kLineFollowingReverseDistanceM
		? post_line_reverse_state_
		: RobotState::LINE_FOLLOWING_REVERSE;
}

RobotState AI::tickHabitatPlace(){
	if (!xy_sequence_runner_.complete()){
		return RobotState::HABITAT_PLACE;
	}
	// Rock released. Sweep back to the line and line-follow the habitat approach
	// again — which re-runs the whole find sequence (square-up, approach backup,
	// the sonar strafe) rather than assuming the next slot is where this one
	// was. That strafe is also what flips current_habitat_find_direction_ on the
	// second slot, so the cycle has to pass back through HABITAT_FIND for the
	// two-slot behaviour to happen at all — see tickHabitatFind.
	post_reacquire_state_ = RobotState::HABITAT_LINE_FOLLOWING;
	// Keep turning the way the place rotation already went, rather than
	// reversing it as a search normally would. The place turn
	// (kHabitatPlaceXYSequence::rotation_degrees) swung the robot off the line
	// to face the slot; continuing that way carries it the long way round to
	// meet the line again, which is the approach the next pickup wants.
	line_search_continues_last_rotation_ = true;
	return RobotState::REACQUIRING_LINE;
}

AI::DriveMode AI::desiredDriveMode() const{
	if (current_state_ == RobotState::DONE){
		return DriveMode::IDLE;
	}
	if (current_state_ == RobotState::REACQUIRING_LINE){
		return DriveMode::SEARCHING_FOR_LINE;
	}
	if (current_state_ == RobotState::ROCK_REACQUIRE_LINE){
		return DriveMode::ROCK_SEARCHING_FOR_LINE;
	}
	if (current_state_ == RobotState::REVERSE_180){
		return DriveMode::REVERSE_180;
	}
	if (current_state_ == RobotState::ROTATING_TIL_ROCK){
		return DriveMode::ROTATING_TIL_ROCK;
	}
	if (current_state_ == RobotState::HABITAT_FIND){
		return DriveMode::STRAFING_TIL_HABITAT;
	}
	if (current_state_ == RobotState::HABITAT_SQUARE_UP){
		return DriveMode::SQUARING_UP;
	}
	if (current_state_ == RobotState::HABITAT_BACKUP || current_state_ == RobotState::HABITAT_APPROACH_BACKUP ||
	    current_state_ == RobotState::HABITAT_POST_PICKUP_BACKUP){
		return DriveMode::BACKING_UP;
	}
	if (current_state_ == RobotState::HABITAT_HOLD_AND_MOVE){
		return DriveMode::HOLDING_AND_MOVING;
	}
	if (current_state_ == RobotState::METAL_DETECTING || current_state_ == RobotState::PICKUP_ROCK ||
	    current_state_ == RobotState::HABITAT_PICKUP || current_state_ == RobotState::HABITAT_PLACE){
		// Driven by xy_sequence_runner_, not a fixed sequenceForState() entry
		// (see transitionTo) — special-cased here the same way.
		return DriveMode::APPLYING_SEQUENCE;
	}
	return sequenceForState(current_state_) ? DriveMode::APPLYING_SEQUENCE : DriveMode::LINE_FOLLOWING;
}

double AI::lineFollowingSpeed() const{
	// The habitat legs both end on a sensor event whose position everything
	// downstream is measured from — the marker strip for HABITAT_LINE_FOLLOWING,
	// the place marker for LINE_FOLLOWING_REVERSE — so they run slow to keep the
	// stopping point tight. The rock-phase legs have no such constraint.
	if (current_state_ == RobotState::HABITAT_LINE_FOLLOWING ||
	    current_state_ == RobotState::LINE_FOLLOWING_REVERSE){
		return HABITAT_FORWARD_SPEED;
	}
	return FORWARD_SPEED;
}

double AI::lineFollowingDirection() const{
	// Always forward — the photoresistor array is front-mounted, so it can't
	// lead while driving in reverse. LINE_FOLLOWING_REVERSE keeps its name
	// (still a distinct RobotState/leg in the state machine) but no longer
	// actually reverses.
	return 1.0;
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
		return 0.0; // the rock xy runner's poses never change heading
	}
	if (current_state_ == RobotState::HABITAT_PICKUP || current_state_ == RobotState::HABITAT_PLACE){
		return xy_sequence_runner_.targetRotationDegrees();
	}
	return sequence_runner_.targetRotationDegrees();
}

bool AI::onRotationReached(){
	if (!arm_){
		return false;
	}
	switch (current_state_){
		case RobotState::METAL_DETECTING: {
			if (metal_probe_pose_applied_){
				// Probe poses all applied — hold here until tickMetalDetecting()
				// reaches a verdict and moves the state on. Without this the
				// runner would keep advancing into PICKUP_ROCK's poses every time
				// a settle window elapsed, grabbing before anything was decided.
				return false;
			}
			bool wrote = xy_sequence_runner_.onRotationReached(*arm_);
			// Flag flips only once the whole probe group has gone out, so the
			// detector window in tickMetalDetecting() starts after the LAST probe
			// pose rather than the first.
			metal_probe_pose_applied_ = xy_sequence_runner_.currentIndex() >= kMetalProbePoseCount;
			return wrote;
		}
		case RobotState::PICKUP_ROCK:
		case RobotState::HABITAT_PICKUP:
		case RobotState::HABITAT_PLACE:
			return xy_sequence_runner_.onRotationReached(*arm_);
		default:
			return sequence_runner_.onRotationReached(*arm_);
	}
}

uint64_t AI::sequencePoseSettleUs() const{
	if (current_state_ == RobotState::METAL_DETECTING || current_state_ == RobotState::PICKUP_ROCK ||
	    current_state_ == RobotState::HABITAT_PICKUP || current_state_ == RobotState::HABITAT_PLACE){
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

int AI::habitatFindDirection() const{
	return current_habitat_find_direction_;
}

double AI::squareUpOmegaSign() const{
	return square_up_omega_sign_;
}
