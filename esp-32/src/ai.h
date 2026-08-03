#pragma once
#include <array>
#include <cstddef>
#include "arm.h"
#include "line_follower.h"
#include "robot_state.h"
#include "robot_poses.h"
#include "sequence_runner.h"
#include "metal_detector.h"
#include "sonar.h"

// Drives the robot behavior state machine described in CLAUDE.md (rocks A-C,
// habitat D). Each RobotState either drives forward with line-following
// (FINDING_ROCK, LINE_FOLLOWING, LINE_FOLLOWING_REVERSE) or runs a stationary
// ArmPoseSequence (see robot_poses.h) that MrKrabs steps through by rotating
// once to the sequence's heading, then applying each pose via SequenceRunner.
class AI {
	public:
		AI();

		// Advances the state machine. Called once per control loop tick by
		// MrKrabs, but only outside of settle delays.
		void tickAI();

		// Stub odometry: accumulates distance traveled in the current state.
		// Currently fed from commanded speed (open-loop dead reckoning) by
		// MrKrabs, not real encoder feedback — see MrKrabs::driveCurrentMode.
		void addProgress(double delta_m);

		// Dead-reckoned position (m) from the start of the run, in the frame the
		// robot started in — MotorController::getFieldPose(), pushed in by
		// MrKrabs::stepControl every tick. Absolute, so unlike addProgress()'s
		// per-state odometer it isn't reset by a state change, which is what
		// lets the rock checkpoints be positions on the field rather than
		// distances into a leg. Both axes are needed because the checkpoints
		// before the course turns are reached in y and the one after it in x.
		// See tickFindingRock/kRockCheckpoints.
		void setFieldPosition(double x_m, double y_m);

		// Pushed by MrKrabs::driveCurrentMode's SEARCHING_FOR_LINE case: false
		// while the fixed initial turn is still in progress, true once it's
		// reached its target and the reactive continuous spin has taken over.
		// tickReacquiringLine() ignores the line sensors until this is true —
		// otherwise a state that left the robot already sitting on the line
		// (HABITAT_PICKUP/HABITAT_PLACE, which don't rotate) could see
		// bothMidSensorsOnLine() read true a few degrees into the turn, long
		// before the intended fixed amount, cutting the turn short.
		void setLineSearchInitialTurnDone(bool done);

		// True when the upcoming REACQUIRING_LINE sweep should carry on turning
		// the SAME way as the last rotation instead of reversing it, which is
		// what MrKrabs::startSearchingForLine does by default. Set only on the
		// hand-off out of HABITAT_PLACE — see tickHabitatPlace.
		bool lineSearchContinuesLastRotation() const;

		void setArm(Arm* arm);
		void setLineFollower(LineFollower* line_follower);
		void setSonar(Sonar* sonar);

		// Called by MrKrabs when the RPi reports a Teletubby (Command::
		// teletubby_detected). Only takes effect while current_state_ is
		// METAL_DETECTING — see tickMetalDetecting; ignored otherwise so a
		// stray/late detection can't queue up a TELETUBBYING entry later.
		// Also ignored once TELETUBBYING has already run kMaxTeletubbies
		// times — the RPi has no memory of which teletubbies it already
		// reported (see computer_vision.py), so the cap lives here instead.
		void notifyTeletubbyDetected();

		enum class DriveMode { LINE_FOLLOWING, APPLYING_SEQUENCE, SEARCHING_FOR_LINE, IDLE, ROTATING_TIL_ROCK , STRAFING_TIL_HABITAT, BACKING_UP, HOLDING_AND_MOVING, REVERSE_180, SQUARING_UP, ROCK_SEARCHING_FOR_LINE, DRIVING_FORWARD};
		// What the drivetrain should currently be doing, derived from current_state_.
		DriveMode desiredDriveMode() const;
		// +1 while driving forward, -1 during LINE_FOLLOWING_REVERSE.
		double lineFollowingDirection() const;

		// Speed (m/s) the current line-following leg should drive at:
		// HABITAT_FORWARD_SPEED for the habitat legs, FORWARD_SPEED for the rest.
		// Valid while desiredDriveMode() == LINE_FOLLOWING. Also the speed
		// LineFollower::calculateCorrection() must be given, so its corrections
		// scale to match.
		double lineFollowingSpeed() const;
		// Valid while desiredDriveMode() == ROTATING_TIL_ROCK. Sign (+1/-1) of
		// the current rock checkpoint's heading — see tickRotatingTilRock.
		double rockSearchOmegaRadS() const;
		// How long (us) the sonar must hold in range before ROTATING_TIL_ROCK
		// commits to a rock, chosen by the direction of this checkpoint's search
		// spin — see kRockSonarConfirmDelayCcwUs/CwUs in ai.cpp. RAMP_LINE_
		// FOLLOWING doesn't use this: it has no spin, so no direction to pick by.
		uint64_t rockSearchConfirmDelayUs() const;
		// Valid while desiredDriveMode() == STRAFING_TIL_HABITAT. Sign (+1/-1)
		// of the strafe direction — flipped by tickHabitatFind() once the
		// second habitat slot has been found.
		int habitatFindDirection() const;
		// Valid while desiredDriveMode() == SQUARING_UP. Sign (+1 = CCW) of the
		// in-place rotation that brings the not-yet-triggered side sensor onto
		// the habitat strip, latched by tickHabitatLineFollowing() from whichever
		// sensor fired first — see RobotState::HABITAT_SQUARE_UP.
		double squareUpOmegaSign() const;

		// Valid while desiredDriveMode() == APPLYING_SEQUENCE. Delegates to sequence_runner_.
		double targetRotationDegrees() const;
		// Called by MrKrabs once the drivetrain has reached the current target
		// heading. Delegates to sequence_runner_; returns true iff a pose was
		// just written (so the caller knows to start a settle delay).
		bool onRotationReached();
		// This sequence's per-pose settle delay (see ArmPoseSequence::pose_settle_us
		// in robot_poses.h). Delegates to sequence_runner_; valid immediately
		// after onRotationReached() returns true.
		uint64_t sequencePoseSettleUs() const;

		// Raw MetalDetector reading (interrupt-driven, see metal_detector.cpp) —
		// exposed for MrKrabs's debug print, independent of METAL_DETECTING's
		// own gating in tickMetalDetecting().
		bool metalDetected();

		RobotState currentState() const;
		// How many times state has been entered (the current state included,
		// counting from 1). Used by per-state handlers to index checkpoint/
		// threshold tables — see tickFindingRock, tickHabitatLineFollowing.
		int visits(RobotState state) const;

	private:
		void transitionTo(RobotState next);
		RobotState tickCurrentState();
		// Shared by tickMetalDetecting/tickPickupRock: once a rock checkpoint
		// is resolved (picked up or skipped), decide whether to look for the
		// next rock or move on to the final line-following leg.
		RobotState nextRockOrDone();

		// See RobotState::REACQUIRING_LINE.
		RobotState tickReacquiringLine();
		// See RobotState::REVERSE_180. Same completion condition as
		// tickReacquiringLine (fixed initial turn, then both mid sensors on
		// the line) — only the drivetrain's direction/initial angle differ,
		// and those live in MrKrabs::startReverse180().
		RobotState tickReverse180();
		// See RobotState::ROCK_REACQUIRE_LINE. Same completion test as
		// tickReacquiringLine, minus the fixed-initial-turn gate — the rock search
		// already rotated the robot off the line, so there is nothing to clear.
		RobotState tickRockReacquireLine();

		RobotState tickFindingRock();
		RobotState tickRotatingTilRock();
		RobotState tickMetalDetecting();
		RobotState tickTeletubbying();
		RobotState tickPickupRock();
		// See RobotState::LINE_FOLLOWING — line-follows until the field y
		// reaches rampTriggerYM(), then hands to the ramp leg.
		RobotState tickLineFollowing();
		// Field y (m) at which the ramp leg starts: the end of the rock
		// checkpoint chain, driven straight through without stopping to probe
		// any of the checkpoints left unvisited, plus kPostRocksRampDistanceM
		// once the rock quota is filled. Being absolute, it reaches the ramp at
		// the same point on the field whichever checkpoint the first rock came
		// from.
		double rampTriggerYM() const;
		// See RobotState::RAMP_LINE_FOLLOWING / RobotState::SECOND_ROCK_FIND.
		RobotState tickRampLineFollowing();
		RobotState tickSecondRockFind();
		RobotState tickHabitatLineFollowing();
		RobotState tickHabitatPickup();
		RobotState tickLineFollowingReverse();
		RobotState tickHabitatPlace();
		// See RobotState::HABITAT_SMACK. Backward leg first, then the arm
		// sequence — which phase it's in is smack_sequence_started_ below.
		RobotState tickHabitatSmack();
		// See RobotState::HABITAT_SMACK_FORWARD.
		RobotState tickHabitatSmackForward();
		RobotState tickHabitatSquareUp();
		RobotState tickHabitatApproachBackup();
		RobotState tickHabitatFind();
		RobotState tickHabitatBackup();
		RobotState tickHabitatPostPickupBackup();
		RobotState tickHabitatHoldAndMove();

		static constexpr size_t idx(RobotState s) { return static_cast<size_t>(s); }

		MetalDetector metal_detector_;
		Arm* arm_;
		LineFollower* line_follower_;
		Sonar* sonar_ = nullptr;
		SequenceRunner sequence_runner_;
		XySequenceRunner xy_sequence_runner_;

		RobotState current_state_;
		// Set by notifyTeletubbyDetected(), consumed and cleared by
		// tickMetalDetecting() on its next tick.
		bool teletubby_detected_ = false;
		// Sonar reading (m, mount-offset corrected) cached by
		// tickRotatingTilRock() once a rock is found; the origin
		// kPickupRockXYSequence's sonar-relative poses are offset from.
		double cached_rock_sonar_x_m_ = 0.0;
		// esp_timer_get_time() timestamp the sonar first entered the
		// rock-detection range this ROTATING_TIL_ROCK visit, or 0 if not
		// currently in range — see tickRotatingTilRock's debounce.
		uint64_t rock_sonar_in_range_since_us_ = 0;
		// Same debounce for RAMP_LINE_FOLLOWING's forward-looking check: when the
		// sonar first read inside the second-rock window, or 0 if it currently
		// isn't. Separate from the rock-search one so the two can't consume each
		// other's confirmation. See tickRampLineFollowing.
		uint64_t ramp_sonar_in_range_since_us_ = 0;
		// Last plausible sonar reading (cm) of the second rock, and the
		// SECOND_ROCK_FIND progress (m) it was taken at. Seeded by
		// tickRampLineFollowing() from the reading that confirmed the rock, then
		// refreshed each approach tick. Both are needed, not just the range:
		// SECOND_ROCK_FIND now ends on distance rather than on the sonar, so the
		// last reading is generally older than the stopping point and has to be
		// corrected by the travel since. See tickSecondRockFind.
		double second_rock_sonar_cm_ = 0.0;
		double second_rock_sonar_progress_m_ = 0.0;
		// esp_timer_get_time() timestamp HABITAT_FIND's sonar first read closer
		// than HABITAT_SIDE_THRESHOLD, or 0 before that happens — the start of
		// kHabitatSideStopDelayUs's extra-strafe countdown. Cleared again when
		// the countdown completes, so the next HABITAT_FIND visit starts fresh.
		uint64_t habitat_side_seen_us_ = 0;
		// Guards METAL_DETECTING's probe pose (kPickupRockXYSequence[0]) so it's
		// applied exactly once and then held until metal is detected, instead
		// of auto-advancing to the next pose every settle cycle. Reset by
		// transitionTo() on entering METAL_DETECTING. See onRotationReached().
		bool metal_probe_pose_applied_ = false;
		// False while HABITAT_SMACK is still driving its backward leg, true once
		// tickHabitatSmack() has handed kHabitatSmackXYSequence to
		// xy_sequence_runner_. This is the state's phase bit: desiredDriveMode()
		// reads it to pick BACKING_UP vs APPLYING_SEQUENCE, and the arm-sequence
		// accessors (targetRotationDegrees/onRotationReached/sequencePoseSettleUs)
		// only route to the XY runner once it's set — before that the runner still
		// holds HABITAT_PLACE's finished sequence. Reset by transitionTo().
		bool smack_sequence_started_ = false;
		// How many rocks have actually been collected — incremented on entering
		// PICKUP_ROCK, which only happens off a real metal hit, so a rock skipped
		// for lack of metal doesn't count. nextRockOrDone() routes the run off
		// this: 1 sends it down the line to the ramp, 2 back to the checkpoint
		// machinery for the third, kRocksToCollect stops the searching. Never
		// cleared.
		int rocks_collected_ = 0;
		// esp_timer_get_time() stamp of the tick the probe pose landed, or 0
		// before that — the start of kMetalProbeTimeoutUs's window. Timed from
		// the pose landing rather than from entering the state so the arm's
		// travel out doesn't count against the detector's chance to read. Reset
		// by transitionTo() on entering METAL_DETECTING.
		uint64_t metal_probe_started_us_ = 0;
		std::array<int, kNumRobotStates> state_visit_count_{};
		// Distance (m) traveled since the current state was entered.
		double current_state_progress_m_;
		// See setFieldPosition(). Never reset — these are field coordinates, not
		// leg odometers.
		double field_x_m_ = 0.0;
		double field_y_m_ = 0.0;
		// Where tickReacquiringLine() should hand off once the line is
		// found. Set by whichever tick*() function transitions into
		// REACQUIRING_LINE (see nextRockOrDone, tickHabitatPickup, etc.).
		RobotState post_reacquire_state_;
		// Where tickLineFollowingReverse() should hand off once its distance
		// leg completes — HABITAT_PLACE after the outbound (post-pickup) leg,
		// HABITAT_PICKUP after the return (post-place) leg. Set by whichever
		// tick*() function transitions into LINE_FOLLOWING_REVERSE via
		// REACQUIRING_LINE.
		RobotState post_line_reverse_state_ = RobotState::HABITAT_PLACE;
		// See setLineSearchInitialTurnDone().
		bool line_search_initial_turn_done_ = false;
		// See lineSearchContinuesLastRotation(). Every tick*() that hands off to
		// REACQUIRING_LINE sets this explicitly, so it always describes the sweep
		// about to start rather than a stale one.
		bool line_search_continues_last_rotation_ = false;


		// Control ticks elapsed since exactly one side sensor first read the
		// habitat strip, 0 when no such window is open — see
		// kSideSensorGraceTicks in ai.cpp. Counts ticks in which the state
		// machine actually ran, which is what we want: tickAI() is skipped during
		// settle delays, and no travel happens then either.
		int side_sensor_grace_ticks_ = 0;
		// Rotation direction for HABITAT_SQUARE_UP (+1 = CCW), and the
	// esp_timer_get_time() stamp that state was entered at, so the rotation
		// can't spin forever if the second sensor never reads the strip. Both set
		// on entering the state — see tickHabitatLineFollowing/tickHabitatSquareUp.
		double square_up_omega_sign_ = 1.0;
		uint64_t square_up_entered_us_ = 0;

		int habitat_found_num_ = 0;
		int current_habitat_find_direction_ = 1;

		// Sonar reading (cm) below which HABITAT_FIND counts itself as having
		// reached the habitat — the only threshold it uses, now that the
		// second (depth) check is gone. See tickHabitatFind.
		static constexpr double HABITAT_SIDE_THRESHOLD = 5.8;

};
