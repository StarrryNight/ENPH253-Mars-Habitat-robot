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

		// Pushed by MrKrabs::driveCurrentMode's SEARCHING_FOR_LINE case: false
		// while the fixed initial turn is still in progress, true once it's
		// reached its target and the reactive continuous spin has taken over.
		// tickReacquiringLine() ignores the line sensors until this is true —
		// otherwise a state that left the robot already sitting on the line
		// (HABITAT_PICKUP/HABITAT_PLACE, which don't rotate) could see
		// bothMidSensorsOnLine() read true a few degrees into the turn, long
		// before the intended fixed amount, cutting the turn short.
		void setLineSearchInitialTurnDone(bool done);

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

		enum class DriveMode { LINE_FOLLOWING, APPLYING_SEQUENCE, SEARCHING_FOR_LINE, IDLE, ROTATING_TIL_ROCK };
		// What the drivetrain should currently be doing, derived from current_state_.
		DriveMode desiredDriveMode() const;
		// +1 while driving forward, -1 during LINE_FOLLOWING_REVERSE.
		double lineFollowingDirection() const;
		// Valid while desiredDriveMode() == ROTATING_TIL_ROCK. Sign (+1/-1) of
		// the current rock checkpoint's heading — see tickRotatingTilRock.
		double rockSearchOmegaRadS() const;

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
		// threshold tables — see tickFindingRock, tickLineFollowing.
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

		RobotState tickFindingRock();
		RobotState tickRotatingTilRock();
		RobotState tickMetalDetecting();
		RobotState tickTeletubbying();
		RobotState tickPickupRock();
		RobotState tickLineFollowing();
		RobotState tickHabitatPickup();
		RobotState tickLineFollowingReverse();
		RobotState tickHabitatPlace();

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
		// Guards METAL_DETECTING's probe pose (kPickupRockXYSequence[0]) so it's
		// applied exactly once and then held until metal is detected, instead
		// of auto-advancing to the next pose every settle cycle. Reset by
		// transitionTo() on entering METAL_DETECTING. See onRotationReached().
		bool metal_probe_pose_applied_ = false;
		std::array<int, kNumRobotStates> state_visit_count_{};
		// Distance (m) traveled since the current state was entered.
		double current_state_progress_m_;
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
};
