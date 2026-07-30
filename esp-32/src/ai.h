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

		enum class DriveMode { LINE_FOLLOWING, APPLYING_SEQUENCE, SEARCHING_FOR_LINE, IDLE, ROTATING_TIL_ROCK , STRAFING_TIL_HABITAT, BACKING_UP, HOLDING_AND_MOVING, REVERSE_180, SQUARING_UP};
		// What the drivetrain should currently be doing, derived from current_state_.
		DriveMode desiredDriveMode() const;
		// +1 while driving forward, -1 during LINE_FOLLOWING_REVERSE.
		double lineFollowingDirection() const;
		// Valid while desiredDriveMode() == ROTATING_TIL_ROCK. Sign (+1/-1) of
		// the current rock checkpoint's heading — see tickRotatingTilRock.
		double rockSearchOmegaRadS() const;
		// Valid while desiredDriveMode() == STRAFING_TIL_HABITAT. Sign (+1/-1)
		// of the strafe direction — flipped by tickHabitatFind() once the
		// second habitat slot has been found.
		int habitatFindDirection() const;
		// Valid while desiredDriveMode() == SQUARING_UP: the signed angle (rad,
		// +ve = CCW) the robot should rotate through to square onto the habitat
		// strip. Computed by tickHabitatLineFollowing() from the distance the
		// robot travelled between its two side sensors reaching the strip, not
		// searched for — see RobotState::HABITAT_SQUARE_UP.
		double squareUpTargetRad() const;
		// Sign (+1 = CCW) of squareUpTargetRad(), for the drivetrain to pick a
		// rotation direction.
		double squareUpOmegaSign() const;

		// Pushed by MrKrabs::driveCurrentMode's SQUARING_UP case once the
		// measured rotation has covered squareUpTargetRad(). tickHabitatSquareUp()
		// waits on this rather than on the line sensors: by the time the rotation
		// starts, both side sensors are already on the strip (that's what ended
		// the measurement), so they can't say when the turn is done.
		void setSquareUpDone(bool done);

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

		RobotState tickFindingRock();
		RobotState tickRotatingTilRock();
		RobotState tickMetalDetecting();
		RobotState tickTeletubbying();
		RobotState tickPickupRock();
		RobotState tickHabitatLineFollowing();
		RobotState tickHabitatPickup();
		RobotState tickLineFollowingReverse();
		RobotState tickHabitatPlace();
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


		// Control ticks elapsed since exactly one side sensor first read the
		// habitat strip, 0 when no such window is open — see
		// kSideSensorGraceTicks in ai.cpp. Counts ticks in which the state
		// machine actually ran, which is what we want: tickAI() is skipped during
		// settle delays, and no travel happens then either.
		int side_sensor_grace_ticks_ = 0;
		// Sign (+1 = CCW) of the correction, latched from whichever side sensor
		// reached the strip first — see tickHabitatLineFollowing.
		double square_up_omega_sign_ = 1.0;
		// current_state_progress_m_ at the moment that first sensor triggered.
		// Diffed against the reading when the second one triggers to get the
		// travel the misalignment angle is computed from — progress is fed from
		// the encoder-integrated odometer (MrKrabs::driveCurrentMode ->
		// addProgress), so this is measured travel, not commanded.
		double square_up_start_progress_m_ = 0.0;
		// The correction itself (rad, signed), computed once both sensors have
		// triggered and then executed open-loop-to-target by the drivetrain.
		double square_up_target_rad_ = 0.0;
		// Set by setSquareUpDone() once the drivetrain reports the rotation
		// covered; cleared on entering the state.
		bool square_up_done_ = false;
		// esp_timer_get_time() stamp of HABITAT_SQUARE_UP's first tick, so the
		// rotation can't run forever if it never reaches target. Started on that
		// first tick rather than at the transition — see transitionTo.
		uint64_t square_up_entered_us_ = 0;

		int habitat_found_num_ = 0;
		int current_habitat_find_direction_ = -1;

		// Sonar reading (cm) below which HABITAT_FIND counts itself as having
		// reached the habitat — the only threshold it uses, now that the
		// second (depth) check is gone. See tickHabitatFind.
		static constexpr double HABITAT_SIDE_THRESHOLD = 5;

};
