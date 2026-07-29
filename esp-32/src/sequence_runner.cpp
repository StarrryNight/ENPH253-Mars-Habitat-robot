#include "sequence_runner.h"

void SequenceRunner::start(const ArmPoseSequence& sequence) {
	sequence_ = &sequence;
	index_ = 0;
}

bool SequenceRunner::complete() const {
	return sequence_ == nullptr || index_ >= sequence_->poses.size();
}

double SequenceRunner::targetRotationDegrees() const {
	if (complete()) return 0.0;
	return sequence_->rotation_degrees;
}

uint64_t SequenceRunner::poseSettleUs() const {
	if (sequence_ == nullptr) return 0;
	return sequence_->pose_settle_us;
}

bool SequenceRunner::onRotationReached(Arm& arm) {
	if (complete()) return false;
	arm.setPose(sequence_->poses[index_]);
	++index_;
	return true;
}

void XySequenceRunner::start(const RockXYSequence& sequence, double sonar_x_origin_m) {
	sequence_ = &sequence;
	index_ = 0;
	sonar_x_origin_m_ = sonar_x_origin_m;
}

bool XySequenceRunner::complete() const {
	return sequence_ == nullptr || index_ >= sequence_->poses.size();
}

size_t XySequenceRunner::currentIndex() const {
	return index_;
}

uint64_t XySequenceRunner::poseSettleUs() const {
	if (sequence_ == nullptr) return 0;
	return sequence_->pose_settle_us;
}

bool XySequenceRunner::onRotationReached(Arm& arm) {
	if (complete()) return false;
	ArmCoordinate pose = sequence_->poses[index_];
	if (pose.x_is_sonar_relative) {
		pose.x_pos = sonar_x_origin_m_ + pose.x_pos;
	}
	arm.setPoseXY(pose);
	++index_;
	return true;
}
