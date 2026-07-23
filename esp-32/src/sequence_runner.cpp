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

bool SequenceRunner::onRotationReached(Arm& arm) {
	if (complete()) return false;
	arm.setPose(sequence_->poses[index_]);
	++index_;
	return true;
}
