#include "sequence_runner.h"

void SequenceRunner::start(const RobotSequence& sequence) {
	sequence_ = &sequence;
	index_ = 0;
}

bool SequenceRunner::complete() const {
	return sequence_ == nullptr || index_ >= sequence_->size();
}

double SequenceRunner::targetRotationDegrees() const {
	if (complete()) return 0.0;
	return (*sequence_)[index_].rotation_degrees;
}

bool SequenceRunner::onRotationReached(Arm& arm) {
	if (complete()) return false;
	arm.setPose((*sequence_)[index_].arm_pose);
	++index_;
	return true;
}
