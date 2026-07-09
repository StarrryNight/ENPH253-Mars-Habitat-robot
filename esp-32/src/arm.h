#pragma once
class Arm
{
	enum ArmPoses {METAL, GRAB_HABITAT, HOLD_HABITAT, PLACE_HABITAT};
	public:
		Arm();
		void tickArm();
	private:
		void p;
		ArmPoses current_pose_;
		



};
