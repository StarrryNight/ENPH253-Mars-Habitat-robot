#include "line_follower.h"
#include "orientation_controller.h"
#include "motor_controller.h"
class MrKrabs
{

public:
	void setup();

	void reset();

	void update();

private:
	void stepControl();
	void startRotation();

	bool is_rotating_;
	LineFollower line_follower_;
	OrientationController orientation_controller_;
	MotorController motor_controller_;
};
