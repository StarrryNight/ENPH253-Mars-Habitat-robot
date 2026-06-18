#include "line_follower.h"
class MrKrab
{

public:
	void setup();

	void reset();

	void update();

private:
	void stepControl();
	LineFollower line_follower_;
};
