#pragma once

enum MotorState {Stopped,Forward, Backward, Clockwise, CounterClockwise};
class MotorDriver{

public:
MotorDriver(MotorState initial_state){
	current_state = initial_state;
}
void temp(){


}
private:



MotorState current_state;

};
