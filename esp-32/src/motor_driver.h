#pragma once

enum MotorState {Stopped,Forward, Backward, Clockwise, CounterClockwise};
class MotorDriver{

public:
	MotorDriver(MotorState initial_state){
		current_state = initial_state;
	}
	private:
	
	
	void changeDirectionBuffer();
	
	MotorState current_state;

};
