#ifndef MotorPair_h
#define MotorPair_h

#include "Arduino.h"

class MotorPair {
	public:
		MotorPair(int enable_a, int input1, int input2, int enable_b, int input3, int input4);
		void stop();
		void moveForwards();
		void moveBackwards();
		void turnLeft();
		void turnRight();
    private:
		void rampUp(int set_speed);
		void rampDown(int curr_speed);
		int m_enable_a;
		int m_input1;
		int m_input2;
		int m_enable_b;
		int m_input3;
		int m_input4;
};

#endif
