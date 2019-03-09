#ifndef MotorPair_h
#define MotorPair_h

#include "Arduino.h"

class MotorPair {
	public:
		MotorPair();
		static void stop();
		static void setMotorASpeed(int speed);
		static void setMotorBSpeed(int speed);
		static void moveForwards();
		static void moveBackwards();
		static void turnLeft();
		static void turnRight();
    private:
		static void rampUp(int set_speed);
		static void rampDown(int curr_speed);
};

#endif
