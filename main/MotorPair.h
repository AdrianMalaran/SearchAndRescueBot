#ifndef MotorPair_h
#define MotorPair_h

#include "Arduino.h"
#include "Imu.h"
#include "Flame.h"


/*
TODO: Should the MotorPair have access to the IMU ?
*/
class MotorPair {
	public:
		MotorPair();
		MotorPair(Imu imu_sensor);
		void setupMotorPair();
		static void stop();
    	void standby();
		static void setMotorASpeed(int speed);
		static void setMotorBSpeed(int speed);
		void moveForwards();
		void moveBackwards();
		void turnLeft();
		void turnRight();
		void extinguishFireTurn();
    private:
		void rampUp(int set_speed);
		void rampDown(int curr_speed);
		Imu m_imu_sensor;
		double m_orientation;
};

#endif
