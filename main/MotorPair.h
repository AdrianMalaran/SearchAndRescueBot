#ifndef MotorPair_h
#define MotorPair_h

#include "Arduino.h"
#include "Imu.h"
#include "Flame.h"
#include "Fan.h"

class MotorPair {
	public:
		MotorPair();
		MotorPair(Imu imu_sensor);
		void setupMotorPair();
		static void stop();
    	void standby();
		static void setMotorASpeed(int speed);
		static void setMotorBSpeed(int speed);
		void turnLeft();
		void turnRight();
		bool extinguishFireTurn();
    private:
		Imu m_imu_sensor;
		double m_orientation;
};

#endif
