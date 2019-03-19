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
		// MotorPair(Imu imu_sensor, Ultrasonic ultrasonic_front);
		void setupMotorPair();
		static void stop();
    	void standby();

		static void setMotorAPWM(int PWM);
		static void setMotorBPWM(int PWM);

		void turnLeft(int speed);
		void turnRight();
    private:
		Imu m_imu_sensor;
		// Ultrasonic m_ultrasonic_front;
		double m_orientation;
};

#endif
