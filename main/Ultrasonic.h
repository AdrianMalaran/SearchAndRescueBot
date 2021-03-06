#ifndef Ultrasonic_h
#define Ultrasonic_h

#include "Arduino.h"

class Ultrasonic {
	public:
		Ultrasonic();
		Ultrasonic(int trig_pin, int echo_pin);
		double getDistance();
    private:
		int m_trig_pin;
		int m_echo_pin;
};

#endif