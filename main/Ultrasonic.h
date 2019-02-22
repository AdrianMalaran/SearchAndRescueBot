#ifndef Ultrasonic_h
#define Ultrasonic_h

#include "Arduino.h"

class Ultrasonic {
	public:
		Ultrasonic(int trigPin, int echoPin);
		float getDistance();
    private:
		int _trigPin;
		int _echoPin;
};

#endif