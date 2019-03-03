#ifndef Flame_h
#define Flame_h

#include "Arduino.h"

class Flame {
	public:
		Flame(int digital_pin);
		Flame(int digital_pin, int analog_pin);
		bool isFire();
		int getFireMagnitude();
    private:
		int m_digital_pin;
		int m_analog_pin;
};

#endif
