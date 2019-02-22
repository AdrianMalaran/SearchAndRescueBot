#ifndef Flame_h
#define Flame_h

#include "Arduino.h"

class Flame {
	public:
		Flame(int digitalPin);
		Flame(int digitalPin, int analogPin);
		bool isFire();
		float getFireMagnitude();
    private:
		int _digitalPin;
		int _analogPin;
};

#endif
