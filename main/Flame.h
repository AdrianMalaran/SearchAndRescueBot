#ifndef Flame_h
#define Flame_h

#include "Arduino.h"

class Flame {
	public:
		Flame();
		static void setupFlame();
		static bool isFire();
		static int getFireMagnitude();
    private:
};

#endif
