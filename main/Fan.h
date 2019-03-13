#ifndef Fan_h
#define Fan_h

#include <Arduino.h>

class Fan {
	public:
		Fan();
		static void on();
		static void off();
};

#endif
