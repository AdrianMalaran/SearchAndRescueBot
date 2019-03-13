#ifndef LED_h
#define LED_h

#include <Arduino.h>

class LED {
	public:
		LED();
		static void on();
		static void off();
};

#endif
