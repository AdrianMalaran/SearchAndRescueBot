#include "LED.h"
#include <Arduino.h>

const int LEDPin = 34;

LED::LED() {
    pinMode(LEDPin, OUTPUT);
    digitalWrite(LEDPin, LOW);
}

static void LED::on() {
	digitalWrite(LEDPin, HIGH);
}

static void LED::off() {
	digitalWrite(LEDPin, LOW);
}
