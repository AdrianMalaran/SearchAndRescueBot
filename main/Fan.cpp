#include "Fan.h"
#include <Arduino.h>

const int FanPin = 34;

Fan::Fan() {
    pinMode(FanPin, OUTPUT);
    digitalWrite(FanPin, LOW);
}

static void Fan::on() {
	digitalWrite(FanPin, HIGH);
}

static void Fan::off() {
	digitalWrite(FanPin, LOW);
}
