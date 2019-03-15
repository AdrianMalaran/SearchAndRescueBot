#include "Fan.h"
#include <Arduino.h>

const int FanPin = 33;

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
