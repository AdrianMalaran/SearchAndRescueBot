#include "Flame.h"

const int analog_pin = 15;
const int digital_pin = 45;

Flame::Flame() {
	setupFlame();
}

static void Flame::setupFlame() {
	pinMode(digital_pin, INPUT);
}

static bool Flame::isFire() {
	return digitalRead(digital_pin) == LOW ? true : false;
}

static int Flame::getFireMagnitude() {
	return map(analogRead(analog_pin), 1024, 0, 0, 100);
}
