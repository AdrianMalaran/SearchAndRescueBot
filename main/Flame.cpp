#include "Flame.h"

Flame::Flame(int digital_pin) {
	pinMode(digital_pin, INPUT);
	m_digital_pin = digital_pin;
}

Flame::Flame(int digital_pin, int analog_pin) {
	pinMode(digital_pin, INPUT);
	m_digital_pin = digital_pin;
	m_analog_pin = analog_pin;
}

bool Flame::isFire() {
	return digitalRead(m_digital_pin) == LOW ? true : false;
}

int Flame::getFireMagnitude() {
	return map(analogRead(m_analog_pin), 1024, 0, 0, 100);
}
