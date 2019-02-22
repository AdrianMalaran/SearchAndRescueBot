#include "Flame.h"

Flame::Flame(int digitalPin) {
	pinMode(digitalPin, INPUT);
	_digitalPin = digitalPin;
}

Flame::Flame(int digitalPin, int analogPin) {
  pinMode(digitalPin, INPUT);
  _digitalPin = digitalPin;
  _analogPin = analogPin;
}

bool Flame::isFire() {
  return digitalRead(_digitalPin) == LOW ? true : false;
}

float Flame::getFireMagnitude() {
  return analogRead(_analogPin);
}
