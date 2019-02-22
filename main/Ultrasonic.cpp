#include "Ultrasonic.h"

float distance, duration;

Ultrasonic::Ultrasonic(int trigPin, int echoPin) {
	pinMode(trigPin, OUTPUT);
	pinMode(echoPin, INPUT);
	_trigPin = trigPin;
	_echoPin = echoPin;
}

float Ultrasonic::getDistance() {
	// Clears the trigPin
	digitalWrite(_trigPin, LOW);
	delayMicroseconds(5);

  // Sets the trigPin on HIGH state for 10 micro seconds
  digitalWrite(_trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(_trigPin, LOW);

  // Reads the echoPin, returns the sound wave travel time in microseconds
  duration = pulseIn(_echoPin, HIGH);

  // Calculating the distance
  distance = duration * 0.0343 / 2;

  return distance;
}
