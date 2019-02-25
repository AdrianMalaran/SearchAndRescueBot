#include "Ultrasonic.h"

float distance, duration;

Ultrasonic::Ultrasonic(int trig_pin, int echo_pin) {
	pinMode(trig_pin, OUTPUT);
	pinMode(echo_pin, INPUT);
	m_trig_pin = trig_pin;
	m_echo_pin = echo_pin;
}

float Ultrasonic::getDistance() {
	// Clears the trigPin
	digitalWrite(m_trig_pin, LOW);
	delayMicroseconds(5);

  // Sets the trigPin on HIGH state for 10 micro seconds
  digitalWrite(m_trig_pin, HIGH);
  delayMicroseconds(10);
  digitalWrite(m_trig_pin, LOW);

  // Reads the echoPin, returns the sound wave travel time in microseconds
  duration = pulseIn(m_echo_pin, HIGH);

  // Calculating the distance
  distance = duration * 0.0343 / 2;

  return distance;
}
