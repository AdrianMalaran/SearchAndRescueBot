#include "Ultrasonic.h"

double distance, duration, temp = 19;

Ultrasonic::Ultrasonic() {}

Ultrasonic::Ultrasonic(int trig_pin, int echo_pin) {
	pinMode(trig_pin, OUTPUT);
	pinMode(echo_pin, INPUT);
	m_trig_pin = trig_pin;
	m_echo_pin = echo_pin;
}

double Ultrasonic::getDistance() {
	// Clears the trigPin
	digitalWrite(m_trig_pin, LOW);
	delayMicroseconds(5);

  // Sets the trigPin on HIGH state for 10 micro seconds
  digitalWrite(m_trig_pin, HIGH);
  delayMicroseconds(10);
  digitalWrite(m_trig_pin, LOW);

  // Reads the echoPin, returns the sound wave travel time in microseconds
  duration = pulseIn(m_echo_pin, HIGH);

  double speedOfSound = 331.3 + 0.606 * temp;

  // Calculating the distance
  distance = (duration / 20000.0) * speedOfSound;

  return distance;
}
