#include "MotorPair.h"

//////////////////// TODO ////////////////////
// We need to decide how to move:
//   pass a time based value |
//   each call is one tile of motion |
//   pass a distance based value;
//
// Also do we need to deal with ramping to avoid
// possible brownouts? Do we need to handle 
// ramping for turns?
//
// Code assumes A is left, B is right
//
// Motor speed is 0-255

const int MAX_SPEED = 255, MIN_SPEED = 0;

MotorPair::MotorPair(int enable_a, int input1, int input2, int enable_b, int input3, int input4) {
	// Set motor A pins
	pinMode(enable_a, OUTPUT);
	pinMode(input1, OUTPUT);
	pinMode(input2, OUTPUT);

	// Set motor B pins
	pinMode(enable_b, OUTPUT);
	pinMode(input3, OUTPUT);
	pinMode(input4, OUTPUT);

	// Set motor A member vars
	m_enable_a = enable_a;
	m_input1 = input1;
	m_input2 = input2;

	// Set motor B member vars
	m_enable_b = enable_b;
	m_input3 = input3;
	m_input4 = input4;
}

void MotorPair::stop() {
	analogWrite(m_enable_a, MIN_SPEED);
	analogWrite(m_enable_b, MIN_SPEED);
}

void MotorPair::rampUp(int set_speed) {
	for(int speed = MIN_SPEED; speed < set_speed; speed++) {
		analogWrite(m_enable_a, speed);
		analogWrite(m_enable_b, speed);
	}
}

void MotorPair::rampDown(int curr_speed) {
	for(int speed = curr_speed; speed > MIN_SPEED; speed--) {
		analogWrite(m_enable_a, speed);
		analogWrite(m_enable_b, speed);
	}
}

void MotorPair::moveForwards() {
	digitalWrite(m_input1, LOW);
	digitalWrite(m_input2, HIGH);
	digitalWrite(m_input3, LOW);
	digitalWrite(m_input4, HIGH);

	rampUp(MAX_SPEED);

	analogWrite(m_enable_a, MAX_SPEED);
	analogWrite(m_enable_b, MAX_SPEED);
	delay(1000);

	rampDown(MAX_SPEED);
	stop();
}

void MotorPair::moveBackwards() {
	digitalWrite(m_input1, HIGH);
	digitalWrite(m_input2, LOW);
	digitalWrite(m_input3, HIGH);
	digitalWrite(m_input4, LOW);

	rampUp(MAX_SPEED);

	analogWrite(m_enable_a, MAX_SPEED);
	analogWrite(m_enable_b, MAX_SPEED);
	delay(1000);

	rampDown(MAX_SPEED);
	stop();
}

void MotorPair::turnLeft() {
	digitalWrite(m_input1, LOW);
	digitalWrite(m_input2, HIGH);
	digitalWrite(m_input3, HIGH);
	digitalWrite(m_input4, LOW);

	rampUp(MAX_SPEED/2);

	analogWrite(m_enable_a, MAX_SPEED/2);
	analogWrite(m_enable_b, MAX_SPEED/2);
	delay(500);

	rampDown(MAX_SPEED/2);
	stop();
}

void MotorPair::turnRight() {
	digitalWrite(m_input1, HIGH);
	digitalWrite(m_input2, LOW);
	digitalWrite(m_input3, LOW);
	digitalWrite(m_input4, HIGH);

	rampUp(MAX_SPEED/2);

	analogWrite(m_enable_a, MAX_SPEED/2);
	analogWrite(m_enable_b, MAX_SPEED/2);
	delay(500);

	rampDown(MAX_SPEED/2);
	stop();
}
