#include "MotorPair.h"

//////////////////// TODO ////////////////////
// We need to decide how to move:
//   pass a time based value |
//   each call is one tile of motion |
//   pass a distance based value;
//
// Also do we need to deal with ramping to astatic void
// possible brownouts? Do we need to handle
// ramping for turns?
//
// Code assumes A is left, B is right
//
// Motor speed is 0-255

const int MAX_SPEED = 255, MIN_SPEED = 0;

// MotorPair motor_pair(9, 3, 4, 10, 5, 6);

// Motor A
const int enable_a = 9;
const int input1 = 3;
const int input2 = 4;

// Motor B
const int enable_b = 10;
const int input3 = 5;
const int input4 = 6;

MotorPair::MotorPair() {
	// Set motor A pins
	pinMode(enable_a, OUTPUT);
	pinMode(input1, OUTPUT);
	pinMode(input2, OUTPUT);

	// Set motor B pins
	pinMode(enable_b, OUTPUT);
	pinMode(input3, OUTPUT);
	pinMode(input4, OUTPUT);
}

static void MotorPair::stop() {
	analogWrite(enable_a, MIN_SPEED);
	analogWrite(enable_b, MIN_SPEED);
}

static void MotorPair::rampUp(int set_speed) {
	for(int speed = MIN_SPEED; speed < set_speed; speed++) {
		analogWrite(enable_a, speed);
		analogWrite(enable_b, speed);
	}
}

static void MotorPair::rampDown(int curr_speed) {
	for(int speed = curr_speed; speed > MIN_SPEED; speed--) {
		analogWrite(enable_a, speed);
		analogWrite(enable_b, speed);
	}
}

static void MotorPair::setMotorASpeed(int speed) {
	if (speed > 0) {
		digitalWrite(input1, LOW);
		digitalWrite(input2, HIGH);
	} else {
		digitalWrite(input1, HIGH);
		digitalWrite(input2, LOW);
	}

	// Continually Turn Motors
	analogWrite(enable_a, fabs(speed));
}

static void MotorPair::setMotorBSpeed(int speed) {

}

static void MotorPair::moveForwards() {
	digitalWrite(input1, LOW);
	digitalWrite(input2, HIGH);
	digitalWrite(input3, LOW);
	digitalWrite(input4, HIGH);

	rampUp(MAX_SPEED);

	analogWrite(enable_a, MAX_SPEED);
	analogWrite(enable_b, MAX_SPEED);
	delay(1000);

	rampDown(MAX_SPEED);
	stop();
}

static void MotorPair::moveBackwards() {
	digitalWrite(input1, HIGH);
	digitalWrite(input2, LOW);
	digitalWrite(input3, HIGH);
	digitalWrite(input4, LOW);

	rampUp(MAX_SPEED);

	analogWrite(enable_a, MAX_SPEED);
	analogWrite(enable_b, MAX_SPEED);
	delay(1000);

	rampDown(MAX_SPEED);
	stop();
}

static void MotorPair::turnLeft() {
	digitalWrite(input1, LOW);
	digitalWrite(input2, HIGH);
	digitalWrite(input3, HIGH);
	digitalWrite(input4, LOW);

	rampUp(MAX_SPEED/2);

	analogWrite(enable_a, MAX_SPEED/2);
	analogWrite(enable_b, MAX_SPEED/2);
	delay(500);

	rampDown(MAX_SPEED/2);
	stop();
}

static void MotorPair::turnRight() {
	digitalWrite(input1, HIGH);
	digitalWrite(input2, LOW);
	digitalWrite(input3, LOW);
	digitalWrite(input4, HIGH);

	rampUp(MAX_SPEED/2);

	analogWrite(enable_a, MAX_SPEED/2);
	analogWrite(enable_b, MAX_SPEED/2);
	delay(500);

	rampDown(MAX_SPEED/2);
	stop();
}
