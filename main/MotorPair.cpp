#include "MotorPair.h"
#include <Arduino.h>

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
// Code assumes:
// Motor A is right
// Motor B is left
//
// Motor speed is 0-255

const int MAX_SPEED_A = 255;
const int MIN_SPEED_A = 0;

const int MAX_SPEED_B = 255;
const int MIN_SPEED_B = 0;

// MotorPair motor_pair(9, 3, 4, 10, 5, 6);

// Motor A
const int enable_a = 9;
const int input1 = 28;
const int input2 = 29;

// Motor B
const int enable_b = 10;
const int input3 = 30;
const int input4 = 31;

// standby pin
const int stand_by = 34;

MotorPair::MotorPair() {}

MotorPair::MotorPair(Imu imu_sensor) {
	m_imu_sensor = imu_sensor;
}

void MotorPair::setupMotorPair() {
	m_orientation = m_imu_sensor.getEuler().x();

	// Set motor A pins
	pinMode(enable_a, OUTPUT);
	pinMode(input1, OUTPUT);
	pinMode(input2, OUTPUT);

	// Set motor B pins
	pinMode(enable_b, OUTPUT);
	pinMode(input3, OUTPUT);
	pinMode(input4, OUTPUT);

  	// Set standby pin
  	pinMode(stand_by, OUTPUT);
}

static void MotorPair::stop() {
	analogWrite(enable_a, MIN_SPEED_A);
	analogWrite(enable_b, MIN_SPEED_B);
}

static void MotorPair::standby() {
  digitalWrite(stand_by, LOW);
}

static void MotorPair::rampUp(int set_speed) {
	for(int speed = MIN_SPEED_A; speed < set_speed; speed++) {
		analogWrite(enable_a, speed);
		analogWrite(enable_b, speed);
	}
}

// Possibly Remove
// static void MotorPair::rampUp(int set_speed) {
// 	for(int speed = MIN_SPEED; speed < set_speed; speed++) {
// 		analogWrite(enable_a, speed);
// 		analogWrite(enable_b, speed);
// 	}
// }
//
// static void MotorPair::rampDown(int curr_speed) {
// 	for(int speed = curr_speed; speed > MIN_SPEED; speed--) {
// 		analogWrite(enable_a, speed);
// 		analogWrite(enable_b, speed);
// 	}
// }

static void MotorPair::setMotorASpeed(int speed) {
	if (speed > 0) {
		digitalWrite(input1, LOW);
		digitalWrite(input2, HIGH);
	} else {
		digitalWrite(input1, HIGH);
		digitalWrite(input2, LOW);
	}

	// Continually Turn Motors
	analogWrite(enable_a, min(fabs(speed), MAX_SPEED_A));
}

static void MotorPair::setMotorBSpeed(int speed) {
	if (speed > 0) {
		digitalWrite(input3, LOW);
		digitalWrite(input4, HIGH);
	} else {
		digitalWrite(input3, HIGH);
		digitalWrite(input4, LOW);
	}

	// Continually Turn Motors
	analogWrite(enable_b, min(fabs(speed), MAX_SPEED_B));
}


void MotorPair::turnLeft() {
	// determine the desired orientation of imu with wrapper
	if (m_orientation - 90 < 0)
		m_orientation+=270;
	else
		m_orientation-=90;

	// orientate motors for left turn
  	digitalWrite(stand_by, HIGH);
	digitalWrite(input1, LOW);
	digitalWrite(input2, HIGH);
	digitalWrite(input3, HIGH);
	digitalWrite(input4, LOW);

	// rampUp(MAX_SPEED/2);

	analogWrite(enable_a, MAX_SPEED_A/2);
	analogWrite(enable_b, MAX_SPEED_B/2);
	
	while (m_imu_sensor.getEuler().x() != m_orientation) {}

	// rampDown(MAX_SPEED/2);
	stop();
}

void MotorPair::turnRight() {
	// determine the desired orientation of imu with wrapper
	double desired_orientaion;
	if (m_orientation + 90 > 360)
		desired_orientaion = m_orientation - 270;
	else
		desired_orientaion = m_orientation + 90;

  	digitalWrite(stand_by, HIGH);
  
	digitalWrite(input1, HIGH);
	digitalWrite(input2, LOW);
	digitalWrite(input3, LOW);
	digitalWrite(input4, HIGH);

	// rampUp(MAX_SPEED/2);

	analogWrite(enable_a, MAX_SPEED_A/2);
	analogWrite(enable_b, MAX_SPEED_B/2);

	while (m_imu_sensor.getEuler().x() != desired_orientaion) {}

	// rampDown(MAX_SPEED/2);
	stop();
}

bool MotorPair::extinguishFireTurn() {
	double start_orientation = m_imu_sensor.getEuler().x();
	bool fire_extinguished = false;

	// orientate motors for left turn
  	digitalWrite(stand_by, HIGH);
	digitalWrite(input1, LOW);
	digitalWrite(input2, HIGH);
	digitalWrite(input3, HIGH);
	digitalWrite(input4, LOW);

	analogWrite(enable_a, MAX_SPEED_A/3);
	analogWrite(enable_b, MAX_SPEED_B/3);
	
	while (m_imu_sensor.getEuler().x() != m_orientation) {
		if (Flame::getFireMagnitude() > 5) {
			stop();
			while (Flame::getFireMagnitude() > 5) {
				Fan::on();
			}
			Fan::off();
			
			if (fabs(start_orientation - m_orientation) < 180) {
				digitalWrite(input1, HIGH);
				digitalWrite(input2, LOW);
				digitalWrite(input3, LOW);
				digitalWrite(input4, HIGH);
			}

			analogWrite(enable_a, MAX_SPEED_A/2);
			analogWrite(enable_b, MAX_SPEED_B/2);

			while (m_imu_sensor.getEuler().x() != start_orientation) {}

			fire_extinguished = true;
		}
		if (fire_extinguished) break;
	}

	stop();

	return fire_extinguished;
}
