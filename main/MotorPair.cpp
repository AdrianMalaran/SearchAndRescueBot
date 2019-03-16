#include "MotorPair.h"
#include <Arduino.h>

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
	// get the latest heading
	m_orientation = m_imu_sensor.getEuler().x();

	// determine the desired orientation of imu with wrapper
	m_orientation-=90;
	if (m_orientation < 0) m_orientation+=270;

	// orientate motors for left turn
  	digitalWrite(stand_by, HIGH);
	digitalWrite(input1, LOW);
	digitalWrite(input2, HIGH);
	digitalWrite(input3, HIGH);
	digitalWrite(input4, LOW);

	analogWrite(enable_a, MAX_SPEED_A/2);
	analogWrite(enable_b, MAX_SPEED_B/2);
	
	while (m_imu_sensor.getEuler().x() != m_orientation) {}

	stop();
}

void MotorPair::turnRight() {
	// get the latest heading
	m_orientation = m_imu_sensor.getEuler().x();

	// determine the desired orientation of imu with wrapper
	m_orientation+=90;
	if (m_orientation > 360) m_orientation-=270;

  	digitalWrite(stand_by, HIGH);
  
	digitalWrite(input1, HIGH);
	digitalWrite(input2, LOW);
	digitalWrite(input3, LOW);
	digitalWrite(input4, HIGH);

	analogWrite(enable_a, MAX_SPEED_A/2);
	analogWrite(enable_b, MAX_SPEED_B/2);

	while (m_imu_sensor.getEuler().x() != m_orientation) {}

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
