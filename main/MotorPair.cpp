#include "MotorPair.h"
#include <Arduino.h>

// Code assumes:
// Motor A is left
// Motor B is right
//
// Motor speed is 0-255

const int MAX_SPEED_A = 255;
const int MIN_SPEED_A = 0;

const int MAX_SPEED_B = 255;
const int MIN_SPEED_B = 0;

// MotorPair motor_pair(9, 3, 4, 10, 5, 6);

// Motor A
const int enable_a = 9;
const int input1 = 29;
const int input2 = 28;

// Motor B
const int enable_b = 10;
const int input3 = 31;
const int input4 = 30;

// standby pin
const int stand_by = 34;

MotorPair::MotorPair() {}

MotorPair::MotorPair(Imu imu_sensor) {
	m_imu_sensor = imu_sensor;
}

/* Max PWM for both motors to provide the same torque:
	MAX: Motor A: 255, Motor B: 223
	MIN: Motor A: 180, MOTOR B: 155
	MIN: Motor A: 160, Motor B: 130
	MIN: Motor A: 135, Motor B: 120

	Right Turns are pretty garbage
	Left Turns are pretty good at 255, 223 speed
*/

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

	digitalWrite(stand_by, HIGH);
}

static void MotorPair::stop() {
	analogWrite(enable_a, MIN_SPEED_A);
	analogWrite(enable_b, MIN_SPEED_B);
}

static void MotorPair::standby() {
  digitalWrite(stand_by, LOW);
}

static void MotorPair::setMotorAPWM(int PWM) {
	if (PWM > 0) {
		digitalWrite(input1, LOW);
		digitalWrite(input2, HIGH);
	} else {
		digitalWrite(input1, HIGH);
		digitalWrite(input2, LOW);
	}
	// Continually Turn Motors
	analogWrite(enable_a, min(fabs(PWM), MAX_SPEED_A));
}

static void MotorPair::setMotorBPWM(int PWM) {
	if (PWM > 0) {
		digitalWrite(input3, LOW);
		digitalWrite(input4, HIGH);
	} else {
		digitalWrite(input3, HIGH);
		digitalWrite(input4, LOW);
	}
	// Serial.println("Setting Motor Speed");
	// Continually Turn Motors
	analogWrite(enable_b, min(fabs(PWM), MAX_SPEED_B));
}
