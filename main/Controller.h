#ifndef Controller_h
#define Controller_h

#include <Arduino.h>

#include "MotorPair.h"
#include "Encoder/Encoder.h"
#include <TimerThree.h>

const int encoderAPin1 = 2;
const int encoderAPin2 = 3;
const int encoderBPin1 = 18;
const int encoderBPin2 = 19;

class Controller {
    public:
        Controller() {};
        Controller(Encoder encA, Encoder encB);

        void driveStraightController();
        void turnController(double desired_heading, double current_heading, double nominal_pwm, bool turn_left);
        void driveStraight(double desired_heading, double current_heading, double nominal_speed = 180);
        void rotateLeft(int speed);
        void speedControl(double desired_speed, double current_speed, double nominal_pwm);
        double mapSpeedToPWM();
    private:
        double set_point;
        double input;
        double output;

        double Kp;
        double Ki;

        Encoder m_encoderA;
        Encoder m_encoderB;

        double total_error;
        double last_time;
        int counter = 0;
};

#endif
