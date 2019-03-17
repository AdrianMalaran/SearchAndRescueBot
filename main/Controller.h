#ifndef Controller_h
#define Controller_h

#include <Arduino.h>

#include "MotorPair.h"
#include "PID_v1.h"
#include "Encoder.h"

class Controller {
    public:
        Controller() {};
        Controller(Encoder encA, Encoder encB);

        void driveStraight(double desired_heading, double current_heading, double nominal_speed = 180);
        void rotateLeft(int speed);
        void SpeedControl(double desired_speed, double current_speed);
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
