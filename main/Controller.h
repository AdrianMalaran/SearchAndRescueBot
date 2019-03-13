#ifndef Controller_h
#define Controller_h

#include <Arduino.h>

#include "MotorPair.h"
#include "PID_v1.h"

class Controller {
    public:
        Controller ();

        static void DriveStraight(double desired_heading, double current_heading, double nominal_speed = 180);
    private:
        double set_point;
        double input;
        double output;

        double Kp;
        double Ki;
};

#endif
