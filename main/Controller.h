#ifndef Controller_h
#define Controller_h

#include <Arduino.h>

#include "MotorPair.h"
#include "Encoder.h"

class Controller {
    public:
        Controller() {};
        Controller(Encoder encA, Encoder encB);

        void turnLeftController(double desired_heading, double current_heading, double nominal_pwm, bool turn_left);
        void turnRightController(double desired_heading, double current_heading, double nominal_pwm, bool turn_left);
        void driveStraightController(double desired_heading, double current_heading, double nominal_pwm = 180);

        void headingControl(double &pwm_a, double &pwm_b, double desired_heading, double current_heading, double nominal_pwm);
        void speedControl(double &output_pwm,  double desired_speed, double current_speed, double reference_pwm);

        double mapSpeedToPWM();
    private:
        Encoder m_encoderA;
        Encoder m_encoderB;

        double total_error;
        double last_time;
        int counter = 0;

        double m_motor_speed_A;
        double m_motor_speed_B;
};

#endif
