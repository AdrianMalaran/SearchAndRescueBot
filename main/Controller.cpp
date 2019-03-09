/*
mag_heading = wrap360(v[0] + yaw_offset); //yaw = v[0], correct for magnetic North

// heading error and PID steering, "point_bearing" is the setpoint

 error = heading_error(point_bearing, mag_heading);  //wrap to +/- 180 degrees

 //error is positive if current_heading > bearing (compass direction)
 //positive bias acts to reduce left motor speed, so bear left

 bias = (kp*error)/10;  //Kp in tenths (Kp=40;  Ki, Kd not necessary in water)

 // the motor routines internally limit the argument to {-255, 255}

 set_m1_speed(motorSpeed-bias); //left motor
 set_m2_speed(motorSpeed+bias); //right motor

 */
#include "MotorPair.h"
#include <PID_v1.h>
#include <Arduino.h>

class Controller {
public:
    Controller () {
        // PID pid(&input, &output, &set_point, 2, 5, 1, DIRECT);
        // m_pid = pid;
        // //Specify the links and initial tuning parameters
        // m_pid.SetMode(AUTOMATIC);
    };

    void DriveStraight(double desired_heading, double current_heading) {
        input = current_heading;
        set_point = desired_heading;

        // m_pid.Compute();

        double error = desired_heading - current_heading;

        double bias = (Kp * error)/10;

        int nominal_speed = 220;
        MotorPair::setMotorASpeed(nominal_speed - bias);
        MotorPair::setMotorBSpeed(nominal_speed + bias);
    }

private:
    // PID m_pid;
    double set_point;
    double input;
    double output;
    // Gain Parameters
    double Kp;
    double Ki;
};

 // void loop()
 // {
 //   Input = analogRead(0);
 //   myPID.Compute();
 //   analogWrite(3,Output);
 // }
