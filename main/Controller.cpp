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

 #include "Controller.h"

Controller::Controller(Encoder encA, Encoder encB) {
    m_encoderA = encA;
    m_encoderB = encB;
}

void Controller::DriveStraight(double desired_heading, double current_heading, double nominal_speed) {

    if (current_heading == 0 && current_heading == 0) {
        Serial.println("IMU not working, Don't Drive");
        // IMU not working, Don't Drive
        MotorPair::stop();
        return;
    }

    double Kp = 20;

    // input = current_heading;
    // set_point = desired_heading;
    // pid.Compute();

    double error = desired_heading - current_heading;

    if (error > 180)
        error -= 360;
    else if (error < -180)
        error += 360;

    double bias = (Kp * error)/10;

    Serial.print("Desired: "); Serial.print(desired_heading);
    Serial.print(", Current: "); Serial.print(current_heading);
    Serial.print(", Error: "); Serial.print(error);
    Serial.print(", bias: "); Serial.print(bias); Serial.print(" | ");
    Serial.print(" SpeedA: "); Serial.print(nominal_speed - bias);
    Serial.print(" SpeedB: "); Serial.println(nominal_speed + bias);

    // TODO: Separate actuation from PWN calculation
    MotorPair::setMotorASpeed(nominal_speed + bias);
    MotorPair::setMotorBSpeed(nominal_speed - bias);
}

void Controller::SpeedControl(double desired_speed, double current_speed) {

    // Get current speed
    // Calculate error between between desired speed and current speed
    m_encoderA.read();


    double error = desired_speed - current_speed;

    // if (error) is negative - we are travelling faster than what we desire -> decrease speed
    // if (error) is positive - we are travelling slower than what we desire -> increase speed

    //TODO: Add
    double speed_Kp = 30;

    double speed_bias = (Kp * error)/10;

    // Calculate PWM
}
