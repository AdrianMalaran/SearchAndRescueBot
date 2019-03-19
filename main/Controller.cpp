 #include "Controller.h"

Controller::Controller(Encoder encA, Encoder encB) {
    m_encoderA = encA;
    m_encoderB = encB;
}


//TODO: Rename to headingController => outputs a PWM for both motors
void Controller::driveStraight(double desired_heading, double current_heading, double nominal_speed) {
    // if (current_heading == 0 && desired_heading == 0) {
    //     Serial.println("IMU not working, Don't Drive");
    //     // IMU not working, Don't Drive
    //     MotorPair::stop();
    //     return;
    // }
    double Kp = 15;
    double Ki = 0.0;
    // input = current_heading;
    // set_point = desired_heading;
    // pid.Compute();

    double error = desired_heading - current_heading;
    double current_time = micros();
    total_error += error * (current_time - last_time)/1000000;

    if (error > 180)
        error -= 360;
    else if (error < -180)
        error += 360;

    // counter++;
    // if (counter == 1000) {
    //     Serial.print(", Error: "); Serial.print(error);
    //     counter = 0;
    // }

    double bias = (Kp * error) + (Ki * total_error);
    last_time = current_time;

    // Serial.print("Desired: "); Serial.print(desired_heading);
    // Serial.print(", Current: "); Serial.print(current_heading);
    // Serial.print(", Error: "); Serial.print(error);
    // Serial.print(", bias: "); Serial.print(bias); Serial.print(" | ");
    // Serial.print(" SpeedA: "); Serial.print(nominal_speed - bias);
    // Serial.print(" SpeedB: "); Serial.println(nominal_speed + bias);

    // TODO: Separate actuation from PWN calculation
    MotorPair::setMotorAPWM(nominal_speed + bias);
    MotorPair::setMotorBPWM(nominal_speed - bias);
}

//TODO: Rename to headingController => outputs a PWM for both motors
void Controller::turnController(double desired_heading, double current_heading, double nominal_pwm, bool turn_left) {
    double Kp = 0.5;
    double Ki = 0.0;

    double error = desired_heading - current_heading;
    double current_time = micros();
    total_error += error * (current_time - last_time)/1000000;

    if (error > 180)
        error -= 360;
    else if (error < -180)
        error += 360;

    double bias = (Kp * error) + (Ki * total_error);
    last_time = current_time;

    //TODO: Change to different polarity

    double polarity = 1;
    if (turn_left)
        polarity = -1;

    MotorPair::setMotorAPWM(polarity * (nominal_pwm + bias));
    MotorPair::setMotorBPWM(-1 * polarity * (nominal_pwm + bias));
}

double mapMotorASpeedToPWM(double speed) {
    double min_speed;
    double max_speed;
    double min_pwm;
    double max_pwm;

    return map(speed, min_speed, max_speed, min_pwm, max_pwm);
}

double mapMotorBSpeedToPWM(double speed) {
    double min_speed;
    double max_speed;
    double min_pwm;
    double max_pwm;

    return map(speed, min_speed, max_speed, min_pwm, max_pwm);
}

//OVERALL CONTROLLER
void Controller::driveStraightController() {
    // This will take in a desired heading and ask for the desired PWM using heading correction
    //
}

// TODO: Need a means to measure speed mapped to a PWM
// This requires us to characterize the motors to map the min_PWM to supply to the motors
void Controller::speedControl(double desired_speed, double current_speed, double nominal_pwm) {

    // Get current speed
    // Calculate error between between desired speed and current speed
    // m_encoderA.read();

    // I know that I desire a position of (something) number of ticks
    // if I'm lagging behind, apply a bias that multiplies that position control

    double error = desired_speed - current_speed;

    // if (error) is negative - we are travelling faster than what we desire -> decrease speed
    // if (error) is positive - we are travelling slower than what we desire -> increase speed

    double pwm_Kp = 30;
    double pwm_bias = (Kp * error)/10;

    // Calculate PWM
    MotorPair::setMotorAPWM(nominal_pwm + pwm_bias);
}
