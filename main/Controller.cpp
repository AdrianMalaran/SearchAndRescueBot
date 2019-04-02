 #include "Controller.h"

Controller::Controller(Encoder encA, Encoder encB) {
    m_encoderA = encA;
    m_encoderB = encB;
}


//TODO: Rename to headingController => outputs a PWM for both motors
void Controller::headingControl(double &pwm_a, double &pwm_b, double desired_heading, double current_heading, double nominal_pwm) {
    // if (current_heading == 0 && desired_heading == 0) {
    //     Serial.println("IMU not working, Don't Drive");
    //     // IMU not working, Don't Drive
    //     MotorPair::stop();
    //     return;
    // }

    double Kp = 15;
    double Ki = 0.0;

    if (nominal_pwm < 0)
        Kp = 15;

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
    // MotorPair::setMotorAPWM(nominal_speed + bias);
    // MotorPair::setMotorBPWM(nominal_speed - bias);
    pwm_a = nominal_pwm + bias;
    pwm_b = nominal_pwm - bias;
}

//TODO: Rename to headingController => outputs a PWM for both motors
void Controller::turnLeftController(double desired_heading, double current_heading, double nominal_pwm) {
    /* Most Promising Parameters to date
        double Kp = 0.9;
        double Ki = 0.0001;
        MotorPair::setMotorAPWM(polarity*10*(bias - 4));
        MotorPair::setMotorBPWM(polarity*-10 * (bias - 4));

        double Kp = 0.5;
        double Ki = 0.0001;
        MotorPair::setMotorAPWM(polarity*10*(bias - 15));
        MotorPair::setMotorBPWM(polarity*-10 * (bias + 4));
        MotorPair::setMotorAPWM(polarity*10*(bias - 15));
        MotorPair::setMotorBPWM(polarity*-10 * (bias + 4));
    */
    double Kp = 0.9;
    double Ki = 0.0001;

    double error = desired_heading - current_heading;
    double current_time = micros();
    total_error += error * (current_time - last_time)/1000000;

    if (error > 180)
        error -= 360;
    else if (error < -180)
        error += 360;

    double bias = (Kp * error) + (Ki * total_error);
    last_time = current_time;

    // MotorPair::setMotorAPWM(10*(bias + 8)*1.2);
    // MotorPair::setMotorBPWM(-10 * (bias - 4)*0.7);

    // MotorPair::setMotorAPWM(10*(bias) * 1.1);
    // MotorPair::setMotorBPWM(-10 * (bias));

    MotorPair::setMotorAPWM(10*(bias + 8)*1.2);
    MotorPair::setMotorBPWM(-10 * (bias - 4)*0.8);
}

//TODO: Rename to headingController => outputs a PWM for both motors
void Controller::turnRightController(double desired_heading, double current_heading, double nominal_pwm) {
    double Kp = 1.0;
    double Ki = 0.0001;

    double error = desired_heading - current_heading;
    double current_time = micros();
    total_error += error * (current_time - last_time)/1000000;

    if (error > 180)
        error -= 360;
    else if (error < -180)
        error += 360;

    double bias = (Kp * error) + (Ki * total_error);
    last_time = current_time;

    // MotorPair::setMotorAPWM(10*(bias + 10));
    // MotorPair::setMotorBPWM(-10 * (bias + 3));

    MotorPair::setMotorAPWM(10*(bias + 13)*0.8);
    MotorPair::setMotorBPWM(-10 * (bias + 3)*0.9);
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
void Controller::driveStraightController(double desired_heading, double current_heading, double nominal_pwm) {
    // This will take in a desired heading and ask for the desired PWM using heading correction
    // Get PWM calculated for motor a and motor b
    double motor_A_pwm;
    double motor_B_pwm;

    headingControl(motor_A_pwm, motor_B_pwm, desired_heading, current_heading, nominal_pwm);

    // Add speed control here

    MotorPair::setMotorAPWM(motor_A_pwm);
    MotorPair::setMotorBPWM(motor_B_pwm);
}

// TODO: Need a means to measure speed mapped to a PWM
// This requires us to characterize the motors to map the min_PWM to supply to the motors
// Speed control of one motor
// TODO: Reference pwm can be omitted
void Controller::speedControl(double &output_pwm,  double desired_speed, double current_speed, double reference_pwm) {
    // Get current speed
    // Calculate error between between desired speed and current speed
    // m_encoderA.read();

    // I know that I desire a position of (something) number of ticks
    // if I'm lagging behind, apply a bias that multiplies that position control
    double error = desired_speed - current_speed;

    // if (error) is negative - we are travelling faster than what we desire -> decrease speed
    // if (error) is positive - we are travelling slower than what we desire -> increase speed
    double pwm_Kp = 5;
    double bias = (pwm_Kp * error);

    // Calculate PWM
    // MotorPair::setMotorAPWM(nominal_pwm + bias);

    output_pwm = reference_pwm + bias;
}
