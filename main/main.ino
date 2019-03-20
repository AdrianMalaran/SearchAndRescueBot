#include "Tests.h"
#include "Ultrasonic.h"
#include "Color.h"
#include "Imu.h"
#include "Main.h"

#include <TimerOne.h>

#include <Wire.h>

// LED Pins
#define LEDPin 13

// Color Pins
#define SDA1pin 22
#define SCL1pin 23
#define SDA2pin 24
#define SCL2pin 25

// Encoder Pins
#define encoderApin1 18
#define encoderApin2 19
#define encoderBpin1 2
#define encoderBpin2 3

// Ultrasonic Pins
#define ultrasonicFrontTrig 49
#define ultrasonicFrontEcho 48
#define ultrasonicRightTrig 51
#define ultrasonicRightEcho 50
#define ultrasonicLeftTrig 47
#define ultrasonicLeftEcho 46
#define ultrasonicBackTrig 53
#define ultrasonicBackEcho 52


#define BNO055_SAMPLERATE_DELAY_MS 100

Imu imu_sensor = Imu();

// color(0xC0, 0x00, 22, 23)
Color color_front(SDA1pin, SCL1pin, COLOR_INTEGRATIONTIME_154MS, COLOR_GAIN_1X),
      color_down(SDA2pin, SCL2pin, COLOR_INTEGRATIONTIME_154MS, COLOR_GAIN_1X);

// ultrasonic(trigPin, echoPin)
Ultrasonic ultrasonic_front(ultrasonicFrontTrig, ultrasonicFrontEcho), ultrasonic_right(ultrasonicRightTrig, ultrasonicRightEcho),
                              ultrasonic_left(ultrasonicLeftTrig, ultrasonicLeftEcho), ultrasonic_back(ultrasonicBackTrig, ultrasonicBackEcho);

// MotorPair declaration
MotorPair motor_pair(imu_sensor);

// fan
Fan fan;

// controller
double input, output, set_point, Kp = 30;
// PID pid(&input, &output, &set_point, 2, 5, 1, DIRECT);

// encoders
// Encoder encoderA(encoderApin1, encoderApin2);
// Encoder encoderB(encoderBpin1, encoderBpin2);
// Controller controller(encoderA, encoderB);
Controller controller;

// /*
//     Controller Math:
//     1 Revolution = (16/21) * 1 Revolution = 0.7619 Revolutions of Wheel/1 revolution of Motor
//     Therefore, 1 revolution of motor = (0.7691)*(8*pi) = 19.1487 centimeters travelled
// */
// double calculateSpeed(long current_encoder_value, long last_encoder_value, long period) {
//     double wheel_circumference = 3.14*8;
//     double distancePerWheelTick = wheel_circumference / 341.2; // 341.2 counts/revolution of the wheel
//     double distancePerMotorTick = distancePerWheelTick * (16/21.0); // 1 Revolution of the motor = Gear Ratio = 16/21
//
//     // Serial.print("Distance: "); Serial.println(distancePerTick * (current_encoder_value - last_encoder_value));
//     // double speed = distancePerTick * (current_encoder_value - last_encoder_value) / (period/1000000.0); // Convert from microseconds to seconds
//     long double speed = distancePerMotorTick * ((current_encoder_value - last_encoder_value) / (period / 1000000.0)); // Convert from microseconds to seconds
//     return speed;
// }

// void updateActualSpeed() {
//     long current_encA_value = encoderA.read();
//     m_motor_speed_A = calculateSpeed(current_encA_value, last_encA_value, timer_micro_seconds);
//     last_encA_value = current_encA_value;
//
//     long current_encB_value = encoderB.read();
//     m_motor_speed_B = calculateSpeed(current_encB_value, last_encB_value, timer_micro_seconds);
//     last_encB_value = current_encB_value;
//
//     // Serial.print("Enc A: "); Serial.print(current_encA_value);
//     // Serial.print(" Enc B: "); Serial.print(current_encB_value);
//     // Serial.println("Updating Speed!");
//     Serial.print(" MA: "); Serial.print(m_motor_speed_A);
//     Serial.print(" MB: "); Serial.println(m_motor_speed_B);
// }

// void initializeTimers() {
//     // Timers
//     Timer1.initialize(timer_micro_seconds); // Microseconds
//     Timer1.attachInterrupt(updateActualSpeed);
// }

void testMoveForward(double PWM1, double PWM2) {
    motor_pair.setMotorAPWM(PWM1);
    motor_pair.setMotorBPWM(PWM2);

    delay(5000);

    motor_pair.stop();
}

void testLeftTurn(double PWM1, double PWM2) {
    motor_pair.setMotorAPWM(-1*PWM1);
    motor_pair.setMotorBPWM(PWM2);

    delay(5000);

    motor_pair.stop();
}

void testRightTurn(double PWM1, double PWM2) {
    motor_pair.setMotorAPWM(PWM1);
    motor_pair.setMotorBPWM(-1*PWM2);

    delay(5000);

    motor_pair.stop();
}

/**************
*    SETUP    *
***************/
void setup() {
    Serial.begin(9600);
    Serial.println("Running Tests");
    // Tests test;
    // test.RunAllTests();
    Main main_engine(motor_pair, imu_sensor, color_front, color_down, ultrasonic_front, ultrasonic_right, ultrasonic_left, ultrasonic_back, controller);

    //main_engine.extinguishFire();
}

/***************
*     LOOP     *
****************/
void loop() {

}
