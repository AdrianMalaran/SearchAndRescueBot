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
#define encoderApin1 19
#define encoderApin2 18
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

// imu
Imu imu_sensor = Imu();

// color(0xC0, 0x00, 22, 23)
Color color_front(SDA1pin, SCL1pin, COLOR_INTEGRATIONTIME_154MS, COLOR_GAIN_1X),
      color_down(SDA2pin, SCL2pin, COLOR_INTEGRATIONTIME_154MS, COLOR_GAIN_1X);

// ultrasonic(trigPin, echoPin)
Ultrasonic ultrasonic_front(ultrasonicFrontTrig, ultrasonicFrontEcho), ultrasonic_right(ultrasonicRightTrig, ultrasonicRightEcho),
                              ultrasonic_left(ultrasonicLeftTrig, ultrasonicLeftEcho), ultrasonic_back(ultrasonicBackTrig, ultrasonicBackEcho);

// motors
MotorPair motor_pair(imu_sensor);

// fan
Fan fan;

// encoders
Encoder encoderA(encoderApin1, encoderApin2);
Encoder encoderB(encoderBpin1, encoderBpin2);

// Controller controller(encoderA, encoderB);
Controller controller;

// Encoders

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

    delay(2000);

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
    Tests test;
    test.RunAllTests();

    Main main_engine(motor_pair, imu_sensor, color_front, color_down,
                     ultrasonic_front, ultrasonic_right, ultrasonic_left,
                     ultrasonic_back, controller, encoderA, encoderB);

    // Coord current_position = main_engine.getGlobalPosition(Pose(Coord(-2, -2), NORTH));
    // printCoord(current_position);
                     // testRightTurn(255, 223);
                     // testLeftTurn(230, 200);
                     // testLeftTurn(200, 200);
    main_engine.run();
    // main_engine.moveForwardSetDistance(60.0, NORTH);
    // main_engine.turnLeft(WEST); //
    // main_engine.turnLeft(SOUTH); //
    // main_engine.moveForwardSetDistance(60.0, SOUTH);
    // main_engine.turnLeft(EAST); //
    // main_engine.turnLeft(NORTH); //
    // main_engine.turnRight(EAST);

    // main_engine.turnRight(EAST);
    // main_engine.turnRight(SOUTH);
    // main_engine.moveForwardSetDistance(90.0);
    // main_engine.turnRight(WEST);
    // main_engine.turnRight(NORTH);
    // main_engine.turnRight();

    // motor_pair.setupMotorPair();
    // testMoveForward(180, 160);
    // testLeftTurn(255, 223);
    // testRightTurn(255, 255);
    // main_engine.moveForwardOneBlock(60.0);
    // main_engine.turnLeft();
    // motor_pair.stop();

    // test

    /* Max PWM for both motors to provide the same torque:
        MAX: Motor A: 255, Motor B: 223
        MIN: Motor A: 180, MOTOR B: 155
        MIN: Motor A: 160, Motor B: 130
        MIN: Motor A: 135, Motor B: 120

        Right Turns are garbage
        Left Turns are pretty good at 255, 223 speed
        Right Turns
    */

    // Orientation orientation;
    // Queue<Instruction> ins;
    // ins.push(MOVE_FORWARD);
    // ins.push(ROTATE_LEFT);
    // ins.push(MOVE_FORWARD);
    // main_engine.executeInstructions(ins, orientation);
}

int counter = 0;
/***************
*     LOOP     *
****************/
void loop() {
    if (counter == 100) {
        // Serial.print(encoderA.read()); Serial.print(" ");
        // Serial.println(encoderA.read());
        counter = 0;
    }

    counter ++;
    // Serial.println("Getting Possible Landmarks");
    // Coord our_pos = Coord(2,3);
    // main_engine.getPossibleLandmarks(main_engine.m_global_map, Pose(our_pos, NORTH));
    // // printMap(main_engine.m_global_map);
    //
    // for (int i = 0; i < GLOBAL_ROW; i ++) {
    //     for (int j = 0; j < GLOBAL_COL; j++) {
    //         Serial.print(main_engine.m_global_map[i][j].land_mark_spot); Serial.print(" ");
    //     }
    //     Serial.println("");
    //
    // }
    // Serial.println("");
    // Serial.println("");


    // Serial.print("Front: "); Serial.print(ultrasonic_front.getDistance());
    // Serial.print(" Back: "); Serial.print(ultrasonic_back.getDistance());
    // Serial.print(" Left: "); Serial.print(ultrasonic_left.getDistance());
    // Serial.print(" Right: "); Serial.println(ultrasonic_right.getDistance());
}
