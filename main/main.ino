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
/*
TEST PRIORITY:
- [o] leftturn control
- [x] mapBlockLandmarkInFront()
- [x] findFire()
- [] moveToPossibleFireLocation
- [] moveForwardSetDistance w/ Ultrasonic feedback
*/

/**************
*    SETUP    *
***************/
void setup() {
    Serial.begin(9600);
    Serial.println("Running Tests");
    // Tests test;
    // test.RunAllTests();

    Main main_engine(motor_pair, imu_sensor, color_front, color_down,
                     ultrasonic_front, ultrasonic_right, ultrasonic_left,
                     ultrasonic_back, controller, encoderA, encoderB);

    // main_engine.run();

    // main_engine.findFire();

    //// WORKING COMBINATIONS - Motion ////
    // main_engine.turnLeft(WEST);
    //main_engine.moveBackwardSetDistance(5.0, WEST);
    // main_engine.turnLeft(SOUTH);
    // main_engine.moveBackwardSetDistance(5.0, SOUTH);
    // main_engine.turnLeft(EAST);
    // main_engine.moveBackwardSetDistance(5.0, EAST);
    // main_engine.turnLeft(NORTH);
    // main_engine.moveBackwardSetDistance(4.0, NORTH);

    // main_engine.turnLeft(SOUTH);
    // main_engine.moveForwardSetDistance(60.0, SOUTH);
    // main_engine.turnLeft(EAST);
    // main_engine.turnLeft(NORTH);
    // main_engine.turnRight(EAST);

    // main_engine.turnRight(EAST);
    // main_engine.turnRight(SOUTH);
    // main_engine.moveForwardSetDistance(90.0);
    // main_engine.turnRight(WEST);
    // main_engine.turnRight(NORTH);
    // main_engine.turnRight();

    // testMoveForward(180, 160);
    // testLeftTurn(255, 223);
    // testRightTurn(255, 223);
    // main_engine.moveForwardOneBlock(60.0);
    // main_engine.turnLeft();
    // motor_pair.stop();

    //// EXECUTE INSTRUCTIONS TEST ////
    // Orientation finish_ori = NORTH;
    // Queue<Instruction> ins;
    // ins.push(ROTATE_LEFT);
    // ins.push(ROTATE_RIGHT);
    // ins.push(MOVE_FORWARD);
    // ins.push(ROTATE_RIGHT);
    // main_engine.executeInstructions(ins, finish_ori);
    // main_engine.mapBlockTerrainInFront(main_engine.m_global_map, Pose(Coord(4,3), EAST), 0.0, Coord(4,4));

    //// TEST LANDMARK DETECTION ////
    // Pose our_pose(Coord(4,4), WEST);
    // main_engine.getPossibleLandmarks(main_engine.m_global_map, our_pose);
    // for(int i = 0; i < 6; i++) {
    //     for(int j = 0; j < 6; j++) {
    //         Serial.print(main_engine.m_global_map[i][j].land_mark_spot); Serial.print(" ");
    //     }
    //     Serial.println("");
    // }
    // Serial.println("");
    //
    // for(int i = 0; i < 6; i++) {
    //     for(int j = 0; j < 6; j++) {
    //         Serial.print(main_engine.m_global_map[i][j].searched); Serial.print(" ");
    //     }
    //     Serial.println("");
    // }

    // Coord our_coord(Coord(4,4));
    // Pose to_map(Coord(3,4), NORTH);
    // main_engine.mapBlockLandmarkInFront(main_engine.m_global_map, to_map, our_coord);

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
}
