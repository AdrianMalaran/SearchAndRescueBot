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
Color color_front(SDA1pin, SCL1pin, COLOR_INTEGRATIONTIME_154MS, COLOR_GAIN_1X), color_down(SDA2pin, SCL2pin, COLOR_INTEGRATIONTIME_154MS, COLOR_GAIN_1X);

// ultrasonic(trigPin, echoPin)
Ultrasonic ultrasonic_front(ultrasonicFrontTrig, ultrasonicFrontEcho), ultrasonic_right(ultrasonicRightTrig, ultrasonicRightEcho), 
                              ultrasonic_left(ultrasonicLeftTrig, ultrasonicLeftEcho), ultrasonic_back(ultrasonicBackTrig, ultrasonicBackEcho);

// MotorPair declaration
MotorPair motor_pair(imu_sensor);

// controller
double input, output, set_point, Kp = 30;
// PID pid(&input, &output, &set_point, 2, 5, 1, DIRECT);

// encoders
// Encoder encoderA(encoderApin1, encoderApin2);
// Encoder encoderB(encoderBpin1, encoderBpin2);
// Controller controller(encoderA, encoderB);
Controller controller;

Tests test;

double m_desired_heading;

// Global Variable
int m_motor_speed_A = 0;
int m_motor_speed_B = 0;
int last_encA_value = 0;
int last_encB_value = 0;

//Testing
int current_encA_value = 0;
int current_encB_value = 0;


const int timer_micro_seconds = 1000000;

double calculateSpeed(int current_encoder_value, int last_encoder_value, int period) {
    double distance = 3.14*8;
    double distancePerTick = distance / 341.2; // 341.2 counts/revolution

    //TODO: check if need to wrap
    //TODO: Test Encoder class input
    // Serial.print("Distance: "); Serial.println(distancePerTick * (current_encoder_value - last_encoder_value));
    // double speed = distancePerTick * (current_encoder_value - last_encoder_value) / (period/1000000.0); // Convert from microseconds to seconds
    double speed = (current_encoder_value - last_encoder_value) / (period/1000000.0); // Convert from microseconds to seconds
    return speed;
}

void updateActualSpeed() {
    // int current_encA_value = encoderA.read();
    // m_motor_speed_A = (current_encA_value - last_encA_value)/timer_micro_seconds;
    m_motor_speed_A = calculateSpeed(current_encA_value, last_encA_value, timer_micro_seconds);
    last_encA_value = current_encA_value;

    // int current_encB_value = encoderB.read();
    m_motor_speed_B = calculateSpeed(current_encB_value, last_encB_value, timer_micro_seconds);
    last_encB_value = current_encB_value;

    // Serial.print("MotorA Speed: "); Serial.print(m_motor_speed_A);
    // Serial.print(" MotorB Speed: "); Serial.print(m_motor_speed_B);
    // Serial.println("");
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
    main_engine.run();

    // main_engine.moveForwardOneBlock();

    // main_engine.turnLeft();
    // delay(900);
    // motor_pair.stop();

    // imu_sensor.calibrate();
    //
    // motor_pair.setupMotorPair();
    // motor_pair.turnLeft();

    // Timers
    // Timer1.initialize(timer_micro_seconds); // Microseconds
    // Timer1.attachInterrupt(updateActualSpeed);
    // Timer1.setPeriod(timer_micro_seconds);
}

/***************
*     LOOP     *
****************/
void loop() {
    // Serial.print("Ultrasonic Front: "); Serial.println(ultrasonic_front.getDistance());
    // current_encA_value ++;
    // current_encB_value ++;
    // motor_pair.setMotorBSpeed(160);
    // Serial.print("Encoder: "); Serial.println(encoderB.read());
    // Serial.print(current_encA_value); Serial.print(" "); Serial.println(current_encB_value);
    // Serial.println("gangang");
    // Serial.println(m_current_speed);
/*
    //Serial.println(ultrasonic.getDistance());

    uint16_t r, g, b, c;
    color1.getRawData(&r, &g, &b, &c);
    Serial.print("R: "); Serial.print(r, DEC); Serial.print(" ");
    Serial.print("G: "); Serial.print(g, DEC); Serial.print(" ");
    Serial.print("B: "); Serial.print(b, DEC); Serial.println(" ");

    Serial.println(color1.getTerrainColor());

    uint16_t r2, g2, b2, c2;
    color2.getRawData(&r2, &g2, &b2, &c2);
    Serial.print("R2: "); Serial.print(r2, DEC); Serial.print(" ");
    Serial.print("G2: "); Serial.print(g2, DEC); Serial.print(" ");
    Serial.print("B2: "); Serial.print(b2, DEC); Serial.println(" ");

    // imu::Vector<3> euler = imu_sensor.getMag(); // Magnet
    // Serial.print("Yaw: "); Serial.print(euler.x());
    // Serial.print(" Pitch: "); Serial.print(euler.y());
    // Serial.print(" Roll: "); Serial.println(euler.z());
    imu::Vector<3> euler = imu_sensor.getEuler();
    double desired_heading = m_desired_heading;
    double current_heading = euler.x();

    //Controller::DriveStraight(desired_heading, current_heading);
    // MotorPair::moveForwards();
    // Serial.println("Driving");
    motor_pair.turnLeft();
*/

    // Controller::DriveStraight(m_desired_heading, imu_sensor.getEuler().x(), 140);

    //Serial.print("Structure: "); Serial.println(color_front.getStructureColor());
    //Serial.println(Flame::getFireMagnitude());
    //if (Flame::getFireMagnitude() > 1) {
    //    Fan::on();
    //    delay(5000);
    //}
    //Fan::off();

    // digitalWrite(33,HIGH);
    // delay(3000);
    // digitalWrite(33,LOW);
    // delay(3000);
    /*
    MapLocation map_location = MapLocation(UNKNOWN);
    Serial.print("landmark ahead: "); Serial.println(main_engine.isLandmarkAhead(map_location, Pose(Coord(2,2), NORTH)));
    */

    /*
    main_engine.extinguishFire();
    */

    /*
    Serial.print("global position: "); printCoord(main_engine.getGlobalPosition(Pose(Coord(2,2), NORTH))); Serial.println("");
    */
}
