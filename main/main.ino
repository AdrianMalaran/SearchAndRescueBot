#include "Tests.h"
#include "Ultrasonic.h"
#include "Color.h"
#include "Imu.h"
#include "Main.h"

#include <Wire.h>

#define LEDPin 13
#define SDA1pin 22
#define SCL1pin 23
#define SDA2pin 24
#define SCL2pin 25

#define BNO055_SAMPLERATE_DELAY_MS 100

Imu imu_sensor = Imu();

// color(0xC0, 0x00, 22, 23)
Color color_front(SDA1pin, SCL1pin, COLOR_INTEGRATIONTIME_154MS, COLOR_GAIN_1X), color_down(SDA2pin, SCL2pin, COLOR_INTEGRATIONTIME_154MS, COLOR_GAIN_1X);

// ultrasonic(trigPin, echoPin)
Ultrasonic ultrasonic_front(49,48), ultrasonic_right(51,50), ultrasonic_left(47,46), ultrasonic_back(53,52);

// MotorPair declaration
MotorPair motor_pair(imu_sensor);

// controller
double input, output, set_point, Kp = 30;
// PID pid(&input, &output, &set_point, 2, 5, 1, DIRECT);

// void setupController() {
//     pid.SetMode(AUTOMATIC);
// }

Tests test;

Main main_engine(motor_pair, imu_sensor, color_front, color_down, ultrasonic_front, ultrasonic_right, ultrasonic_left, ultrasonic_back);

double m_desired_heading;

/**************
*    SETUP    *
***************/
void setup() {
    Serial.begin(9600);
    Serial.println("Hello World!");

    Serial.println("Running Tests");
    test.RunAllTests();
}

/***************
*     LOOP     *
****************/
void loop() {
    // Controller::DriveStraight(imu_sensor.getEuler().x(), imu_sensor.getEuler().x(), 200);

    Serial.print("euler X: "); Serial.println(imu_sensor.getEuler().x());
    Serial.print("mag X: "); Serial.print(imu_sensor.getMag().x());
    Serial.print(" mag Y: "); Serial.print(imu_sensor.getMag().y());
    Serial.print(" mag Z: "); Serial.println(imu_sensor.getMag().z());

    delay(1000);

    /*
    Serial.print("Terrain: "); Serial.println(color_down.getTerrainColor());
    */

    /*
    Serial.print("Structure: "); Serial.println(color_front.getStructureColor());
    */

    /*
    MapLocation map_location = MapLocation(UNKNOWN);
    Serial.print("landmark ahead: "); Serial.println(main_engine.isLandmarkAhead(map_location, Pose(Coord(2,2), NORTH)));
    */

    /*
    Serial.print("global position: "); printCoord(main_engine.getGlobalPosition(Pose(Coord(2,2), NORTH))); Serial.println("");
    */
}
