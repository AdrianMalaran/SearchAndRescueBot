#include "Tests.cpp"
#include "Ultrasonic.h"
#include "Flame.h"
// #include "MotorPair.h"
#include "Color.h"
#include "Imu.h"
#include "utilities/imumaths.h"
#include "Adafruit_Sensor.h"

#include <Wire.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>


#define LEDPin 13

#define SDA1pin 22
#define SCL1pin 23
#define SDA2pin 24
#define SCL2pin 25

#define BNO055_SAMPLERATE_DELAY_MS (100)

Imu imu_sensor = Imu();

// color1(0xC0, 0x00, 22, 23)
Color color1(SDA1pin, SCL1pin, COLOR_INTEGRATIONTIME_154MS, COLOR_GAIN_1X);

// color2(0xC0, 0x00)
Color color2(SDA2pin, SCL2pin, COLOR_INTEGRATIONTIME_154MS, COLOR_GAIN_1X);

// ultrasonic(trigPin, echoPin)
Ultrasonic ultrasonic(52,53);

MotorPair motor_pair(imu_sensor);

// controller
double input, output, set_point, Kp = 30;
PID pid(&input, &output, &set_point, 2, 5, 1, DIRECT);

void stopProgram() {
    set_sleep_mode(SLEEP_MODE_PWR_DOWN);
    cli();  // Disable interrupts
    sleep_mode();
}

void setupController() {
    pid.SetMode(AUTOMATIC);
}

double m_desired_heading;

/**************
*    SETUP    *
***************/
void setup() {
    Serial.begin(9600);

    Serial.println("Running Tests");
    Tests::RunAllTests();

    // LED pin for testing
    pinMode(LEDPin, OUTPUT);

    imu_sensor.calibrate();

    delay(1000);

    Flame::setupFlame();

    motor_pair.setupMotorPair();
}

/***************
*     LOOP     *
****************/
void loop() {

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
}
