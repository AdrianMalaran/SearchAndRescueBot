#include "PathPlanning.cpp"
#include "Ultrasonic.h"
#include "Flame.h"
#include "MotorPair.h"
//#include "sensors/ColorSoft.h"
#include "Color.h"
#include "Imu.h"
#include "utilities/imumaths.h"

#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>

#define LEDPin 13

#define SDApin 22
#define SCLpin 23
#define BNO055_SAMPLERATE_DELAY_MS (100)

Imu imu_sensor = Imu();

// color1(0xC0, 0x00, 22, 23)
//ColorSoft color1(COLOR_INTEGRATIONTIME_154MS, COLOR_GAIN_1X, SDApin, SCLpin);

// color2(0xC0, 0x00)
Color color2(COLOR_INTEGRATIONTIME_154MS, COLOR_GAIN_1X);

// ultrasonic(trigPin, outPin)
Ultrasonic ultrasonic(52,53);

// flame(digitalPin, analogPin)
Flame flame(27, 9);

// motor_pair(enableA, input1-A, input2-A, enableB, input3-B, input4-B)
MotorPair motor_pair(9, 3, 4, 10, 5, 6);

/* Print Functions */
void printCoord(Coord coord) {
    Serial.print("("); Serial.print(coord.row); Serial.print(",");
    Serial.print(coord.col); Serial.print(") ");
}

void printStack(Stack<Coord> stack) {
    Serial.print("Path Size: ");
    Serial.println(stack.size());

    Serial.print("(START)");
    while (!stack.empty()) {
        Serial.print("(");
        Serial.print(stack.top().row);
        Serial.print(",");
        Serial.print(stack.top().col);
        Serial.print(") ");
        if(stack.size() > 1) Serial.print(" -> ");
        stack.pop();
    }
    Serial.println("(FINISH)");
}

// TODO: Remove all tests for game day
// Store them in a seperate location
class Tests {
public:
    static Stack<Coord> TestPathPlanning(int grid[][GLOBAL_COL], Coord start, Coord finish) {
        printStack(AStarSearch(grid, start, finish));
    }
    static Stack<Coord> TestPathPlanning() {
        int testGrid[GLOBAL_ROW][GLOBAL_COL] =
        {
        //    0, 1, 2, 3, 4, 5
            { 0, 0, 0, 0, 0, 0}, // 0
            { 0, 0, 0, 0, 0, 0}, // 1
            { 0, 0, 0, 0, 0, 0}, // 2
            { 0, 0, 0, 0, 0, 0}, // 3
            { 0, 0, 0, 0, 0, 0}, // 4
            { 0, 0, 0, 0, 0, 0}  // 5
        };

        // TestPathPlanning(grid, start, finish)
        TestPathPlanning(testGrid, Coord(4,3), Coord(2,0)); // NORTH WEST
        TestPathPlanning(testGrid, Coord(0,0), Coord(5,5)); // SOUTH EAST
        TestPathPlanning(testGrid, Coord(0,0), Coord(0,5)); // EAST
        TestPathPlanning(testGrid, Coord(0,5), Coord(0,0)); // WEST
        TestPathPlanning(testGrid, Coord(5,5), Coord(0,0)); // WEST

        int testGrid2[GLOBAL_ROW][GLOBAL_COL] =
        {
        //    0, 1, 2, 3, 4, 5
            { 0, 1, 0, 0, 0, 0}, // 0
            { 0, 1, 1, 1, 1, 0}, // 1
            { 0, 0, 0, 0, 0, 0}, // 2
            { 1, 0, 1, 0, 1, 0}, // 3
            { 0, 1, 0, 1, 0, 0}, // 4
            { 0, 0, 0, 0, 0, 0}  // 5
        };
        // Immediate Path is Blocked
        TestPathPlanning(testGrid2, Coord(0,2), Coord(0,0));
        TestPathPlanning(testGrid2, Coord(0,2), Coord(5,0));
        TestPathPlanning(testGrid2, Coord(4,2), Coord(0,0));
        TestPathPlanning(testGrid2, Coord(4,0), Coord(0,2));

        // At Destination
        TestPathPlanning(testGrid2, Coord(0,0), Coord(0,0));

        int testGrid3[GLOBAL_ROW][GLOBAL_COL] =
        {
        //    0, 1, 2, 3, 4, 5
            { 0, 1, 0, 0, 0, 1}, // 0
            { 0, 0, 1, 1, 1, 0}, // 1
            { 0, 0, 0, 0, 0, 0}, // 2
            { 0, 0, 0, 0, 0, 0}, // 3
            { 0, 0, 0, 0, 1, 1}, // 4
            { 0, 0, 0, 0, 1, 0}  // 5
        };
        // No Path
        TestPathPlanning(testGrid3, Coord(0,2), Coord(5,5));
    }
};

void stopProgram() {
    set_sleep_mode(SLEEP_MODE_PWR_DOWN);
    cli();  // Disable interrupts
    sleep_mode();
}

// Motors
void demo() {
    // move forwards until the device is within 15cm
    while(ultrasonic.getDistance() > 15) {
        motor_pair.moveForwards();
    }

    // turn the robot right
    motor_pair.turnRight();

    // move forwards until the device detects fire
    while(flame.getFireMagnitude() < 25) {
        motor_pair.moveForwards();
    }

    // move backwards 1 duration
    motor_pair.moveBackwards();

    // stop the device
    motor_pair.stop();

    stopProgram();
}

int BAUD_RATE = 9600;

void setup() {
    Serial.begin(BAUD_RATE);

    Serial.println("Running Tests");
    Tests::TestPathPlanning();

    // LED pin for testing
    pinMode(LEDPin, OUTPUT);

    if(!imu_sensor.begin()) {
        /* There was a problem detecting the BNO055 ... check your connections */
        Serial.print("Ooops, no imu_sensor detected ... Check your wiring or I2C ADDR!");
        while(1);
    }

    int8_t temp = imu_sensor.getTemp();
    Serial.print("Current Temperature: "); Serial.print(temp); Serial.println(" C");

    imu_sensor.setExtCrystalUse(true);

    uint8_t system = 0, gyro, accel, mag = 0;
    while(system == 0) {
        imu_sensor.getCalibration(&system, &gyro, &accel, &mag);
        Serial.print("CALIBRATION: Sys="); Serial.print(system, DEC); Serial.print(" Gyro="); Serial.print(gyro, DEC); 
        Serial.print(" Accel="); Serial.print(accel, DEC); Serial.print(" Mag="); Serial.println(mag, DEC);
        delay(100);
    }
}

void loop() {
/*
  uint16_t r, g, b, c;
  color1.getRawData(&r, &g, &b, &c);
  Serial.print("R: "); Serial.print(r, DEC); Serial.print(" ");
  Serial.print("G: "); Serial.print(g, DEC); Serial.print(" ");
  Serial.print("B: "); Serial.print(b, DEC); Serial.println(" ");

  uint16_t r2, g2, b2, c2;
  color2.getRawData(&r2, &g2, &b2, &c2);
  Serial.print("R2: "); Serial.print(r2, DEC); Serial.print(" ");
  Serial.print("G2: "); Serial.print(g2, DEC); Serial.print(" ");
  Serial.print("B2: "); Serial.print(b2, DEC); Serial.println(" ");
*/

  imu::Vector<3> euler = imu_sensor.getEuler();
  Serial.print("X: "); Serial.print(euler.x());
  Serial.print(" Y: "); Serial.print(euler.y());
  Serial.print(" Z: "); Serial.println(euler.z());
}
