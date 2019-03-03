#include "PathPlanning.cpp"
#include "Ultrasonic.h"
#include "Flame.h"
#include "MotorPair.h"

#include <avr/interrupt.h>
#include <avr/sleep.h>

// ultrasonic(trigPin, outPin)
Ultrasonic ultrasonic(52,53);

// flame(digitalPin, analogPin)
Flame flame(27, 9);

// motorPair(enableA, input1-A, input2-A, enableB, input3-B, input4-B)
MotorPair motorPair(9, 3, 4, 10, 5, 6);

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

void demo() {
    // Move forwards until the device is within 10cm
    while(ultrasonic.getDistance() > 15) {
        motorPair.moveForwards();
    }

    // Turn the robot right
    motorPair.turnRight();

    // Move forwards until the device detects fire
    while(!flame.isFire()) {
        motorPair.moveForwards();
    }

    // Stop the device
    motorPair.stop();

    Serial.println("test");
    stopProgram();
}

int BAUD_RATE = 9600;

void setup() {
    Serial.begin(BAUD_RATE);

    Serial.println("Running Tests");
    Tests::TestPathPlanning();

    Serial.print("getDistance: ");
    Serial.println(ultrasonic.getDistance(), 1);

    Serial.print("isFire: ");
    Serial.print(flame.isFire());
    Serial.print(", getFireMagnitude: ");
    Serial.println(flame.getFireMagnitude());

    Serial.println("Initializing...");

    // LED pin for testing
    pinMode(13, OUTPUT);

}

void loop() {
   // motorPair.moveForwards();

    while(ultrasonic.getDistance() > 15) {};

    digitalWrite(13, HIGH);
    delay(1000);
    digitalWrite(13, LOW);

    while(flame.getFireMagnitude() < 75) {};

    digitalWrite(13, HIGH);
    delay(1000);
    digitalWrite(13, LOW);
}
