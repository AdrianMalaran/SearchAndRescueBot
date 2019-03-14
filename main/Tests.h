#ifndef Test_h
#define Test_h

#include <Arduino.h>

#include "Main.h"

/* Print Functions */
inline void printCoord(Coord coord) {
    Serial.print("("); Serial.print(coord.row); Serial.print(",");
    Serial.print(coord.col); Serial.print(") ");
}

inline void printOrientation(Orientation ori) {
    if (ori == NORTH)
        Serial.print("NORTH");
    else if (ori == SOUTH)
        Serial.print("SOUTH");
    else if (ori == EAST)
        Serial.print("EAST");
    else if (ori == WEST)
        Serial.print("WEST");
}

inline void printInstruction(Instruction ins) {
    if (ins == MOVE_FORWARD)
        Serial.print("MOVE FORWARD");
    else if (ins == MOVE_BACKWARD)
        Serial.print("MOVE BACK");
    else if (ins == ROTATE_RIGHT)
        Serial.print("TURN RIGHT");
    else if (ins == ROTATE_LEFT)
        Serial.print("TURN LEFT");
}

inline void printStack(Stack<Coord> stack) {
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
    Tests ();

    void RunAllTests();

    static Stack<Coord> TestPathPlanning(MapLocation grid[][GLOBAL_COL], Coord start, Coord finish);
    static Stack<Coord> TestPathPlanning();

    static void TestTrajectoryGeneration();

    static void TestController();

    void TestTravel();

    void TestFindFood();

    void TestExplore();

};

#endif
