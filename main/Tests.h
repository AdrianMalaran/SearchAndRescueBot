#ifndef Test_h
#define Test_h

#include <Arduino.h>

#include "Main.h"

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
