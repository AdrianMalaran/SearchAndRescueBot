#include <Arduino.h>

#include "PathPlanning.h"
#include "Controller.h"

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
    static RunAllTests() {
        // Tests::TestPathPlanning();
        // Tests::TestTrajectoryGeneration();
        Tests::TestController();
    }

    static Stack<Coord> TestPathPlanning(int grid[][GLOBAL_COL], Coord start, Coord finish) {
        printStack(PathPlanning::AStarSearch(grid, start, finish));
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

    static void TestTrajectoryGeneration() {
        // Testing trajectory generation
        Queue<Instruction> ins;

        Orientation start = WEST;
        Orientation finish = SOUTH;

        PathPlanning::addReorientation(ins, start, finish);

        // Make into a function
        printOrientation(start);
        Serial.print(" -> ");
        printOrientation(finish);
        Serial.print(": ");
        while (!ins.empty()) {
            printInstruction(ins.front());
            ins.pop();

            if (ins.size())
                Serial.print(" -> ");
        }

        Serial.println("");

        Stack<Coord> path;
        // path.push(Coord(3,4));
        // path.push(Coord(2,4));
        // path.push(Coord(2,3));
        // path.push(Coord(2,2));
        // path.push(Coord(2,1));
        // path.push(Coord(3,1));
        // path.push(Coord(3,0));
        // path.push(Coord(2,0));
        // path.push(Coord(1,0));
        // path.push(Coord(0,0));

        path.push(Coord(3,3));
        path.push(Coord(3,4));
        path.push(Coord(4,4));
        path.push(Coord(4,3));
        path.push(Coord(3,3));

        ins = PathPlanning::generateTrajectories(path, start, finish);

        Serial.println("New Path:");
        while (!ins.empty()) {
            printInstruction(ins.front());
            ins.pop();

            if (ins.size())
                Serial.print(" -> ");
        }
        Serial.println("");
    }

    static void TestController() {
        Controller::DriveStraight(10, 359);
        Controller::DriveStraight(0, 359);
        Controller::DriveStraight(350, 359);
        Controller::DriveStraight(180, 270);

    }
};
