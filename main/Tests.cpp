#include "Tests.h"

// TODO: Remove all tests for game day
// Store them in a seperate location
Tests::Tests () {};

void Tests::RunAllTests() {
    // TestPathPlanning();
    // TestTrajectoryGeneration();
    // TestController();
    // TestTravel();
    // TestExplore();
}
/*
static Stack<Coord> TestPathPlanning(MapLocation grid[][GLOBAL_COL], Coord start, Coord finish) {
    printStack(PathPlanning::findShortestPath(grid, start, finish));
}
static Stack<Coord> TestPathPlanning() {
    MapLocation MP(P);
    MapLocation MS(S);
    MapLocation MW(W);
    MapLocation MG(G);
    MapLocation MU(U);

    MapLocation testGrid[GLOBAL_ROW][GLOBAL_COL] =
    {
    //    0, 1, 2, 3, 4, 5
        { MP, MP, MP, MP, MP, MP}, // 0
        { MP, MP, MP, MP, MP, MP}, // 1
        { MP, MP, MP, MP, MP, MP}, // 2
        { MP, MP, MP, MP, MP, MP}, // 3
        { MP, MP, MP, MP, MP, MP}, // 4
        { MP, MP, MP, MP, MP, MP}  // 5
    };

    // TestPathPlanning(grid, start, finish)
    TestPathPlanning(testGrid, Coord(4,3), Coord(2,0)); // NORTH WEST
    TestPathPlanning(testGrid, Coord(0,0), Coord(5,5)); // SOUTH EAST
    TestPathPlanning(testGrid, Coord(0,0), Coord(0,5)); // EAST
    TestPathPlanning(testGrid, Coord(0,5), Coord(0,0)); // WEST
    TestPathPlanning(testGrid, Coord(5,5), Coord(0,0)); // WEST

    MapLocation testGrid2[GLOBAL_ROW][GLOBAL_COL] =
    {
    //    0, 1, 2, 3, 4, 5
        { MS, MW, MS, MS, MS, MS}, // 0
        { MS, MW, MW, MW, MW, MS}, // 1
        { MS, MS, MS, MS, MS, MS}, // 2
        { MW, MS, MW, MS, MW, MS}, // 3
        { MS, MW, MS, MW, MS, MS}, // 4
        { MS, MS, MS, MS, MS, MS}  // 5
    };
    // Immediate Path is Blocked
    TestPathPlanning(testGrid2, Coord(0,2), Coord(0,0));
    TestPathPlanning(testGrid2, Coord(0,2), Coord(5,0));
    TestPathPlanning(testGrid2, Coord(4,2), Coord(0,0));
    TestPathPlanning(testGrid2, Coord(4,0), Coord(0,2));

    // At Destination
    TestPathPlanning(testGrid2, Coord(0,0), Coord(0,0));

    MapLocation testGrid3[GLOBAL_ROW][GLOBAL_COL] =
    {
    //    0, 1, 2, 3, 4, 5
        { MP, MU, MG, MG, MG, MU}, // 0
        { MG, MG, MU, MU, MU, MG}, // 1
        { MG, MG, MG, MG, MG, MG}, // 2
        { MG, MG, MG, MG, MG, MG}, // 3
        { MG, MG, MG, MG, MU, MU}, // 4
        { MG, MG, MG, MG, MU, MG}  // 5
    };
    // No Path
    TestPathPlanning(testGrid3, Coord(0,2), Coord(5,5));
}

static void Tests::TestTrajectoryGeneration() {
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
    // path.push(Coord(3,4));
    // path.push(Coord(4,4));
    // path.push(Coord(4,3));
    // path.push(Coord(3,3));

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

static void Tests::TestController() {
    Controller::DriveStraight(10, 359);
    Controller::DriveStraight(0, 359);
    Controller::DriveStraight(350, 359);
    Controller::DriveStraight(180, 270);

}

Main m_main_engine;

void Tests::TestTravel() {
    MapLocation MP(PARTICLE);
    MapLocation MS(SAND);
    MapLocation MW(WATER);
    MapLocation MG(GRAVEL);
    MapLocation MU(UNKNOWN);

    MapLocation map[GLOBAL_ROW][GLOBAL_COL] =
    {
    //    0, 1, 2, 3, 4, 5
        { MS, MW, MS, MS, MS, MS}, // MS
        { MS, MW, MW, MW, MW, MS}, // 1
        { MS, MS, MS, MS, MS, MS}, // 2
        { MW, MS, MW, MS, MW, MS}, // 3
        { MS, MW, MS, MW, MS, MS}, // 4
        { MS, MS, MS, MS, MS, MS}  // 5
    };

    Coord start = Coord(4,5);
    Coord finish = Coord(4,5);
    Orientation start_ori = NORTH;
    Orientation finish_ori = SOUTH;

    Pose start_pose (start, start_ori);
    Pose finish_pose (finish, finish_ori);

    Serial.println("Testing Travel to block");

    Main m_main_engine;
    m_main_engine.travelToBlock(map, start_pose, finish_pose);
}

void Tests::TestFindFood() {
    //TEST find closest sand block
    //TEST get manhattan distance
}

void Tests::TestExplore() {
    Serial.println("Running TestExplore:");

    MapLocation MP(PARTICLE);
    MapLocation O(SAND);
    MapLocation X(WATER);
    MapLocation MG(GRAVEL);
    MapLocation U(UNKNOWN);
    MapLocation I(PARTICLE);
    I.land_mark_spot = true;
    I.landmark = PEOPLE;
    I.block_type = PARTICLE;

    MapLocation map[GLOBAL_ROW][GLOBAL_COL] =
    {
        { O, I, O, O, O, O}, // O
        { O, O, X, O, O, X}, // 1
        { O, O, U, X, X, O}, // 2
        { O, O, X, X, O, O}, // 3
        { O, O, O, O, X, O}, // 4
        { O, O, X, O, O, O}  // 5
    //    0, 1, 2, 3, 4, 5
    };

    Coord start_loc(3,5);

    Main m_main_engine;

    // Test hasUnknownNeighbors()
    Orientation finish_ori;
    MapLocation unknownlocation;
    MapLocation interestlocation;
    interestlocation.block_type = PARTICLE;
    interestlocation.land_mark_spot = true;
    interestlocation.landmark = SURVIVOR;
    bool res = m_main_engine.hasMatchingNeighbors(map, unknownlocation, start_loc, finish_ori);
    Serial.print("Has Unknown neighbors?: "); Serial.println(res);

    Coord block = m_main_engine.findClosestBlockToInterest(map, interestlocation, start_loc, finish_ori);
    Serial.print("Closest: ("); Serial.print(block.row); Serial.print(","); Serial.print(block.col); Serial.print(") Direction: ");
    Serial.print(finish_ori); Serial.println("");
    printMap(map);
}
*/
