#include "Main.h"

void drawMap(MapLocation global_map[][GLOBAL_COL]) {
    for (int i = 0; i < GLOBAL_ROW; i++) {
        for(int j = 0; j < GLOBAL_COL; j++) {
            Serial.print(global_map[i][j].block_type);
            Serial.print(" ");
        }
    }
}

Main::Main() {
    init();
}

void Main::init() {
    // (1) Initialize map;
    // (2) Test Path Planning
    // (3) Calibrate sensors

    // Set start coord
    m_start_coord = Coord(4, 5);

    // Initialize Start Map
    for (int i = 0; i < GLOBAL_ROW; i ++)
        for (int j = 0; j < GLOBAL_COL; j++)
            global_map[i][j].block_type = U;

    global_map[m_start_coord.row][m_start_coord.col].block_type = P;
}

// This function will be run in the loop() function of the arduino
void Main::run() {

    if (tasks.empty()) {
        returnToStart();
        // Stop Program
    } else {
        // All tasks are not completed yet
        completeNextTask();
    }
}

void Main::completeNextTask() {
    // Retrieve current task
    Task current_task = tasks.front();

    switch (current_task) {
        case EXTINGUISH_FIRE:
            Serial.println("TASK: Extinguishing Fire");
            // extinguishFire();
            break;
        case FIND_FOOD:
            Serial.println("TASK: Finding Survivor");
            // findFood();
            break;
        case FIND_GROUP_OF_PEOPLE:
            Serial.println("TASK: Finding Group of People");
            break;
        case DELIVER_FOOD:
            Serial.println("TASK: Delivering Food");
            break;
        case FIND_SURVIVOR:
            Serial.println("TASK: Finding Survivor");
            break;
        default:
            Serial.println("UNKNOWN TASK");
            break;
    }

    // Mark task as completed
    tasks.pop();
}

void Main::returnToStart() {
    // Get current location
    // Travel To Start Location();
    // stopToProgram();
}

/***********************
* PERIPHERAL FUNCTIONS *
************************/

void Main::mapAdjacentBlocks(MapLocation *global_map[][GLOBAL_COL], Coord current_loc) {
    // Use motor encoders to measure distance ??
    // Detect adjacent blocks that are undiscovered

    // Explore all adjacent blocks
    Coord adjacent_blocks[4];
    adjacent_blocks[0] = Coord(current_loc.row - 1, current_loc.col);
    adjacent_blocks[0] = Coord(current_loc.row + 1, current_loc.col);
    adjacent_blocks[0] = Coord(current_loc.row, current_loc.col - 1);
    adjacent_blocks[0] = Coord(current_loc.row, current_loc.col + 1);

    for (int i = 0; i < 4; i++) {
        int row = adjacent_blocks[i].row;
        int col = adjacent_blocks[i].col;
        MapLocation map_location = *global_map[row][col];
        if (isValid(row, col) && map_location.block_type == U) {
            // Use color sensor to detect terrain
        }
    }
}

// TODO: Test
bool Main::isUnexplored(MapLocation global_map[][GLOBAL_COL], Coord coord) {
    // If coord is invalid, don't explore
    if (isValid(coord.row, coord.col) && global_map[coord.row][coord.col].block_type == U)
        return true;

    return false;
}

void Main::mapTerrainOfBlockInFront() {
    /* Questions we want to answer:
        How do we detect that we're at the edge of one block ? (Use ultrasonic sensors )
    */

    /*
        Move forward until boundary of current block is met
        Take colour sensor reading to determine correct terrain of block infront
        Map this block
        Move robot back to center of the block
    */
}

bool Main::isValid(int row, int col) {
  return row >= 0 && col >= 0 && row < GLOBAL_ROW && col < GLOBAL_COL;
}

/***********************
*    TASK FUNCTIONS    *
************************/

void Main::findFood(MapLocation global_map[][GLOBAL_COL], Coord current_loc) {
    // Check to see if there are mapped sand blocks
    //     if there are no mapped sand blocks, go explore

    // Search for closest sand block
    // travel to sand block
    // inspect sand block to see if a magnet is detected

    //TODO: Need to know if the robot can just stay in place and the IMU will detect
    // that reading

    //TODO: if we get close enough to sand block boundary to detect a magnet,
    // then we should automatically mark that spot as the food location

    Coord closest_sand_block = getClosestSandBlock(global_map, current_loc);
}

Coord Main::getClosestSandBlock(MapLocation global_map[][GLOBAL_COL], Coord current_loc) {
    int min_distance = INT_MAX;
    Coord closest_sand_block = Coord(-1, -1); // Error coordinate

    for (int i = 0; i < GLOBAL_ROW; i++) {
        for (int j = 0; j < GLOBAL_COL; j++) {
            // Check to see if the block is a SAND block and that
            // it has not already been searched
            if (global_map[i][j].block_type == S && global_map[i][j].searched == false) {
                int distance = getManhattanDistance(Coord(i,j), current_loc);

                if (distance < min_distance) {
                    min_distance = distance;
                    closest_sand_block = Coord(i,j);
                }
            }
        }
    }

    return closest_sand_block;
}

int Main::getManhattanDistance(Coord c1, Coord c2) {
    return abs(c1.row - c2.row) + abs(c1.col - c2.col);
}

// TODO: Rigourous testing needed for this algorithm
void Main::travelToBlock(MapLocation map[][GLOBAL_COL], Coord current_loc, Coord dest,
                         Orientation start_ori, Orientation finish_ori) {
    // Path Plan from current_location to dest
    Stack<Coord> shortest_path =
        PathPlanning::AStarSearch(map, current_loc, dest);
    Serial.println("Calculated Shortest Path");

    Queue<Instruction> maneuver_instructions =
        PathPlanning::generateTrajectories(shortest_path, start_ori, finish_ori);

    Serial.println("Calculated Trajectories");
    PathPlanning::executeInstructions(maneuver_instructions);
    Serial.println("Executed Trajectories");

    //updateLocation(); possibly update location ?
}
