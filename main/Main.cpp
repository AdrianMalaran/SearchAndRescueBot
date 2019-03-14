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

Main::Main(MotorPair motor_pair, Imu imu_sensor, Color color_front, Color color_down,
            Ultrasonic ultrasonic_front, Ultrasonic ultrasonic_right, Ultrasonic ultrasonic_left,
            Ultrasonic ultrasonic_back) {
    m_motor_pair = motor_pair;
    m_imu_sensor = imu_sensor;
    m_color_front = color_front;
    m_color_down = color_down;
    m_ultrasonic_front = ultrasonic_front;
    m_ultrasonic_right = ultrasonic_right;
    m_ultrasonic_left = ultrasonic_left;
    m_ultrasonic_back = ultrasonic_back;
    init();
}

void Main::init() {
    // (0) Initialize sensors and actuators
    // (1) Initialize map
    // (2) Test Path Planning
    // (3) Calibrate sensors

    // Calibrate Imu
    // m_imu_sensor.calibrate();

    delay(1000);

    // Set motor pins
    m_motor_pair.setupMotorPair();

    // Set flame pins
    Flame::setupFlame();

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
            // Assume there is some sort of Path Planning (Adrian) that gets us within
            // the flame sensors range of the candle.
            Serial.println("TASK: Extinguishing Fire");
            extinguishFire();
            break;
        case FIND_FOOD:
            Serial.println("TASK: Finding Food");
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

Coord Main::getGlobalPosition() {
    double col1 = m_ultrasonic_left.getDistance() / 30.3;
    double col2 = m_ultrasonic_right.getDistance() / 30.3;
    double row1 = m_ultrasonic_front.getDistance() / 30.3;
    double row2 = m_ultrasonic_back.getDistance() / 30.3;

    return Coord(round((row1 + row2)/2), round((col1 + col2)/2));
}

void Main::mapAdjacentBlocks(MapLocation (&global_map)[GLOBAL_ROW][GLOBAL_COL], Pose start_pose) {
    // Use motor encoders to measure distance ??
    // Detect adjacent blocks that are undiscovered

    // Explore all adjacent blocks
    Pose current_pose = start_pose;
    Pose desired_pose;

    Pose adjacent_blocks[4];
    adjacent_blocks[0] = Pose(Coord(start_pose.coord.row - 1, start_pose.coord.col), NORTH);
    adjacent_blocks[1] = Pose(Coord(start_pose.coord.row + 1, start_pose.coord.col), SOUTH);
    adjacent_blocks[2] = Pose(Coord(start_pose.coord.row, start_pose.coord.col - 1), WEST);
    adjacent_blocks[3] = Pose(Coord(start_pose.coord.row, start_pose.coord.col + 1), EAST);

    for (int i = 0; i < 4; i++) {
        int row = adjacent_blocks[i].coord.row;
        int col = adjacent_blocks[i].coord.col;
        MapLocation map_location = global_map[row][col];
        if (isValid(row, col) && map_location.block_type == U) {
            desired_pose = Pose(current_pose.coord, adjacent_blocks[i].orientation);
            travelToBlock(global_map, current_pose, desired_pose);

            // Try with ultrasonic - If this doesn't work, include encoder control
            double start_distance = m_ultrasonic_front.getDistance();
            while(m_ultrasonic_front.getDistance() > start_distance - 7) { 
                // Move forwards
                Controller::DriveStraight(m_imu_sensor.getEuler().x(), m_imu_sensor.getEuler().x(), 180);
            }
            m_motor_pair.stop();

            map_location.block_type = m_color_down.getTerrainColor();

            while(m_ultrasonic_front.getDistance() < start_distance) { 
                // Move backwards
                Controller::DriveStraight(m_imu_sensor.getEuler().x(), m_imu_sensor.getEuler().x(), -180);
            }
            m_motor_pair.stop();

            current_pose = desired_pose;
        }
        travelToBlock(global_map, current_pose, start_pose);
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

void Main::findFood(MapLocation global_map[][GLOBAL_COL], Pose current_pose) {
    // Check to see if there are mapped sand blocks
    //     if there are no mapped sand blocks, go explore

    // Search for closest sand block
    // travel to sand block
    // inspect sand block to see if a magnet is detected

    //TODO: Need to know if the robot can just stay in place and the IMU will detect
    // that reading

    //TODO: if we get close enough to sand block boundary to detect a magnet,
    // then we should automatically mark that spot as the food location

    Coord closest_sand_block = getClosestSandBlock(global_map, current_pose.coord);
    if (closest_sand_block.row == -1 && closest_sand_block.col == -1) {
        Serial.println("All sandblocks mapped have been searched, explore for more");
        // Go explore unmapped blocks
    } else {
        Serial.println("Travelling to closest sand block");
        travelToBlock(global_map, current_pose, Pose(closest_sand_block, DONTCARE));
    }
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
void Main::travelToBlock(MapLocation map[][GLOBAL_COL], Pose start_pose, Pose finish_pose) {
    // Path Plan from current_location to dest
    Stack<Coord> shortest_path =
        PathPlanning::AStarSearch(map, start_pose.coord, finish_pose.coord);
    Serial.println("Calculated Shortest Path");

    // Handle Invalid Path errors
    if (shortest_path.size() == 1 &&
        shortest_path.top().row == -1 &&
        shortest_path.top().col == -1) {
        Serial.println("No Path Found");
        return;
    }

    Queue<Instruction> maneuver_instructions =
        PathPlanning::generateTrajectories(shortest_path, start_pose.orientation, finish_pose.orientation);
    Serial.println("Calculated Trajectories");

    PathPlanning::executeInstructions(maneuver_instructions);
    Serial.println("Executed Trajectories");

    //updateLocation(); possibly update location ?
}

void Main::extinguishFire() {
    m_motor_pair.extinguishFireTurn();
}
