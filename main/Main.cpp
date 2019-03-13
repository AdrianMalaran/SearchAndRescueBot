#include "Main.h"

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
}

void Main::Init() {
    // (1) Initialize map;
    // (2) Test Path Planning
    // (3) Calibrate sensors

    // Calibrate Imu
    m_imu_sensor.calibrate();

    delay(1000);

    m_motor_pair.setupMotorPair();

    Flame::setupFlame();

    // Set start coord
    start_coord = Coord(4, 5);

    // Initialize Start Map
    for (int i = 0; i < GLOBAL_ROW; i ++)
        for (int j = 0; j < GLOBAL_COL; j++)
            global_map[i][j] = U;

    global_map[start_coord.row][start_coord.col] = P;
}

// This function will be run in the loop() function of the arduino
void Main::Run() {

    if (tasks.empty()) {
        ReturnToStart();
        // Stop Program
    } else {
        // All tasks are not completed yet
        completeNextTask();
    }
}

void Main::completeNextTask() {
    // Retrieve current task
    TASK current_task = tasks.front();

    switch (current_task) {
        case EXTINGUISH_FIRE:
            // Assume there is some sort of Path Planning (Adrian) that gets us within
            // the flame sensors range of the candle.
            Serial.println("TASK: Extinguishing Fire");
            extinguishFire();
            break;
        case FIND_FOOD:
            Serial.println("TASK: Finding Survivor");
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

void Main::ReturnToStart() {
    // Get current location
    // Travel To Start Location();
    // stopToProgram();
}

/***********************
* PERIPHERAL FUNCTIONS *
************************/

static void Main::mapAdjacentBlocks(BLOCK_TYPE *global_map[][GLOBAL_COL]) {
    // Use motor encoders to measure distance ??
    // Detect adjacent blocks that are undiscovered

    // Check global map for any undiscovered blocks
    Coord current_location;

    // Explore all adjacent blocks
    Coord adjacent_blocks[4];
    adjacent_blocks[0] = Coord(current_location.row - 1, current_location.col);
    adjacent_blocks[0] = Coord(current_location.row + 1, current_location.col);
    adjacent_blocks[0] = Coord(current_location.row, current_location.col - 1);
    adjacent_blocks[0] = Coord(current_location.row, current_location.col + 1);

    for (int i = 0; i < 4; i++) {
        int row = adjacent_blocks[i].row;
        int col = adjacent_blocks[i].col;
        if (isValid(row, col) && *global_map[row][col] == U) {
            // Use color sensor to detect terrain
        }
    }
}

// TODO: Test
static bool Main::isUnexplored(BLOCK_TYPE global_map[][GLOBAL_COL], Coord coord) {
    // If coord is invalid, don't explore
    if (isValid(coord.row, coord.col) && global_map[coord.row][coord.col] == U)
        return true;

    return false;
}

static void Main::mapBlockInFrontTerrain() {
    /* Questions we want to answer:
        How do we detect that we're at the edge of one block ? (Use ultrasonic sensors ?)
    */

    /*
        Move forward until boundary of current block is met
        Take colour sensor reading to determine correct terrain of block infront
        Map this block
        Move robot back to center of the block
    */
}


static bool Main::isValid(int row, int col) {
  return row >= 0 && col >= 0 && row < GLOBAL_ROW && col < GLOBAL_COL;
}

/***********************
*    TASK FUNCTIONS    *
************************/
void Main::extinguishFire() {
    m_motor_pair.extinguishFireTurn();
}
