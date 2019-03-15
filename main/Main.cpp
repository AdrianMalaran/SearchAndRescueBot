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
    Serial.println("Main Engine Constructor");
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
    Serial.println("Main Engine Init()");
    // (0) Initialize sensors and actuators
    // (1) Initialize map
    // (2) Test Path Planning
    // (3) Calibrate sensors

    // Calibrate Imu
    // m_imu_sensor.calibrate();

    delay(1000);

    // Initialize Tasks
    m_found_food = false;
    m_found_people = false;
    m_found_survivor = false;
    m_extinguished_fire = false;

    // Set motor pins
    m_motor_pair.setupMotorPair();

    // Set flame pins
    Flame::setupFlame();
    m_extinguished_fire = false;

    // Set start coord
    m_start_coord = Coord(4, 5);

    // Initialize Start Map
    for (int i = 0; i < GLOBAL_ROW; i ++)
        for (int j = 0; j < GLOBAL_COL; j++)
            m_global_map[i][j].block_type = UNKNOWN;

    m_global_map[m_start_coord.row][m_start_coord.col].block_type = PARTICLE;
}

//TODO: This function will be run in the loop() function of the arduino (?)
// OR just in the setup loop?
void Main::run() {

    //TODO: Store next tasks in a different data structure than a queue,
    // because only putting out the fire needs to be done first
    while (!allTasksCompleted()) {
        Task task = getNextTask();

        if (taskIsMapped(task)) {
            engageObjectiveMode(task);
        } else {
            engageExploreMode();
        }
    }

    returnToStart(m_global_map, m_current_pose);
    // Stop Program
}

bool Main::allTasksCompleted() {
    return (m_found_food &&
            m_found_people &&
            m_found_survivor &&
            m_extinguished_fire);
}

Task Main::getNextTask() {
    if (!m_extinguished_fire)
        return EXTINGUISH_FIRE;

    // TODO: Allow for these task to be in any order
    if (!m_found_food)
        return FIND_FOOD;

    if (!m_found_people)
        return FIND_GROUP_OF_PEOPLE;

    return FIND_SURVIVOR;
}

bool Main::taskIsMapped(Task task) {
    // TODO: I'm not sure if we want to add this as a task if we plan
    // to put out the fire immediately when we start, may take out later
    if (task == EXTINGUISH_FIRE)
        // Special Case: This should be the first task performed
        // Assume that the candle is mapped when we perform extinguish fire
        return true;
    if (task == FIND_FOOD)
        return m_food_mapped;
    if (task == FIND_GROUP_OF_PEOPLE)
        return m_group_mapped;
    if (task == FIND_SURVIVOR)
        return m_survivor_mapped;

    Serial.println("Given task is unspecified");
    return false;
}

void Main::engageExploreMode() {
    Serial.println("Enaging Explore Mode!");

    //TODO: Test this
    MapLocation location_of_interest(UNKNOWN); // Initialize to unknown block
    Coord explore_block = findClosestBlockToInterest(m_global_map, location_of_interest, m_current_pose.coord);
    // Coord explore_block = findClosestBlockWithUnknownNeighbors(m_global_map, m_current_pose.coord);
    travelToBlock(m_global_map, m_current_pose, Pose(explore_block, DONTCARE));
    mapAdjacentBlocks(m_global_map, m_current_pose);
}

void Main::engageObjectiveMode(Task task) {
    Serial.println("Enaging Objective Mode!");

    switch (task) {
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

void Main::returnToStart(MapLocation global_map[][GLOBAL_COL], Pose current_pose) {
    Serial.println("Travelling back to start");
    travelToBlock(global_map, current_pose, Pose(m_start_coord, DONTCARE));
    Serial.println("Shutting Down...");
    stopProgram();
}

/***********************
* PERIPHERAL FUNCTIONS *
************************/

bool Main::isLandmarkAhead(MapLocation &map, Pose pose) {
    double front_distance = m_ultrasonic_front.getDistance();
    double back_distance = m_ultrasonic_back.getDistance();
    int row = pose.coord.row;
    int col = pose.coord.col;

    if (front_distance + back_distance < 155) {
        switch (pose.orientation) {
        case NORTH:
             if (front_distance < 30)
                return true;
        case SOUTH:
            if (front_distance < 30)
                return true;
        case EAST:
            if (front_distance < 30)
                return true;
        case WEST:
            if (front_distance < 30)
                return true;
        default:
            // TODO: We should never hit this case.... but DONTCARE is a thing
            Serial.println("UNKNOWN ORIENTATION");
            return false;
        }
    }

    return false;
}

Coord Main::getGlobalPosition(Pose pose) {
    double left_distance = m_ultrasonic_left.getDistance() / 30.3;
    double right_distance = m_ultrasonic_right.getDistance() / 30.3;
    double front_distance = m_ultrasonic_front.getDistance() / 30.3;
    double back_distance = m_ultrasonic_back.getDistance() / 30.3;

    if (left_distance + right_distance > 155 && front_distance + back_distance > 155) {
        switch (pose.orientation) {
        case NORTH:
            return Coord(floor(front_distance), floor(left_distance));
        case SOUTH:
            return Coord(floor(back_distance), floor(right_distance));
        case EAST:
            return Coord(floor(left_distance), floor(back_distance));
        case WEST:
            return Coord(floor(right_distance), floor(front_distance));
        default:
            Serial.println("UNKNOWN ORIENTATION");
            return Coord(-1,-1);
        }
    } else {
        // There is some sort of obstruction - return invalid coord
        return Coord(-1,-1);
    }
}

void Main::mapAdjacentBlocks(MapLocation (&global_map)[GLOBAL_ROW][GLOBAL_COL], Pose start_pose) {
    // Use motor encoders to measure distance ??
    // Detect adjacent blocks that are undiscovered

    // Explore all adjacent blocks
    Pose current_pose = start_pose;
    Pose desired_pose;

    Pose adjacent_blocks[4];
    adjacent_blocks[0] = Pose(Coord(start_pose.coord.row - 1, start_pose.coord.col), NORTH);
    adjacent_blocks[1] = Pose(Coord(start_pose.coord.row, start_pose.coord.col + 1), EAST);
    adjacent_blocks[2] = Pose(Coord(start_pose.coord.row + 1, start_pose.coord.col), SOUTH);
    adjacent_blocks[3] = Pose(Coord(start_pose.coord.row, start_pose.coord.col - 1), WEST);

    for (int i = 0; i < 4; i++) {
        int row = adjacent_blocks[i].coord.row;
        int col = adjacent_blocks[i].coord.col;
        MapLocation map_location = global_map[row][col];

        if (isValid(adjacent_blocks[i].coord) && map_location.block_type == UNKNOWN) {
            desired_pose = Pose(start_pose.coord, adjacent_blocks[i].orientation);
            travelToBlock(global_map, current_pose, desired_pose);

            current_pose = desired_pose;

            mapBlockInFront(map_location, current_pose);
        }
    }
    travelToBlock(global_map, current_pose, start_pose);
}

// TODO: Test
bool Main::isUnexplored(MapLocation global_map[][GLOBAL_COL], Coord coord) {
    // If coord is invalid, don't explore
    if (isValid(coord) && global_map[coord.row][coord.col].block_type == UNKNOWN)
        return true;

    return false;
}

void Main::mapBlockInFront(MapLocation &map_location, Pose pose) {
    /* Questions we want to answer:
        How do we detect that we're at the edge of one block ? (Use ultrasonic sensors )
    */

    // Try with ultrasonic - If this doesn't work, include encoder control
    double start_distance = m_ultrasonic_front.getDistance();
    while (m_ultrasonic_front.getDistance() > start_distance - 7) {
        // Move forwards
        Controller::DriveStraight(m_imu_sensor.getEuler().x(), m_imu_sensor.getEuler().x(), 180);
    }
    m_motor_pair.stop();

    map_location.block_type = m_color_down.getTerrainColor();

    map_location.land_mark_spot = isLandmarkAhead(map_location, pose);

    while (m_ultrasonic_front.getDistance() < start_distance) {
        // Move backwards
        Controller::DriveStraight(m_imu_sensor.getEuler().x(), m_imu_sensor.getEuler().x(), -180);
    }
    m_motor_pair.stop();
}

// static bool Main::isValid(Coord c) {
//   return c.row >= 0 && c.col >= 0 && c.row < GLOBAL_ROW && c.col < GLOBAL_COL;
// }

/***********************
*    TASK FUNCTIONS    *
************************/

void Main::findFood(MapLocation global_map[][GLOBAL_COL], Pose current_pose) {
    // Check to see if there are mapped sand blocks
    //     if there are no mapped sand blocks, go explore

    // Search for closest sand block
    // travel to sand block
    // inspect sand block to see if a magnet is detected
    // In response to the above higher level plan, would it not be simpler to check the sand blocks
    // as we traverse them, and if we are unsuccesful then go down this route - maybe a checked_for_magnet flag?

    //TODO: Need to know if the robot can just stay in place and the IMU will detect
    // that reading -> We would need to have the nominal stationary value and then
    // compare this value to the nominal - so yes, this should be simple enough to do,
    // we just need to characterize the magnetometer first.

    //TODO: if we get close enough to sand block boundary to detect a magnet,
    // then we should automatically mark that spot as the food location -> this could be possible,
    // but would be highly dependent on the magnetic field orientations.

    /*
    Coord closest_sand_block = getClosestSandBlock(global_map, current_pose.coord);
    if (closest_sand_block.row == -1 && closest_sand_block.col == -1) {
        Serial.println("All sandblocks mapped have been searched, explore for more");
        // Go explore unmapped blocks
    } else {
        // TODO: Maybe we can just go to the closest
        Serial.println("Travelling to closest sand block");
        travelToBlock(global_map, current_pose, Pose(closest_sand_block, DONTCARE));
    }
    */

    /*
    This objective function assumes:
    - that the IMU is already able to detect the magnets
      in the area as soon as the terrain is detected
    - the sandblock is already mapped
    */

    //TODO: If the food is already found, don't we just need to go to the group of people ?
    travelToBlock(global_map, current_pose, Pose(m_food_location, DONTCARE));

}

Coord Main::getClosestSandBlock(MapLocation global_map[][GLOBAL_COL], Coord current_loc) {
    int min_distance = INT_MAX;
    Coord closest_sand_block = Coord(-1, -1); // Error coordinate

    for (int i = 0; i < GLOBAL_ROW; i++) {
        for (int j = 0; j < GLOBAL_COL; j++) {
            // Check to see if the block is a SAND block and that
            // it has not already been searched
            if (global_map[i][j].block_type == SAND && global_map[i][j].searched == false) {
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
        PathPlanning::findShortestPath(map, start_pose.coord, finish_pose.coord);
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
    Serial.println("Executed Maneuvers");

    //updateLocation(); possibly update location ?
}

//TODO: Replace this function
bool Main::hasUnknownNeighbors(MapLocation global_map[][GLOBAL_COL], Coord start_loc) {
    if (!isValid(start_loc)) {
        Serial.println("Invalid start coordinate");
        return false;
    }

    Coord adjacent[4];
    adjacent[0] = Coord(start_loc.row - 1, start_loc.col);
    adjacent[1] = Coord(start_loc.row + 1, start_loc.col);
    adjacent[2] = Coord(start_loc.row, start_loc.col - 1);
    adjacent[3] = Coord(start_loc.row, start_loc.col + 1);

    for (int i = 0; i < 4; i++) {
        //TODO: Replace 2nd condition with neighborMatchesCondition
        if (isValid(adjacent[i]) && global_map[adjacent[i].row][adjacent[i].col].block_type == UNKNOWN) // Unknown
            return true;
    }

    return false;
}

bool Main::hasMatchingNeighbors(MapLocation global_map[][GLOBAL_COL], MapLocation location_of_interest, Coord start_loc) {
    if (!isValid(start_loc)) {
        Serial.println("Invalid start coordinate");
        return false;
    }

    Coord adjacent[4];
    adjacent[0] = Coord(start_loc.row - 1, start_loc.col);
    adjacent[1] = Coord(start_loc.row + 1, start_loc.col);
    adjacent[2] = Coord(start_loc.row, start_loc.col - 1);
    adjacent[3] = Coord(start_loc.row, start_loc.col + 1);

    for (int i = 0; i < 4; i++) {
        //TODO: Replace 2nd condition with neighborMatchesCondition
        if (neighborMatchesCondition(global_map, location_of_interest, start_loc))
            return true;
    }

    return false;
}

bool Main::neighborMatchesCondition(MapLocation global_map[][GLOBAL_COL], MapLocation location_of_interest, Coord coord) {
    if (!isValid(coord))
        return false;
    // Searches for closest block that has a neighbor that matches the criteria:

    // EXPLORE MODE | Block w/ unknown neighbor
    if (location_of_interest.block_type == UNKNOWN)
        return (global_map[coord.row][coord.col].block_type == UNKNOWN);

    // OBJECTIVE MODE | Block w/ location of interest
    if (location_of_interest.land_mark_spot)
        return global_map[coord.row][coord.col].land_mark_spot;

    return false;
}

//TODO: This should already return the direction to the block of interest
Coord Main::findClosestBlockToInterest(MapLocation global_map[][GLOBAL_COL],
    MapLocation location_of_interest,
    Coord start_loc) {

    Coord invalid_coord = Coord(-1, -1);
    if (!isValid(start_loc)) {
        Serial.println("Invalid start coordinate");
        return invalid_coord;
    }

    // Implement a BFS
    Queue<Coord> to_visit;

    to_visit.push(start_loc);
    // Initialize all to false (unvisited)
    bool visited[GLOBAL_ROW][GLOBAL_COL] =
        {
            { 0, 0, 0, 0, 0, 0}, // 0
            { 0, 0, 0, 0, 0, 0}, // 1
            { 0, 0, 0, 0, 0, 0}, // 2
            { 0, 0, 0, 0, 0, 0}, // 3
            { 0, 0, 0, 0, 0, 0}, // 4
            { 0, 0, 0, 0, 0, 0}  // 5
            //0, 1, 2, 3, 4, 5
        };

    visited[start_loc.row][start_loc.col] = true;

    Coord current;

    while (!to_visit.empty()) {
        current = to_visit.front();
        to_visit.pop();
        // TODO: Remove print statements
        // Serial.print("Current Block: ("); Serial.print(current.row); Serial.print(","); Serial.print(current.col); Serial.println(")");
        if (hasMatchingNeighbors(global_map, location_of_interest, current)) // Found a coord with unknown neighbors
            return current;

        // Explore all neighbors
        Coord adjacent[4];
        adjacent[0] = Coord(current.row - 1, current.col);
        adjacent[1] = Coord(current.row + 1, current.col);
        adjacent[2] = Coord(current.row, current.col - 1);
        adjacent[3] = Coord(current.row, current.col + 1);

        for (int i = 0; i < 4; i++) {
            int row = adjacent[i].row;
            int col = adjacent[i].col;

            // Validate traversable block
            if (isValid(adjacent[i]) && !visited[row][col] && isUnblocked(global_map, adjacent[i])) {
                visited[row][col] = true;
                to_visit.push(adjacent[i]);
            }
        }
    }

    Serial.println("No block with interesting neighbors");
    return invalid_coord;
}

//TODO: Delete COPY
Coord Main::findClosestBlockWithUnknownNeighbors(MapLocation global_map[][GLOBAL_COL], Coord start_loc) {
    Coord invalid_coord = Coord(-1, -1);
    if (!isValid(start_loc)) {
        Serial.println("Invalid start coordinate");
        return invalid_coord;
    }

    // Implement a BFS
    Queue<Coord> to_visit;

    to_visit.push(start_loc);
    // Initialize all to false (unvisited)
    bool visited[GLOBAL_ROW][GLOBAL_COL] =
        {
            { 0, 0, 0, 0, 0, 0}, // 0
            { 0, 0, 0, 0, 0, 0}, // 1
            { 0, 0, 0, 0, 0, 0}, // 2
            { 0, 0, 0, 0, 0, 0}, // 3
            { 0, 0, 0, 0, 0, 0}, // 4
            { 0, 0, 0, 0, 0, 0}  // 5
            //0, 1, 2, 3, 4, 5
        };

    visited[start_loc.row][start_loc.col] = true;

    Coord current;

    while (!to_visit.empty()) {
        current = to_visit.front();
        to_visit.pop();
        // TODO: Remove print statements
        // Serial.print("Current Block: ("); Serial.print(current.row); Serial.print(","); Serial.print(current.col); Serial.println(")");
        if (hasUnknownNeighbors(global_map, current)) // Found a coord with unknown neighbors
            return current;

        // Explore all neighbors
        Coord adjacent[4];
        adjacent[0] = Coord(current.row - 1, current.col);
        adjacent[1] = Coord(current.row + 1, current.col);
        adjacent[2] = Coord(current.row, current.col - 1);
        adjacent[3] = Coord(current.row, current.col + 1);

        for (int i = 0; i < 4; i++) {
            int row = adjacent[i].row;
            int col = adjacent[i].col;

            // Validate traversable block
            if (isValid(adjacent[i]) && !visited[row][col] && isUnblocked(global_map, adjacent[i])) {
                visited[row][col] = true;
                to_visit.push(adjacent[i]);
            }
        }
    }

    Serial.println("No block with unknown neighbors");
    return invalid_coord;
}

void Main::stopProgram() {
    set_sleep_mode(SLEEP_MODE_PWR_DOWN);
    cli();  // Disable interrupts
    sleep_mode();
}

void Main::extinguishFire() {
    Serial.println("extinguishFire()");
    m_extinguished_fire = m_motor_pair.extinguishFireTurn();
    Serial.println("Fire extinguished");
}
