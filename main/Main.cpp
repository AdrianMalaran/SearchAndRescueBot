#include "Main.h"

Main::Main() {}

// Global Variables for encoders
double m_motor_speed_A = 999;
double m_motor_speed_B = 999;
long last_encA_value = -100;
long last_encB_value = -100;
long timer_micro_seconds = 1000; // every 1 millisecond

extern Encoder testEncoderA(encoderAPin1, encoderAPin2);
extern Encoder testEncoderB(encoderBPin1, encoderBPin2);

Main::Main(MotorPair motor_pair, Imu imu_sensor, Color color_front, Color color_down,
            Ultrasonic ultrasonic_front, Ultrasonic ultrasonic_right, Ultrasonic ultrasonic_left,
            Ultrasonic ultrasonic_back, Controller controller, Encoder &encoder_A, Encoder &encoder_B) {
    m_motor_pair = motor_pair;
    m_imu_sensor = imu_sensor;
    m_color_front = color_front;
    m_color_down = color_down;
    m_ultrasonic_front = ultrasonic_front;
    m_ultrasonic_right = ultrasonic_right;
    m_ultrasonic_left = ultrasonic_left;
    m_ultrasonic_back = ultrasonic_back;
    m_controller = controller;
    m_encoder_A = encoder_A;
    m_encoder_B = encoder_B;

    init();
}

void Main::init() {
    // (0) Initialize sensors and actuators
    // (1) Initialize map
    // (2) Test Path Planning
    // (3) Calibrate sensors
    Serial.println("Initializing Main Engine...");
    // Start the imu
    m_imu_sensor.begin();
    Serial.println("Imu Sensor Reading");

    delay(1000);

    // Initialize Tasks
    m_found_food = false;
    m_found_people = false;
    m_found_survivor = false;
    m_extinguished_fire = false;
    m_map_discovered = false;

    // Set motor pins
    m_motor_pair.setupMotorPair();

    // Set flame pins
    Flame::setupFlame();

    // Set start pose
    m_start_coord = Coord(5, 3);
    updateLocation(Pose(m_start_coord, NORTH));

    // Initialize Start Map
    for (int i = 0; i < GLOBAL_ROW; i ++) {
        for (int j = 0; j < GLOBAL_COL; j++) {
            m_global_map[i][j].block_type = UNKNOWN;
        }
    }

    m_global_map[m_start_coord.row][m_start_coord.col].block_type = PARTICLE;
    m_global_map[m_start_coord.row-1][m_start_coord.col].block_type = PARTICLE;

    // Initialize timers
    // Timer1.initialize(timer_micro_seconds); // Microseconds
    // Timer1.attachInterrupt(updateActualSpeed);

    /* Initialize Speed Control*/
}

//TODO: This function will be run in the loop() function of the arduino (?)
// OR just in the setup loop?
// TODO: Add possible fail-safe that saves the last image of the main member function
// and possibly reloads it based off of that value
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

    // End program after returning to start
    returnToStart(m_global_map, m_current_pose);
}

bool Main::allTasksCompleted() {
    return (m_found_food &&
            m_found_people &&
            m_found_survivor &&
            m_extinguished_fire);
}

Task Main::getNextTask() {
    if (!m_map_discovered)
        return DISCOVER_MAP;

    if (!m_extinguished_fire)
        return EXTINGUISH_FIRE;

    //TODO: must take care of the case where we find the people first
    // but have not found the food yet
    // In this case when, we find the food then we should find the people
    if (m_deliver_food_to_group)
        return DELIVER_FOOD;


    return OTHER;
}

bool Main::taskIsMapped(Task task) {
    if (task == DISCOVER_MAP)
        return true;
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

    MapLocation location_of_interest(UNKNOWN); // Initialize to unknown block
    Orientation finish_ori = DONTCARE;
    Coord explore_block = findClosestBlockToInterest(m_global_map, location_of_interest, m_current_pose.coord, finish_ori);

    if (!isValid(explore_block)) { // No Path Found
        Serial.println("No other blocks can be explored. Returning to start");
        returnToStart(m_global_map, m_current_pose);
        return;
    }

    travelToBlock(m_global_map, m_current_pose, Pose(explore_block, DONTCARE));
    Serial.println("Mapping Adjacent Blocks");
    mapAdjacentBlocks(m_global_map, m_current_pose);
}

void Main::engageObjectiveMode(Task task) {
    Serial.println("Enaging Objective Mode!");

    switch (task) {
        case DISCOVER_MAP:
            Serial.println("Discovering Correct Map ...");
            findCorrectMap();
        case EXTINGUISH_FIRE:
            // Assume there is some sort of Path Planning (Adrian) that gets us within
            // the flame sensors range of the candle.
            Serial.println("TASK: Extinguishing Fire");
            extinguishFire();
            break;
        case FIND_FOOD:
            Serial.println("TASK: Finding Food");
            // travelToFood();
            break;
        // TODO: Don't need these specific tasks, they will be discovered
        // May be needed if we have it pre-mapped and stuff, hehehehehehe
        case FIND_GROUP_OF_PEOPLE:
            Serial.println("TASK: Finding Group of People");
            break;
        case DELIVER_FOOD:
            Serial.println("TASK: Delivering Food");
            // TODO: Implement this to
            deliverFoodToGroup(); // Map a path to the group of people
            break;
        // case FIND_SURVIVOR:
        //     Serial.println("TASK: Finding Survivor");
        //     break;
        default:
            Serial.println("UNKNOWN TASK");
            break;
    }
}

void Main::returnToStart(MapLocation global_map[][GLOBAL_COL], Pose current_pose) {
    Serial.println("Travelling back to start");
    travelToBlock(global_map, current_pose, Pose(m_start_coord, DONTCARE));
    Serial.println("Shutting Down Program.");
    stopProgram();
}

void Main::deliverFoodToGroup() {
    MapLocation interestlocation(PARTICLE, true, SURVIVOR);
    Orientation finish_ori;
    Coord block = findClosestBlockToInterest(m_global_map, interestlocation, m_current_pose.coord, finish_ori);
    travelToBlock(m_global_map, m_current_pose, Pose(block, DONTCARE));
    // Indicate that its been found
}

void Main::findCorrectMap() {
    MapLocation west_location(UNKNOWN);
    MapLocation north_east_location(UNKNOWN);

    Coord west_block = Coord(m_current_pose.coord.row,m_current_pose.coord.col - 1);
    Coord north_east_block = Coord(m_current_pose.coord.row - 1,m_current_pose.coord.col + 1);

    double start_mag = m_imu_sensor.getMag().z();

    travelToBlock(m_global_map, m_current_pose, Pose(m_current_pose.coord, WEST));
    mapBlockInFront(m_global_map, m_current_pose, start_mag, west_block);
    west_location = m_global_map[west_block.row][west_block.col];

    start_mag = m_imu_sensor.getMag().z();

    travelToBlock(m_global_map, m_current_pose, Pose(Coord(m_current_pose.coord.row - 1, m_current_pose.coord.col), EAST));
    mapBlockInFront(m_global_map, m_current_pose, start_mag, north_east_block);
    north_east_location = m_global_map[north_east_block.row][north_east_block.col];

    // Identify Map
    m_map_discovered = true;
    // TODO: Consider what happens if we detect food in the sand
    if (west_location.block_type == GRAVEL) {
        if (north_east_location.block_type == SAND)
            // m_global_map = potential_map1;
            setCorrectMap(potential_map1);
        else if (north_east_location.block_type == WATER)
            // m_global_map = potential_map4;
            setCorrectMap(potential_map4);
        else {
            Serial.println("UNABLE TO IDENTIFY MAPPING");
            m_map_discovered = false;
            engageExploreMode();
        }
    }
    else if (west_location.block_type == WATER) {
        if (north_east_location.block_type == SAND)
            // m_global_map = potential_map3;
            setCorrectMap(potential_map3);
        else if (north_east_location.block_type == GRAVEL)
            // m_global_map = potential_map2;
            setCorrectMap(potential_map2);
        else {
            Serial.println("UNABLE TO IDENTIFdffksdf");
            m_map_discovered = false;
            engageExploreMode();
        }
    } else {
        Serial.println("UNABLE TO IDENTIFY MAPPING --");
        m_map_discovered = false;
        engageExploreMode();
    }

    printMap(m_global_map);

    Serial.println("Map Set");
}

void Main::setCorrectMap(MapLocation map[][GLOBAL_COL]) {
    for (int i = 0; i < GLOBAL_ROW; i ++) {
        for (int j = 0; j < GLOBAL_COL; j++) {
            m_global_map[i][j].block_type = map[i][j].block_type;
        }
    }
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

Landmark identifyLandMark() {
    // TODO: Implement this function using color sensor readings
    if (true)
        return SURVIVOR;
    if (true)
        return PEOPLE;

    return FIRE;
}

Coord Main::getGlobalPosition(Pose pose) {
    double left_distance = m_ultrasonic_left.getDistance() / 30.3;
    delay(500);
    double right_distance = m_ultrasonic_right.getDistance() / 30.3;
    delay(500);
    double front_distance = m_ultrasonic_front.getDistance() / 30.3;
    delay(500);
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

void Main::getPossibleLandmarks(MapLocation (&global_map)[GLOBAL_ROW][GLOBAL_COL], Pose pose) {
    double left_distance = m_ultrasonic_left.getDistance() / 30.3;
    delay(500);
    double right_distance = m_ultrasonic_right.getDistance() / 30.3;
    delay(500);
    double front_distance = m_ultrasonic_front.getDistance() / 30.3;
    delay(500);
    double back_distance = m_ultrasonic_back.getDistance() / 30.3;

    switch (pose.orientation) {
    case NORTH:
        if(floor(front_distance) != pose.coord.row)
            global_map[(int)floor(front_distance)][pose.coord.col].land_mark_spot = true;
        if(ceil(back_distance) != pose.coord.row)
            global_map[(int)ceil(front_distance)][pose.coord.col].land_mark_spot = true;
        if(floor(left_distance) != pose.coord.col)
            global_map[pose.coord.row][(int)floor(left_distance)].land_mark_spot = true;
        if(ceil(right_distance) != pose.coord.col)
            global_map[pose.coord.row][(int)ceil(right_distance)].land_mark_spot = true;
    case SOUTH:
        if(ceil(front_distance) != pose.coord.row)
            global_map[(int)ceil(front_distance)][pose.coord.col].land_mark_spot = true;
        if(floor(back_distance) != pose.coord.row)
            global_map[(int)floor(front_distance)][pose.coord.col].land_mark_spot = true;
        if(ceil(left_distance) != pose.coord.col)
            global_map[pose.coord.row][(int)ceil(left_distance)].land_mark_spot = true;
        if(floor(right_distance) != pose.coord.col)
            global_map[pose.coord.row][(int)floor(right_distance)].land_mark_spot = true;
    case EAST:
        if(floor(front_distance) != pose.coord.col)
            global_map[pose.coord.row][(int)floor(front_distance)].land_mark_spot = true;
        if(ceil(back_distance) != pose.coord.col)
            global_map[pose.coord.row][(int)ceil(front_distance)].land_mark_spot = true;
        if(ceil(left_distance) != pose.coord.row)
            global_map[(int)ceil(left_distance)][pose.coord.col].land_mark_spot = true;
        if(floor(right_distance) != pose.coord.row)
            global_map[(int)floor(right_distance)][pose.coord.col].land_mark_spot = true;
    case WEST:
        if(ceil(front_distance) != pose.coord.col)
            global_map[pose.coord.row][(int)ceil(front_distance)].land_mark_spot = true;
        if(floor(back_distance) != pose.coord.col)
            global_map[pose.coord.row][(int)floor(front_distance)].land_mark_spot = true;
        if(floor(left_distance) != pose.coord.row)
            global_map[(int)floor(left_distance)][pose.coord.col].land_mark_spot = true;
        if(ceil(right_distance) != pose.coord.row)
            global_map[(int)ceil(right_distance)][pose.coord.col].land_mark_spot = true;
    default:
        Serial.println("UNKNOWN ORIENTATION");
    }
}

void Main::mapAdjacentBlocks(MapLocation (&global_map)[GLOBAL_ROW][GLOBAL_COL], Pose start_pose) {
    // Use motor encoders to measure distance ??
    // Detect adjacent blocks that are undiscovered

    // Explore all adjacent blocks
    Pose current_pose = start_pose;
    Pose desired_pose;

    double start_mag = m_imu_sensor.getMag().z();

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
            Serial.println("Mapping Block in front");
            current_pose = desired_pose;

            mapBlockInFront(global_map, current_pose, start_mag, adjacent_blocks[i].coord);
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

void Main::checkForLandMark(MapLocation (&global_map)[GLOBAL_ROW][GLOBAL_COL],
                            Coord block_to_map,
                            double start_mag,
                            Pose pose) {

    MapLocation& map_location = global_map[block_to_map.row][block_to_map.col];
    if (map_location.block_type == SAND && isFood(start_mag)) {
        m_found_food = true;
        LED::onAndOff();
        m_food_location = block_to_map;

        // At this point, we found the food, now we want to see if we've already found the people
        // if we've already found the group of people, then plan to deliver the food to
        // that group of people straight away
        if (m_found_people) {
            m_deliver_food_to_group = true;
        }
    }

    map_location.land_mark_spot = isLandmarkAhead(map_location, pose);
    // TODO: Need to identify what type of landmark it is
    if (map_location.land_mark_spot) {
        map_location.landmark = identifyLandMark();

        // Mark Task as finished
        m_deliver_food_to_group = false;
    }
}

void Main::mapBlockInFront(MapLocation (&global_map)[GLOBAL_ROW][GLOBAL_COL], Pose pose, double start_mag, Coord block_to_map) {

    //TODO: Remove after Testing
    // MapLocation MP(PARTICLE);
    // MapLocation MS(SAND);
    // MapLocation MW(WATER);
    // MapLocation MG(GRAVEL);
    // MapLocation MU(UNKNOWN);

    // MapLocation MLP(LANDMARK, true, PEOPLE);
    // MapLocation MLS(LANDMARK, true, SURVIVOR);
    // MapLocation MLC(LANDMARK, true, FIRE);
    // MapLocation MLF(SAND, true, FOOD);

    // MapLocation testGrid[GLOBAL_ROW][GLOBAL_COL] =
    // {{ MP, MP, MP, MP, MP, MP}, // 0
    //  { MP, MLF, MP, MLC, MP, MP}, // 1
    //  { MP, MP, MP, MW, MS, MP}, // 2
    //  { MP, MLP, MS, MP, MW, MW}, // 3
    //  { MP, MP, MW, MP, MW, MP}, // 4
    //  { MP, MLS, MW, MP, MP, MP}};// 5

    MapLocation testGrid[GLOBAL_ROW][GLOBAL_COL] =
    {
    //    0, 1, 2, 3, 4, 5
        { MP, MP, MP, MG, MP, MP}, // 0
        { MP, MS, MP, MP, MW, MP}, // 1
        { MW, MP, MP, MS, MP, MP}, // 2
        { MP, MP, MP, MP, MP, MG}, // 3
        { MP, MG, MP, MP, MS, MP}, // 4
        { MP, MP, MW, MP, MP, MP}  // 5
    };
    delay(2000);
    Serial.println("Mapped Block: "); printCoord(block_to_map);

    global_map[block_to_map.row][block_to_map.col] = testGrid[block_to_map.row][block_to_map.col];
    Serial.println("New Map:");
    printMap(global_map);
    return;


    /* Questions we want to answer:
        How do we detect that we're at the edge of one block ? (Use ultrasonic sensors)
    */
    double start_heading = m_imu_sensor.getEuler().x();

    // Try with ultrasonic - If this doesn't work, include encoder control
    double start_distance = m_ultrasonic_front.getDistance();
    while (m_ultrasonic_front.getDistance() > start_distance - 7) {
        // Move forwards
        m_controller.driveStraightController(m_imu_sensor.getEuler().x(), m_imu_sensor.getEuler().x(), 180);
    }
    m_motor_pair.stop();

    // TODO: Get feedback to see if the terrain color is unknown, then keep trying
    BlockType terrain_type = m_color_down.getTerrainColor();
    while (terrain_type == UNKNOWN) {
        terrain_type = m_color_down.getTerrainColor();
        global_map[block_to_map.row][block_to_map.col].block_type = terrain_type;
    }

    checkForLandMark(global_map, block_to_map, start_mag, pose);

    while (m_ultrasonic_front.getDistance() < start_distance) {
        // Move backwards
        m_controller.driveStraightController(m_imu_sensor.getEuler().x(), m_imu_sensor.getEuler().x(), -180);
    }
    m_motor_pair.stop();
}

bool Main::isFood(double current_mag) {
    return (fabs(fabs(m_imu_sensor.getMag().z()) - fabs(current_mag)) > 5);
}

/***********************
*    TASK FUNCTIONS    *
************************/

void Main::travelToFood(MapLocation global_map[][GLOBAL_COL], Pose current_pose) {

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

    MapLocation location_of_interest(SAND, true, FOOD); // Initialize to unknown block
    Orientation finish_ori = DONTCARE;
    Coord explore_block = findClosestBlockToInterest(m_global_map, location_of_interest, m_current_pose.coord, finish_ori);
    travelToBlock(m_global_map, m_current_pose, Pose(m_food_location, finish_ori));

    // Indicate that you've found the food
    // blink LED
}

//TODO: Possibly delete this function
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

// TODO: Return bool to make sure this is succesful
void Main::travelToBlock(MapLocation map[][GLOBAL_COL], Pose start_pose, Pose finish_pose) {

    if (start_pose == finish_pose) {
        Serial.println("already in position");
        return;
    }
    Serial.print("Travelling from "); printPose(start_pose);
    Serial.print(" to "); printPose(finish_pose); Serial.println("");
    // Path Plan from current_location to dest
    Stack<Coord> shortest_path =
        PathPlanning::findShortestPath(map, start_pose.coord, finish_pose.coord);

    // Handle Invalid Path errors
    if (shortest_path.size() == 1 &&
        shortest_path.top().row == -1 &&
        shortest_path.top().col == -1) {
        Serial.println("No Path Found");
        return;
    }

    Queue<Instruction> maneuver_instructions =
        PathPlanning::generateTrajectories(shortest_path, start_pose.orientation, finish_pose.orientation);

    Orientation final_orientation = start_pose.orientation;
    executeInstructions(maneuver_instructions, final_orientation);
    updateLocation(Pose(finish_pose.coord, final_orientation));
}

void Main::updateLocation(Pose finish_pose) {
    Serial.print("Updating Location to ");
    printPose(finish_pose);
    Serial.println("");
    // Update
    m_current_pose = finish_pose;
}

bool Main::hasMatchingNeighbors(MapLocation global_map[][GLOBAL_COL],
                                MapLocation location_of_interest,
                                Coord start_loc,
                                Orientation &dir_towards) {
    if (!isValid(start_loc)) {
        Serial.println("Invalid start coordinate");
        return false;
    }

    Pose adjacent[4];
    adjacent[0] = Pose(Coord(start_loc.row - 1, start_loc.col), NORTH); // NORTH
    adjacent[1] = Pose(Coord(start_loc.row + 1, start_loc.col), SOUTH); // SOUTH
    adjacent[2] = Pose(Coord(start_loc.row, start_loc.col - 1), WEST); // WEST
    adjacent[3] = Pose(Coord(start_loc.row, start_loc.col + 1), EAST); // EAST

    for (int i = 0; i < 4; i++) {
        //TODO: Replace 2nd condition with neighborMatchesCondition
        if (neighborMatchesCondition(global_map, location_of_interest, adjacent[i].coord)) {
            dir_towards = adjacent[i].orientation;
            return true;
        }
    }

    return false;
}

bool Main::neighborMatchesCondition(MapLocation global_map[][GLOBAL_COL],
                                    MapLocation location_of_interest, Coord coord) {
    if (!isValid(coord))
        return false;
    // Searches for closest block that has a neighbor that matches the criteria:

    // EXPLORE MODE | Block w/ unknown neighbor
    if (location_of_interest.block_type == UNKNOWN)
        return (global_map[coord.row][coord.col].block_type == UNKNOWN);

    // OBJECTIVE MODE | Block w/ location of interest
    if (location_of_interest.land_mark_spot) {
        return global_map[coord.row][coord.col].landmark == location_of_interest.landmark;
    }

    return false;
}

Coord Main::findClosestBlockToInterest(MapLocation global_map[][GLOBAL_COL],
                                       MapLocation location_of_interest,
                                       Coord start_loc,
                                       Orientation &dir_towards) {

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
    dir_towards = DONTCARE;

    while (!to_visit.empty()) {
        current = to_visit.front();
        to_visit.pop();
        // TODO: Remove print statements
        // Serial.print("Current Block: ("); Serial.print(current.row); Serial.print(","); Serial.print(current.col); Serial.println(")");

        if (hasMatchingNeighbors(global_map, location_of_interest, current, dir_towards)) // Found a coord with unknown neighbors
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

    Serial.println("No block with unknown/interesting neighbors");
    return invalid_coord;
}

/* Localization Functions */
double Main::getCurrentOrientation() {

}

// Uses Ultra-sonic sensors to detect distance
void Main::moveForwardOneBlock(double distance) {
	// Starting Pose
	// TODO: Make a more intelligent design to drive forward one block
	// by adding checks to validate which ultrasonic readings to get
	double start_distance = m_ultrasonic_front.getDistance();
    double start_heading = m_imu_sensor.getEuler().x();

    Serial.print("Start Distance: "); Serial.print(start_distance);
    Serial.print(" Start Heading: "); Serial.println(start_heading);

    //TODO: Change to the length of one block for now test for 10 cms
	// Want to end up 10 centimeters forward
	double end_distance = start_distance - distance;

	// Moves forward one block using either
	// A: ENCODERS
	// OR
	// B: ULTRASONICS
    int counter = 0;
    int ultra_sonic_reading = start_distance;

	while (true) {
        if (counter >= 100) {
            counter = 0;
            ultra_sonic_reading = m_ultrasonic_front.getDistance();
            if (ultra_sonic_reading <= end_distance)
                break;
        }
        else {
            m_controller.driveStraightController(start_heading, m_imu_sensor.getEuler().x(), TRAVEL_SPEED);
            counter ++;
        }
	}

    Serial.println("Stopping Motors");
    m_motor_pair.stop();
	// Use ultrasonic sensors for now
}

void Main::moveForwardSpeedControl() {
    // Overarching Function to be called
    double set_speed = 0;

    double motor_a_pwm;

    while (true) {
        // speedControl(motor_a_pwm,  180, m_motor_speed_A, 180);
    }
}

void Main::moveForwardSetDistance(double distance) {
    // Using encoders
    // long start_tick_a = m_encoder_A.read();
    // long start_tick_b = m_encoder_B.read();

    //TODO: TEST TO SEE IF WE CAN ZERO THE encoders
    m_encoder_A.write(0);
    m_encoder_B.write(0);

    long distance_in_ticks = distance/distance_per_tick;
    // Assume no slip
    // m_motor_pair.setMotorAPWM(TRAVEL_SPEED);
    // m_motor_pair.setMotorBPWM(TRAVEL_SPEED);

    // Track each wheel separately
    // Map distance to number of ticks that need to be range
    // Encoder A and B should travel the same number of ticks
    while (m_encoder_A.read() < distance_in_ticks || m_encoder_B.read() < distance_in_ticks) {
        // if (m_encoder_A.read() >= distance_in_ticks)
        //     m_motor_pair.setMotorAPWM(0);
        // else if (m_encoder_B.read() >= distance_in_ticks)
        //     m_motor_pair.setMotorBPWM(0);
        // drivestraight
    }

    m_motor_pair.stop();

    // Re-zero the encoders
    m_encoder_A.write(0);
    m_encoder_B.write(0);
}

bool Main::isStabilized(double& last_heading, double current_heading, double end_heading) {
    double tolerance = 2;

    double heading_change = last_heading - current_heading;
    double heading_position = current_heading - end_heading;

    if (heading_change > 180)
        heading_change -= 360;
    else if (heading_change < -180)
        heading_change += 360;

    if (heading_position > 180)
        heading_position -= 360;
    else if (heading_position < -180)
        heading_position += 360;

    if (abs(heading_change) <= 2 && abs(heading_position) <= 5)
        return true;

    return false;
}

void Main::turnLeft() {

    double start_heading = m_imu_sensor.getEuler().x();
	double end_heading = start_heading - 90;
    if (end_heading < 0) end_heading += 360;

    double last_heading = start_heading + 5; //TODO: Offset from heading change

    while (!isStabilized(last_heading, m_imu_sensor.getEuler().x(), end_heading)) {
        m_controller.turnController(end_heading, m_imu_sensor.getEuler().x(), 230, true);
    }

    m_motor_pair.stop();

    /*
    Serial.print("Starting Orientation: "); Serial.println(start_orientation);

    m_motor_pair.setMotorAPWM(-1.0*TURN_SPEED);
	m_motor_pair.setMotorBPWM(TURN_SPEED);

    if (start_orientation > end_orientation) {
        while (m_imu_sensor.getEuler().x() > end_orientation) {}
    } else {
        while (m_imu_sensor.getEuler().x() > 0) {}
        while (m_imu_sensor.getEuler().x() > end_orientation) {}
    }

	m_motor_pair.stop();
    */
}

void Main::turnRight() {

    double start_heading = m_imu_sensor.getEuler().x();
    double end_heading = start_heading + 90;
    if (end_heading > 360) end_heading -= 360;

    double last_heading = start_heading + 5; //TODO: Offset from heading change

    while (!isStabilized(last_heading, m_imu_sensor.getEuler().x(), end_heading)) {
        m_controller.turnController(end_heading, m_imu_sensor.getEuler().x(), 170, false);
    }

    m_motor_pair.stop();
}

// void Main::turnRight() {
//
//     double start_orientation = m_imu_sensor.getEuler().x();
//     double end_orientation = start_orientation + 90;
//     if (end_orientation > 360) end_orientation -= 360;
//
//     m_motor_pair.setMotorAPWM(TURN_SPEED);
//     m_motor_pair.setMotorBPWM(-1.0*TURN_SPEED);
//
//     if (start_orientation < end_orientation) {
//         while (m_imu_sensor.getEuler().x() < end_orientation) {}
//     } else {
//         while (m_imu_sensor.getEuler().x() < 360) {}
//         while (m_imu_sensor.getEuler().x() < end_orientation) {}
//     }
//     //TODO: Possibly implement a PI control for this
//
//     Serial.println("Stop Motors");
//     m_motor_pair.stop();
// }

void Main::findFire() {
    if (Flame::getFireMagnitude() > 0) {
        m_motor_pair.stop();

        extinguishFire();
    } else {
        double start_heading = m_imu_sensor.getEuler().x();
        double end_heading = start_heading;
        double last_heading = start_heading + 5;

        while (!isStabilized(last_heading, m_imu_sensor.getEuler().x(), end_heading)) {
            m_controller.turnController(end_heading, m_imu_sensor.getEuler().x(), 170, true);

            if (Flame::getFireMagnitude() > 0) {
                m_motor_pair.stop();
                extinguishFire();
            }
        }

        m_motor_pair.stop();
    }
}

void Main::extinguishFire() {
    LED::on();
    while (Flame::getFireMagnitude() > 0) {
        Fan::on();

        delay(1500);

        Fan::off();

        delay(1500);
    }
    LED::off();
    m_extinguished_fire = true;
}

// Executes main batch of movement functions necessary for travelling
// between two location blocks
// By the end of this function, the robot should be at the destination location
static void Main::executeInstructions(Queue<Instruction> instructions, Orientation& orientation) {
    // Validate instructions
    if (instructions.empty())
        return;

    Orientation current_orientation = orientation;

    Serial.println("Executing Instructions...");
    while (!instructions.empty()) {
        Instruction ins = instructions.front();
        instructions.pop();

        if (ins == MOVE_FORWARD) {
            Serial.print("FORWARDS->");
            //TODO:
            moveForwardOneBlock(30.0);
            delay(1000);
        }
        else if (ins == MOVE_BACKWARD) {
            Serial.print("BACKWARDS->");
            // Map -> moveBackward()
            // moveBackwardOneBlock();
            delay(1000);
        }
        else if (ins == ROTATE_RIGHT) {
            Serial.print("RIGHT->");
            current_orientation = (current_orientation + 1) % 4;
            // m_motor_pair.turnRight();
            delay(1000);
        }
        else if (ins == ROTATE_LEFT) {
            Serial.print("LEFT->");
            current_orientation = (current_orientation - 1);
            if (current_orientation < 0) current_orientation = 3;

            MotorPair::setMotorAPWM(-255);
            MotorPair::setMotorBPWM(223);
            delay(1000);
            MotorPair::stop();
            // Map -> turnLeft()
            // m_motor_pair.turnLeft();
        }
    }
    Serial.println("");
    orientation = current_orientation;
    Serial.print("Last Orientation: "); printOrientation(orientation); Serial.println("");
}

static void Main::updateActualSpeed() {
    long current_encA_value = testEncoderA.read();
    m_motor_speed_A = calculateSpeed(current_encA_value, last_encA_value, timer_micro_seconds);
    last_encA_value = current_encA_value;

    long current_encB_value = testEncoderB.read();
    m_motor_speed_B = calculateSpeed(current_encB_value, last_encB_value, timer_micro_seconds);
    last_encB_value = current_encB_value;

    // Serial.print("Enc A: "); Serial.print(current_encA_value);
    // Serial.print(" Enc B: "); Serial.print(current_encB_value);
    // Serial.println("Updating Speed!");
    Serial.print(" MA: "); Serial.print(m_motor_speed_A);
    Serial.print(" MB: "); Serial.println(m_motor_speed_B);
}

/*
    Controller Math:
    1 Revolution = (16/21) * 1 Revolution = 0.7619 Revolutions of Wheel/1 revolution of Motor
    Therefore, 1 revolution of motor = (0.7691)*(8*pi) = 19.1487 centimeters travelled
*/
static double Main::calculateSpeed(long current_encoder_value, long last_encoder_value, long period) {
    double wheel_circumference = 3.14*8;
    double distancePerWheelTick = wheel_circumference / 341.2; // 341.2 counts/revolution of the wheel
    double distancePerMotorTick = distancePerWheelTick * (16/21.0); // 1 Revolution of the motor = Gear Ratio = 16/21

    // Serial.print("Distance: "); Serial.println(distancePerTick * (current_encoder_value - last_encoder_value));
    // double speed = distancePerTick * (current_encoder_value - last_encoder_value) / (period/1000000.0); // Convert from microseconds to seconds
    long double speed = distancePerMotorTick * ((current_encoder_value - last_encoder_value) / (period / 1000000.0)); // Convert from microseconds to seconds
    return speed;
}

void Main::stopProgram() {
    set_sleep_mode(SLEEP_MODE_PWR_DOWN);
    cli();  // Disable interrupts
    sleep_mode();
}
