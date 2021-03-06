#include "Main.h"

Main::Main() {}

Encoder m_encoder_A(19, 18);
Encoder m_encoder_B(2,3);

Main::Main(MotorPair motor_pair, Imu imu_sensor, Color color_front, Color color_down,
            Ultrasonic ultrasonic_front, Ultrasonic ultrasonic_right, Ultrasonic ultrasonic_left,
            Ultrasonic ultrasonic_back, Controller controller, Encoder encoder_A, Encoder encoder_B) {
    m_motor_pair = motor_pair;
    m_imu_sensor = imu_sensor;
    m_color_front = color_front;
    m_color_down = color_down;
    m_ultrasonic_front = ultrasonic_front;
    m_ultrasonic_right = ultrasonic_right;
    m_ultrasonic_left = ultrasonic_left;
    m_ultrasonic_back = ultrasonic_back;
    m_controller = controller;

    LED::onAndOff();
    LED::onAndOff();
    LED::onAndOff();
    init();
}

void Main::init() {
    Serial.println("Initializing Main Engine...");

    // Start the imu
    m_imu_sensor.begin();
    Serial.println("Imu Started...");

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

    // START COORDINATE
    m_global_map[m_start_coord.row][m_start_coord.col].block_type = PARTICLE;
    m_global_map[m_start_coord.row][m_start_coord.col].searched = true;

    // BLOCK IMMEDIATELY NORTH - Assume these 2 blocks are unsearched
    m_global_map[m_start_coord.row-1][m_start_coord.col].block_type = PARTICLE;
    m_global_map[m_start_coord.row-1][m_start_coord.col].searched = true;

    // initialize global headings
    m_global_north_heading = 0;
    m_global_east_heading = m_global_north_heading + 90;
    m_global_south_heading = m_global_north_heading + 180;
    m_global_west_heading = m_global_north_heading + 270;

    m_sand_blocks_searched = 0;
    // hardcode the start location
    m_map_discovered = true;
    setCorrectMap(potential_map1);
}

// TODO: Add possible fail-safe that saves the last image of the main member function
// and possibly reloads it based off of that value
void Main::run() {
    while (!allTasksCompleted()) {
        Task task = getNextTask();

        if (taskIsMapped(task)) {
            Serial.println("Task is mapped--");
            engageObjectiveMode(task);
        } else {
            Serial.println("Task is not mapped, investigate closest landmark");
            investigateClosestLandmark();
            // engageExploreMode();
        }
    }

    // End program after returning to start
    returnToStart(m_global_map, m_current_pose);
}

bool Main::allTasksCompleted() {
    Serial.println("");
    return (m_found_food &&
            m_found_people &&
            m_found_survivor &&
            m_extinguished_fire);
}

Task Main::getNextTask() {
    if (!m_map_discovered)
        return DISCOVER_MAP;

    if (!m_extinguished_fire) {// Should we take out the fire out immediately ?
        Serial.println("TRY TO EXTINGUISH A FIRE");
        return EXTINGUISH_FIRE;
    }

    Serial.println("INVESTIGATING CLOSEST LANDMARK");
    return INVESTIGATE_CLOSEST_LANDMARK;
}

//TODO: don't use this function, check for closest sand blocks
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

    // MapLocation location_of_interest(UNKNOWN); // Initialize to unknown block
    // Initialize to an "unsearched block"
    MapLocation unsearched_type_location(PARTICLE, false , NONE, false);
    unsearched_type_location.searched = false;

    Orientation finish_ori = DONTCARE;

    Serial.println("Printing global searched map: ");
    printSearchedMap(m_global_map);
    Coord closest_block;

    Coord explore_block = findClosestBlockToInterest(m_global_map, closest_block, unsearched_type_location, m_current_pose.coord, finish_ori);
    Serial.print("Block to explore: "); printPose(Pose(explore_block, finish_ori)); Serial.println("");

    if (!isValid(explore_block)) { // No Path Found
        Serial.println("No other blocks can be explored. RETURNING TO START");
        returnToStart(m_global_map, m_current_pose);
        return;
    }

    travelToBlock(m_global_map, m_current_pose, Pose(explore_block, DONTCARE));
    Serial.println("Mapping Adjacent Blocks...");
    mapAdjacentBlocks(m_global_map, m_current_pose);
}

void Main::engageObjectiveMode(Task task) {
    Serial.println("Enaging Objective Mode w/ Searched Map!");
    printSearchedMap(m_global_map);

    switch (task) {
        case DISCOVER_MAP:
            Serial.println("Discovering Correct Map ...");
            findCorrectMap();
        case EXTINGUISH_FIRE:
            // Assume there is some sort of Path Planning (Adrian) that gets us within
            // the flame sensors range of the candle.
            Serial.println("TASK: Extinguishing Fire");
            travelToFireExtinguishLocation();
            break;
        case INVESTIGATE_CLOSEST_LANDMARK:
            Serial.println("TASK: INVESTIGATING CLOSEST LANDMARK");
            investigateClosestLandmark();
            break;
        default:
            Serial.println("UNKNOWN TASK");
            stopProgram();
            break;
    }
}

void Main::returnToStart(MapLocation global_map[][GLOBAL_COL], Pose current_pose) {
    Serial.println("Travelling back to start");
    travelToBlock(global_map, current_pose, Pose(m_start_coord, DONTCARE));
    Serial.println("Shutting Down Program.");
    stopProgram();
}

void Main::findCorrectMap() {
    MapLocation west_location(UNKNOWN);
    MapLocation north_east_location(UNKNOWN);

    Coord west_block = Coord(m_current_pose.coord.row,m_current_pose.coord.col - 1);
    Coord north_east_block = Coord(m_current_pose.coord.row - 1,m_current_pose.coord.col + 1);



    travelToBlock(m_global_map, m_current_pose, Pose(m_current_pose.coord, WEST));

    moveForwardSetDistance(JITTER_DISTANCE, WEST);
    moveBackwardSetDistance(JITTER_DISTANCE, WEST);
    double start_mag = m_imu_sensor.getMag().z();
    mapBlockTerrainInFront(m_global_map, m_current_pose, start_mag, west_block);
    west_location = m_global_map[west_block.row][west_block.col];

    moveForwardSetDistance(JITTER_DISTANCE, EAST);
    moveBackwardSetDistance(JITTER_DISTANCE, EAST);
    start_mag = m_imu_sensor.getMag().z();

    travelToBlock(m_global_map, m_current_pose, Pose(Coord(m_current_pose.coord.row - 1, m_current_pose.coord.col), EAST));
    mapBlockTerrainInFront(m_global_map, m_current_pose, start_mag, north_east_block);
    north_east_location = m_global_map[north_east_block.row][north_east_block.col];

    // Identify Map
    m_map_discovered = true;
    // TODO: Consider what happens if we detect food in the sand
    if (west_location.block_type == GRAVEL) {
        if (north_east_location.block_type == SAND)
            setCorrectMap(potential_map1);
        else if (north_east_location.block_type == WATER)
            setCorrectMap(potential_map4);
        else {
            Serial.println("UNABLE TO IDENTIFY MAPPING");
            // m_map_discovered = false;
            // engageExploreMode();
        }
    }
    else if (west_location.block_type == WATER) {
        if (north_east_location.block_type == SAND)
            setCorrectMap(potential_map3);
        else if (north_east_location.block_type == GRAVEL)
            setCorrectMap(potential_map2);
        else {
            Serial.println("Water block detected for left location, unknown north east block");
            // engageExploreMode();
        }
    } else {
        Serial.println("UNABLE TO IDENTIFY MAPPING --");
    }

    printMap(m_global_map);

    Serial.println("Map Set");
}

void Main::setCorrectMap(MapLocation map[][GLOBAL_COL]) {
    for (int i = 0; i < GLOBAL_ROW; i ++) {
        for (int j = 0; j < GLOBAL_COL; j++) {
            m_global_map[i][j].block_type = map[i][j].block_type;
            if (m_global_map[i][j].block_type != PARTICLE)
                m_global_map[i][j].searched = true;
        }
    }
}

void Main::investigateClosestLandmark() {
    // Look for closest sandblock
    Serial.println("Investigating closest landmark");

    // Try to find the food first
    // then try to find the peoples, that'll be most efficient
    bool can_travel_to_possible_food = false;

    if (m_sand_blocks_searched >= 3) {
        Serial.println("All sandblocks search, make true");
        m_found_food = true;
    }

    // If the fire was not yet extinguished, extinguish fire
    if (!m_found_food) {
        Serial.println("Food not found yet, FIND food");
        can_travel_to_possible_food = checkClosestSandBlock();
        if (!can_travel_to_possible_food) {
            Serial.println("No possible path to food");
        }
    }

    // Serial.println("Checking to see closest possible landmark");
    // Try to find closest possible landmark
    //POSSIBLY SWITCH The ordering of these states, may not need go to closest possible landmark
    // bool can_travel_to_closest_landmark = gotoClosestPossibleLandmark();
    // if (!can_travel_to_closest_landmark) {
    //     //TODO: Test this next
    //     Serial.println("No path to possible landmark");
    // }

    Serial.println("Engaging Explore mode");

    // Engage Explore mode otherwise
    engageExploreMode();

}

// Returns false if there are no possible paths to any sandblocks as of right now
bool Main::checkClosestSandBlock() {
    MapLocation location_sand_block_type(SAND, true, NONE); // Initialize to unknown block

    Orientation finish_ori = DONTCARE;
    Coord sand_block;
    Coord closest_sand_block_adjacent = findClosestBlockToInterest(m_global_map, sand_block, location_sand_block_type, m_current_pose.coord, finish_ori);
    Serial.print("Closest Sand Block: "); printCoord(sand_block); Serial.println("");
    Serial.print(" Closest adjacent sand Block: "); printCoord(closest_sand_block_adjacent); Serial.println("");

    if (!isValid(closest_sand_block_adjacent)) { // No path found
        // Signal that there are no paths to the closest possible sand block
        Serial.println("Not a valid sand block location");
        return false;
    }

    travelToBlock(m_global_map, m_current_pose, Pose(closest_sand_block_adjacent, finish_ori));

    moveForwardSetDistance(JITTER_DISTANCE, finish_ori);
    moveBackwardSetDistance(JITTER_DISTANCE, finish_ori);
    double start_mag = m_imu_sensor.getMag().z();

    checkForFood(m_global_map, sand_block, start_mag, finish_ori);

    return true;
}

//
bool Main::gotoClosestPossibleLandmark() {
    MapLocation location_possible_land_mark_type(LANDMARK, true, NONE); // Initialize to unknown block
    // location_possible_land_mark_type.possible_land_mark = true;
    location_possible_land_mark_type.searched = false;
    location_possible_land_mark_type.land_mark_spot = true;

    // Should return closest possible landmark location
    Orientation finish_ori = DONTCARE;
    Coord possible_land_mark_block;
    Coord possible_land_mark_adj = findClosestBlockToInterest(m_global_map, possible_land_mark_block,
                                    location_possible_land_mark_type, m_current_pose.coord, finish_ori);
    Serial.print("Possible Landmark: "); printCoord(possible_land_mark_block); Serial.println("");


    if (!isValid(possible_land_mark_block)) {
        return false;
    }

    travelToBlock(m_global_map, m_current_pose, Pose(possible_land_mark_adj, finish_ori));
    checkForLandMark(m_global_map, possible_land_mark_block, m_current_pose);
}

/***********************
* PERIPHERAL FUNCTIONS *
************************/

bool Main::isLandmarkAhead() {
    double distance = m_ultrasonic_front.getDistance();
    Serial.println(distance);
    if (distance < 44)
        return true;
    else
        return false;
}

Landmark Main::identifyLandMark() {
    if (m_color_front.getStructureColor() == 1) {
        LED::onAndOff();
        return SURVIVOR;
    }
    if (m_color_front.getStructureColor() == 2) {
        LED::onAndOff();
        LED::onAndOff();
        return PEOPLE;
    }

    return FIRE;
}

Coord Main::getGlobalPosition(Orientation orientation) {
    double left_distance = m_ultrasonic_left.getDistance() / 30.3;
    Serial.println(left_distance);
    delay(500);
    double right_distance = m_ultrasonic_right.getDistance() / 30.3;
    Serial.println(right_distance);
    delay(500);
    double front_distance = m_ultrasonic_front.getDistance() / 30.3;
    Serial.println(front_distance);
    delay(500);
    double back_distance = m_ultrasonic_back.getDistance() / 30.3;
    Serial.println(back_distance);

    Serial.println(fabs((left_distance + right_distance) - 5));
    if (fabs((left_distance + right_distance) - 5) < 1 && fabs((front_distance + back_distance) - 5) < 1) {
        switch (orientation) {
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
    double front_distance = 0, left_distance = 0, right_distance = 0, back_distance = 0;

    double left_ultrasonic_distance = m_ultrasonic_left.getDistance() / 30.3;
    delay(500);
    double right_ultrasonic_distance = m_ultrasonic_right.getDistance() / 30.3;
    delay(500);
    double front_ultrasonic_distance = m_ultrasonic_front.getDistance() / 30.3;
    delay(500);
    double back_ultrasonic_distance = m_ultrasonic_back.getDistance() / 30.3;

    Serial.print(front_ultrasonic_distance); Serial.print(" ");
    Serial.print(right_ultrasonic_distance); Serial.print(" ");
    Serial.print(back_ultrasonic_distance); Serial.print(" ");
    Serial.print(left_ultrasonic_distance); Serial.println(" ");

    switch (pose.orientation) {
    case NORTH:
        front_distance = front_ultrasonic_distance;
        back_distance = back_ultrasonic_distance;
        right_distance = right_ultrasonic_distance;
        left_distance = left_ultrasonic_distance;
        break;
    case SOUTH:
        front_distance = back_ultrasonic_distance;
        back_distance = front_ultrasonic_distance;
        right_distance = left_ultrasonic_distance;
        left_distance = right_ultrasonic_distance;
        break;
    case EAST:
        front_distance = left_ultrasonic_distance;
        back_distance = right_ultrasonic_distance;
        right_distance = front_ultrasonic_distance;
        left_distance = back_ultrasonic_distance;
        break;
    case WEST:
        front_distance = right_ultrasonic_distance;
        back_distance = left_ultrasonic_distance;
        right_distance = back_ultrasonic_distance;
        left_distance = front_ultrasonic_distance;
        break;
    default:
        Serial.println("UNKNOWN ORIENTATION");
    }
    if(floor(front_distance) <= 5) {
        if(floor(front_distance) != pose.coord.row) {
            global_map[pose.coord.row - (int)ceil(front_distance)][pose.coord.col].land_mark_spot = true;

            for(int i = (pose.coord.row - (int)floor(front_distance)); i < pose.coord.row; i++) {
                global_map[i][pose.coord.col].searched = true;
            }
        } else {
            for(int i = 0; i < pose.coord.row; i++) {
                global_map[i][pose.coord.col].searched = true;
            }
        }
    }

    if(floor(back_distance) <= 5) {
        if(5 - floor(back_distance) != pose.coord.row) {
            global_map[(int)ceil(back_distance) + pose.coord.row][pose.coord.col].land_mark_spot = true;

            for(int i = pose.coord.row + (int)floor(back_distance); i > pose.coord.row; i--) {
                global_map[i][pose.coord.col].searched = true;
            }
        } else {
            for(int i = 5; i > pose.coord.row; i--) {
                global_map[i][pose.coord.col].searched = true;
            }
        }
    }

    if(floor(left_distance) <= 5) {
        if(floor(left_distance) != pose.coord.col) {
            global_map[pose.coord.row][pose.coord.col - (int)ceil(left_distance)].land_mark_spot = true;
            for(int i = pose.coord.col - (int)floor(left_distance); i < pose.coord.col; i++) {
                global_map[pose.coord.row][i].searched = true;
            }
        } else {
            for(int i = 0; i < pose.coord.col; i++) {
                global_map[pose.coord.row][i].searched = true;
            }
        }
    }

    if(floor(right_distance) <= 5) {
        if(5 - floor(right_distance) != pose.coord.col) {
            global_map[pose.coord.row][(int)ceil(right_distance) + pose.coord.col].land_mark_spot = true;
            for(int i = pose.coord.col + (int)floor(right_distance); i > pose.coord.col; i--) {
                global_map[pose.coord.row][i].searched = true;
            }
        } else {
            for(int i = 5; i > pose.coord.col; i--) {
                global_map[pose.coord.row][i].searched = true;
            }
        }
    }
}

void Main::mapAdjacentBlocks(MapLocation (&global_map)[GLOBAL_ROW][GLOBAL_COL], Pose start_pose) {
    // Use motor encoders to measure distance ??
    // Detect adjacent blocks that are undiscovered
    Serial.println("Mapping ");
    // Explore all adjacent blocks
    Pose current_pose = start_pose;
    Pose desired_pose;

    getPossibleLandmarks(global_map, m_current_pose);

    printMap(global_map);
    Serial.println("Searched map");
    printSearchedMap(global_map);
    Serial.println("Got possible landmarks");

    Pose adjacent_blocks[4];
    adjacent_blocks[0] = Pose(Coord(start_pose.coord.row - 1, start_pose.coord.col), NORTH);
    adjacent_blocks[1] = Pose(Coord(start_pose.coord.row, start_pose.coord.col + 1), EAST);
    adjacent_blocks[2] = Pose(Coord(start_pose.coord.row + 1, start_pose.coord.col), SOUTH);
    adjacent_blocks[3] = Pose(Coord(start_pose.coord.row, start_pose.coord.col - 1), WEST);

    for (int i = 0; i < 4; i++) {

        int row = adjacent_blocks[i].coord.row;
        int col = adjacent_blocks[i].coord.col;
        MapLocation map_location = global_map[row][col];

        if (isValid(adjacent_blocks[i].coord) && map_location.block_type == PARTICLE && !map_location.searched) {
            Serial.print("Checking Orientation: "); printOrientation(adjacent_blocks[i].orientation); Serial.println("");
            desired_pose = Pose(start_pose.coord, adjacent_blocks[i].orientation);
            travelToBlock(global_map, current_pose, desired_pose);
            Serial.println("Mapping Block in front");
            current_pose = desired_pose;

            // TODO: add a function moves closer to the landmarks as much as possible
            // mapBlockTerrainInFront(global_map, current_pose, start_mag, adjacent_blocks[i].coord);
            mapBlockLandmarkInFront(global_map, current_pose, adjacent_blocks[i].coord);
            Serial.println("end of map block landmark in front. New Map:");
            printMap(m_global_map);
        }

    }
    Serial.println("Mapped adjacent blocks!");
}

void Main::checkForFood(MapLocation (&global_map)[GLOBAL_ROW][GLOBAL_COL],
                        Coord block_to_map,
                        double start_mag,
                        Orientation orientation) {
    MapLocation& map_location = global_map[block_to_map.row][block_to_map.col];

    Serial.println("Checking for food at "); printCoord(block_to_map);
    map_location.food_searched = true;
    m_sand_blocks_searched ++;

    moveForwardSetDistance(15.0, m_current_pose.orientation);

    moveForwardSetDistance(JITTER_DISTANCE, orientation);
    moveBackwardSetDistance(JITTER_DISTANCE, orientation);
    Serial.print("Start mag: "); Serial.println(start_mag);
    Serial.print(" Mag value "); Serial.println(m_imu_sensor.getMag().z());

    if (map_location.block_type == SAND && isFood(start_mag, orientation)) {
        Serial.print("FOOD FOUND @ "); printCoord(block_to_map); Serial.println("!!!!");
        m_found_food = true;
        m_food_location = block_to_map;

        LED::onAndOff();

        // At this point, we found the food, now we want to see if we've already found the people
        // if we've already found the group of people, then plan to deliver the food to
        // that group of people straight away
        if (m_found_people) {
            m_deliver_food_to_group = true;
        }
    } else {
        Serial.println("FOOD NOT FOUND, KEEP LOOKING..");
    }

    moveBackwardSetDistance(15.0, m_current_pose.orientation);
}

void Main::checkForLandMark(MapLocation (&global_map)[GLOBAL_ROW][GLOBAL_COL],
                            Coord block_to_map,
                            Pose pose) {

    Serial.println("checkForLandMark()");
    MapLocation& map_location = global_map[block_to_map.row][block_to_map.col];

    map_location.land_mark_spot = isLandmarkAhead();
    map_location.searched = true;

    if (map_location.land_mark_spot) {
        map_location.landmark = identifyLandMark();
        map_location.block_type = LANDMARK;

        // SET block type as untraversable i.e. landmark
        if (map_location.landmark == SURVIVOR) {
            m_survivor_location = block_to_map;

            // Mark Landmark location complete
            m_survivor_mapped = true;
            m_found_survivor = true;
            Serial.print("SURVIVOR LOCATION: "); printCoord(m_people_location); Serial.println("");
        }
        else if (map_location.landmark == PEOPLE) {
            m_people_location = block_to_map;

            m_group_mapped = true;
            m_found_people = true;

            m_deliver_food_to_group = false;
            Serial.print("PEOPLE LOCATION: "); printCoord(m_people_location); Serial.println("");
        }
        else { // Marking Landmark as FIRE
            m_fire_mapped = true;
            m_fire_location = block_to_map;
            Serial.print("FIRE LOCATION: "); printCoord(m_people_location); Serial.println("");
        }
    }

    Serial.println("checkForLandMark() end ");
}

// This funciton will only check the particle boards and never the sand blocks
void Main::mapBlockLandmarkInFront(MapLocation (&global_map)[GLOBAL_ROW][GLOBAL_COL], Pose pose, Coord block_to_map) {
    // MapLocation testGrid[GLOBAL_ROW][GLOBAL_COL] =
    // {
    // //    0, 1, 2, 3, 4, 5
    //     { MP, MP, MP, MG, MP, MP}, // 0
    //     { MP, MS, MP, MP, MW, MP}, // 1
    //     { MW, MP, MP, MS, MP, MP}, // 2
    //     { MP, MP, MP, MP, MP, MG}, // 3
    //     { MP, MG, MP, MP, MS, MP}, // 4
    //     { MP, MP, MW, MP, MP, MP}  // 5
    // };

    Serial.println("Map block landmark in front...");

    delay(1000);
    double dist_to_possible_landmark = m_ultrasonic_front.getDistance();
    double dist_to_travel = dist_to_possible_landmark - (5 + FRONT_ARM_LENGTH);
    Serial.print("Dist to travel: "); Serial.println(dist_to_travel);

    moveForwardSetDistance(dist_to_travel, pose.orientation);

    checkForLandMark(global_map, block_to_map, pose);

    moveBackwardSetDistance(dist_to_travel, pose.orientation);
}

void Main::mapBlockTerrainInFront(MapLocation (&global_map)[GLOBAL_ROW][GLOBAL_COL], Pose pose, double start_mag, Coord block_to_map) {

    double start_distance = m_ultrasonic_front.getDistance();

    // Get Distance to nearest boundary

    // TODO: Implement distance based
    double distance_to_edge_block = start_distance - 7.0;
    moveForwardSetDistance(8.0, m_current_pose.orientation);
    m_motor_pair.stop();

    delay(500);
    // TODO: Get feedback to see if the terrain color is unknown, then keep trying
    BlockType terrain_type = UNKNOWN;
    terrain_type = m_color_down.getTerrainColor();
    while (terrain_type == UNKNOWN) {
        Serial.print("Terrain Type Detected: "); printBlocktype(terrain_type); Serial.println("");
        terrain_type = m_color_down.getTerrainColor();
        global_map[block_to_map.row][block_to_map.col].block_type = terrain_type;
    }

    delay(1000);

    moveBackwardSetDistance(9.0, m_current_pose.orientation);
}

bool Main::isFood(double current_mag, Orientation orientation) {
    moveForwardSetDistance(JITTER_DISTANCE, orientation);
    moveBackwardSetDistance(JITTER_DISTANCE, orientation);

    double new_mag = m_imu_sensor.getMag().z();

    return (fabs(fabs(new_mag) - fabs(current_mag)) > 40);
}

/***********************
*    TASK FUNCTIONS    *
************************/

void Main::travelToFood(MapLocation global_map[][GLOBAL_COL], Pose current_pose) {
    MapLocation location_of_interest(SAND, true, FOOD); // Initialize to unknown block
    Orientation finish_ori = DONTCARE;
    Coord food_block; // interest
    Coord explore_block = findClosestBlockToInterest(m_global_map, food_block, location_of_interest, m_current_pose.coord, finish_ori);
    travelToBlock(m_global_map, m_current_pose, Pose(m_food_location, finish_ori));

    // Indicate that you've found the food
    // blink LED
}

//TODO: Possibly delete this function
// Coord Main::getClosestSandBlock(MapLocation global_map[][GLOBAL_COL], Coord current_loc) {
//     int min_distance = INT_MAX;
//     Coord closest_sand_block = Coord(-1, -1); // Error coordinate
//
//     for (int i = 0; i < GLOBAL_ROW; i++) {
//         for (int j = 0; j < GLOBAL_COL; j++) {
//             // Check to see if the block is a SAND block and that
//             // it has not already been searched
//             if (global_map[i][j].block_type == SAND && global_map[i][j].searched == false) {
//                 int distance = getManhattanDistance(Coord(i,j), current_loc);
//
//                 if (distance < min_distance) {
//                     min_distance = distance;
//                     closest_sand_block = Coord(i,j);
//                 }
//             }
//         }
//     }
//     return closest_sand_block;
// }

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
        !isValid(shortest_path.top())) {
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
            Serial.print("Neighbor matches condition!"); printCoord(adjacent[i].coord); Serial.println("");
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
    // UPDATE: This function will almost never be used
    if (location_of_interest.block_type == UNKNOWN) {
        return (global_map[coord.row][coord.col].block_type == UNKNOWN);
    }

    if (location_of_interest.block_type == SAND) {
        return (global_map[coord.row][coord.col].food_searched == false &&
                global_map[coord.row][coord.col].block_type == SAND);
    }

    // OBJECTIVE MODE | Block w/ location of interest
    if (location_of_interest.land_mark_spot) {
        // return (global_map[coord.row][coord.col].landmark == location_of_interest.landmark);
        return (global_map[coord.row][coord.col].land_mark_spot == location_of_interest.land_mark_spot &&
                global_map[coord.row][coord.col].searched == false);
    }

    // UPDATE: New mapping function to make sure that this undiscovered
    // TODO: Possibly update the m_global_map to already have filtered the gravel/sandblocks
    // to not have possible landmarks
    if (location_of_interest.searched == false) {
        printCoord(coord); Serial.println("");
        return (global_map[coord.row][coord.col].searched == false);
    }

    return false;
}

Coord Main::findClosestBlockToInterest(MapLocation global_map[][GLOBAL_COL],
                                       Coord& closest_block,
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

        // Found a coord with unknown neighbors
        if (hasMatchingNeighbors(global_map, location_of_interest, current, dir_towards)) {
            if (dir_towards == NORTH){
                closest_block = Coord(current.row - 1, current.col);
            } else if (dir_towards == EAST) {
                closest_block = Coord(current.row, current.col + 1);
            } else if (dir_towards == SOUTH) {
                closest_block = Coord(current.row + 1, current.col);
            } else if (dir_towards == WEST) {
                closest_block = Coord(current.row, current.col - 1);
            } else {
                Serial.print("Error in getting the closest location");
            }

            return current;
        }

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
	double start_distance = m_ultrasonic_front.getDistance();
    double start_heading = m_imu_sensor.getEuler().x();

    Serial.print("Start Distance: "); Serial.print(start_distance);
    Serial.print(" Start Heading: "); Serial.println(start_heading);

	double end_distance = start_distance - distance;

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

void Main::moveForwardSetDistance(double distance, Orientation orientation) {
    // Using encoders
    long start_tick_a = m_encoder_A.read();
    long start_tick_b = m_encoder_B.read();

    // double start_heading = m_imu_sensor.getEuler().x();
    double start_heading = getTargetHeadingForOrientation(orientation);

    double relative_start_distance = m_ultrasonic_front.getDistance();
    double end_distance = relative_start_distance - distance;

    // Validate Ultrasonic readings, use a stable value
    bool break_out_of_loop = false;

    // while (m_ultrasonic_front.getDistance() > end_distance) {
    //     Serial.print("Ultrasonic Distance to travel: "); Serial.println(m_ultrasonic_front.getDistance() - end_distance);
        // double dist_to_travel = (m_ultrasonic_front.getDistance() - end_distance)/distance_per_tick;
        double dist_to_travel = distance/distance_per_tick;

        // Ensure that it doesn't travel more than the set distance, set tolerance of 2 cms
        if ((m_ultrasonic_front.getDistance() - end_distance) > distance + 2.0) {
            Serial.println("Unusable Ultra-sonic reading, default to encoder based tracking");
            long dist_to_travel = distance/distance_per_tick;
            break_out_of_loop = true;
        }

        // Track each wheel separately
        // Map distance to number of ticks that need to be range
        // Encoder A and B should travel the same number of ticks
        while (abs(m_encoder_A.read() - start_tick_a) < dist_to_travel && abs(m_encoder_B.read() - start_tick_b) < dist_to_travel) {
            // Serial.println(abs(m_encoder_B.read()));
            m_controller.driveStraightController(start_heading, m_imu_sensor.getEuler().x(), 160);
        }
        // if (break_out_of_loop)
        //     break;
    // }
    Serial.print("Moved forward "); Serial.print(distance); Serial.println(" cm.");

    m_motor_pair.stop();
}

void Main::moveBackwardSetDistance(double distance, Orientation orientation) {
    // Using encoders
    long start_tick_a = m_encoder_A.read();
    long start_tick_b = m_encoder_B.read();

    // double start_heading = m_imu_sensor.getEuler().x();
    double start_heading = getTargetHeadingForOrientation(orientation);

    long distance_in_ticks = distance/distance_per_tick;

    double relative_start_distance = m_ultrasonic_back.getDistance();
    double end_distance = relative_start_distance - distance;

    // Validate Ultrasonic readings, use a stable value

    while (abs(end_distance - m_ultrasonic_back.getDistance()) >= 1.5) {
        double dist_to_travel = (m_ultrasonic_back.getDistance() - end_distance)/distance_per_tick;
        Serial.print("New Distance to travel: "); Serial.println(dist_to_travel);
        // Track each wheel separately
        // Map distance to number of ticks that need to be range
        // Encoder A and B should travel the same number of ticks
        while (abs(m_encoder_A.read() - start_tick_a) < distance_in_ticks && abs(m_encoder_B.read() - start_tick_b) < distance_in_ticks) {
            // Serial.println(abs(m_encoder_A.read() - start_tick_a));
            // Serial.println(abs(m_encoder_B.read()));
            m_controller.driveStraightController(start_heading, m_imu_sensor.getEuler().x(), -220);
        }
        break;
        Serial.print("Error: ");Serial.println(m_global_north_heading - m_imu_sensor.getEuler().x());
    }

    m_motor_pair.stop();
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

    // Serial.print(heading_change); Serial.print(" "); Serial.println(heading_position);

    if (abs(heading_change) <= 2 && abs(heading_position) <= 2)
        return true;

    last_heading = current_heading;
    return false;
}

double Main::getTargetHeadingForOrientation(Orientation orientation) {
    if (orientation == NORTH)
        return m_global_north_heading;
    if (orientation == EAST)
        return m_global_east_heading;
    if (orientation == SOUTH)
        return m_global_south_heading;
    if (orientation == WEST)
        return m_global_west_heading;

    // Orientation: DONTCARE
    return m_global_north_heading;
}

void Main::turnLeft(Orientation target_orientation) {

    double end_heading = getTargetHeadingForOrientation(target_orientation);
    double start_headings = m_imu_sensor.getEuler().x();
	// double end_heading = start_headings - 90;
    // if (end_heading < 0) end_heading += 360;

    Serial.print("Start Heading: "); Serial.print(start_headings);
    Serial.print(" End Heading: "); Serial.println(end_heading);
    double last_heading = start_headings + 5; //TODO: Offset from heading change

    int wait_counter = 0;
    // Add a counter that breaks it if
    while (!isStabilized(last_heading, m_imu_sensor.getEuler().x(), end_heading) && wait_counter <= 600) {
        m_controller.turnLeftController(end_heading, m_imu_sensor.getEuler().x(), 200);
        wait_counter ++;
    }

    m_motor_pair.stop();
}

void Main::turnRight(Orientation target_orientation) {

    double end_heading = getTargetHeadingForOrientation(target_orientation);

    double start_heading = m_imu_sensor.getEuler().x();

    Serial.print("Start Heading: "); Serial.print(start_heading);
    Serial.print(" End Heading: "); Serial.println(end_heading);

    double last_heading = start_heading - 5; //TODO: Offset from heading change
    int wait_counter = 0;
    while (!isStabilized(last_heading, m_imu_sensor.getEuler().x(), end_heading) && wait_counter <= 1000) {
        m_controller.turnRightController(end_heading, m_imu_sensor.getEuler().x(), 200);
        wait_counter ++;
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

void Main::travelToFireExtinguishLocation() {
    // Mark fire extinguished locations as (2,2) (2,3), (3,2), (3,3)
    Serial.println("Travelling to fire extinguishing location");
    Coord fire_extinguished_locs[4];

    fire_extinguished_locs[0] = Coord(3,3);
    fire_extinguished_locs[1] = Coord(3,2);
    fire_extinguished_locs[2] = Coord(2,3);
    fire_extinguished_locs[3] = Coord(2,2);

    bool can_travel_to_fire_extinguish_locations = false;
    Coord fire_extinguish_loc;
    for (int i = 0; i < 4; i++) {
        // findShortestPath (?)
        Stack<Coord> possible_path = PathPlanning::findShortestPath(m_global_map, m_current_pose.coord, fire_extinguished_locs[i]);

        if (isValid(possible_path.top())) {
            Serial.println("Possible travel location found"); printCoord(possible_path.top()); Serial.println("");
            can_travel_to_fire_extinguish_locations = true;
            fire_extinguish_loc = fire_extinguished_locs[i];
            break;
        }
    }

    // If there is a path to a fire extinguish location, then travel to it, otherwise, just explore
    if (can_travel_to_fire_extinguish_locations) {
        Serial.println("CAN travel to fire");
        travelToBlock(m_global_map, m_current_pose, Pose(fire_extinguish_loc, DONTCARE));
        findFire();
    } else {
        Serial.println("Can't travel to a fire extinguishing location");
        engageExploreMode();
    }
    Serial.println("End of travelToFireExtinguishLocation");
}

void Main::findFire() {
    Serial.println("Finding Fire");

    double start_heading = getTargetHeadingForOrientation(m_current_pose.orientation);
    double end_heading = start_heading;
    double last_heading = start_heading + 5;

    int wait_counter = 0;
    long start = millis();
    while (Flame::getFireMagnitude() <= 2) {
        // Serial.print("Flame: "); Serial.println(Flame::getFireMagnitude());
        // Serial.print(wait_counter); Serial.print(" "); Serial.println(abs(m_imu_sensor.getEuler().x() - getTargetHeadingForOrientation(m_current_pose.orientation)));

        if (millis() - start >= 4000 && (abs(m_imu_sensor.getEuler().x() - getTargetHeadingForOrientation(m_current_pose.orientation)) <= 2.5 ||
                                        abs(m_imu_sensor.getEuler().x() - getTargetHeadingForOrientation(m_current_pose.orientation)) >= 358)) {
            Serial.println(millis() - start);
            Serial.println("Breaking");
            break;
        }
        MotorPair::setMotorAPWM(-250);
        MotorPair::setMotorBPWM(220);

    }

    Serial.println("Fire Detected, Stop Motors");
    m_motor_pair.stop();


    while (Flame::getFireMagnitude() > 0) {
        extinguishFire();
        wait_counter ++;
    }

    m_motor_pair.stop();

    while (abs(m_imu_sensor.getEuler().x() - getTargetHeadingForOrientation(m_current_pose.orientation)) >= 2.5) {
        // Serial.println(abs(m_imu_sensor.getEuler().x() - getTargetHeadingForOrientation(m_current_pose.orientation)));
        MotorPair::setMotorAPWM(-250);
        MotorPair::setMotorBPWM(220);
    }

    m_motor_pair.stop();

    Serial.println("Fire Extinguished.");
}

void Main::extinguishFire() {
    LED::on();
    while (Flame::getFireMagnitude() > 0) {
        Fan::on();

        delay(3000);

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
    delay(500);
    Orientation current_orientation = orientation;

    Serial.println("Executing Instructions...");
    while (!instructions.empty()) {
        Instruction ins = instructions.front();
        instructions.pop();

        if (ins == MOVE_FORWARD) {
            Serial.print("FORWARDS->");
            // moveForwardOneBlock(30.0);
            moveForwardSetDistance(30.0, current_orientation);
            delay(2000);
        }
        else if (ins == MOVE_BACKWARD) {
            Serial.print("BACKWARDS->");
            // Map -> moveBackward()
            // moveBackwardOneBlock();
            moveBackwardSetDistance(30.0, current_orientation);
            delay(2000);
        }
        else if (ins == ROTATE_RIGHT) {
            Serial.print("RIGHT->");

            current_orientation = (current_orientation + 1) % 4;
            turnRight(current_orientation);
            delay(2000);
        }
        else if (ins == ROTATE_LEFT) {
            Serial.print("LEFT->");
            // Tuned compensation
            // main_engine.moveBackwardSetDistance(3.5, NORTH);
            // main_engine.turnLeft(WEST);
            // main_engine.moveBackwardSetDistance(6.8, WEST);
            // moveBackwardSetDistance(3.5, current_orientation);
            current_orientation = (current_orientation - 1);
            if (current_orientation < 0) current_orientation = 3;
            turnLeft(current_orientation);
            delay(2000);
            // moveBackwardSetDistance(6.8, current_orientation);
            // MotorPair::setMotorAPWM(-255);
            // MotorPair::setMotorBPWM(223);
            // delay(1000);
            // MotorPair::stop();
        }

    }
    Serial.println("");
    orientation = current_orientation;
    Serial.print("Last Orientation: "); printOrientation(orientation); Serial.println("");
}

void Main::runSearchInPlace() {
    // Search for a block in the row/col that is unsearched
    // possible that the back ultrasonic sensor is mounted too high and won't get reliable data
    bool perform_search = false;
    for (int i = m_current_pose.coord.row; i < GLOBAL_ROW; i++) {
        if (m_global_map[i][m_current_pose.coord.col].searched == false) {
            perform_search = true;
            break;
        }
    }

    for (int j = m_current_pose.coord.col; j < GLOBAL_COL; j++) {
        if (m_global_map[m_current_pose.coord.row][j].searched == false) {
            perform_search = true;
            break;
        }
    }

    if (perform_search)
        getPossibleLandmarks(m_global_map, m_current_pose);
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
