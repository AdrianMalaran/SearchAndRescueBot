#ifndef Main_h
#define Main_h

#include "Core.h"

#include "PathPlanning.h"
#include "Controller.h"
#include "Color.h"
#include "Imu.h"
#include "MotorPair.h"
#include "Ultrasonic.h"
#include "LED.h"

#include <Arduino.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>

#include <TimerOne.h>

/* Core Class */
class Main {
    public:
        Main();
        Main(MotorPair motor_pair, Imu imu_sensor, Color color_front, Color color_down,
            Ultrasonic ultrasonic_front, Ultrasonic ultrasonic_right, Ultrasonic ultrasonic_left,
            Ultrasonic ultrasonic_back, Controller controller, Encoder encoder_A, Encoder encoder_B);
        void init();
        void run();
        void returnToStart(MapLocation global_map[][GLOBAL_COL], Pose current_pose);
        void engageExploreMode();
        void engageObjectiveMode(Task task);

        bool allTasksCompleted();
        Task getNextTask();
        bool taskIsMapped(Task task);

        double getTargetHeadingForOrientation(Orientation orientation);
        bool isLandmarkAhead();
        void checkForLandMark(MapLocation (&global_map)[GLOBAL_ROW][GLOBAL_COL], Coord block_to_map, Pose pose);
        Coord getGlobalPosition(Orientation orientation);
        void getPossibleLandmarks(MapLocation (&global_map)[GLOBAL_ROW][GLOBAL_COL], Pose pose);
        void mapAdjacentBlocks(MapLocation (&global_map)[GLOBAL_ROW][GLOBAL_COL], Pose start_pose);

        void mapBlockTerrainInFront(MapLocation (&global_map)[GLOBAL_ROW][GLOBAL_COL], Pose pose, double start_mag, Coord block_to_map);
        void mapBlockLandmarkInFront(MapLocation (&global_map)[GLOBAL_ROW][GLOBAL_COL], Pose pose, Coord block_to_map);
        void runSearchInPlace();
        bool isFood(double mag, Orientation orientation);
        Landmark identifyLandMark();

        bool isStabilized(double& last_heading, double current_heading, double end_heading);

        void findCorrectMap();
        void setCorrectMap(MapLocation map[][GLOBAL_COL]);

        void travelToFood(MapLocation global_map[][GLOBAL_COL], Pose current_pose);
        Coord getClosestSandBlock(MapLocation global_map[][GLOBAL_COL], Coord current_loc); //TODO: Implement

        void investigateClosestLandmark();
        bool checkClosestSandBlock();
        bool gotoClosestPossibleLandmark();
        void checkForFood(MapLocation (&global_map)[GLOBAL_ROW][GLOBAL_COL], Coord block_to_map, double start_mag, Orientation orientation);

        void travelToFireExtinguishLocation();
        void findFire();
        void extinguishFire();

        void travelToBlock(MapLocation map[][GLOBAL_COL], Pose start_pose, Pose finish_pose);

        /* Localization */
        void updateLocation(Pose finish_pose);

        int getManhattanDistance(Coord c1, Coord c2);

        Coord findClosestBlockToInterest(MapLocation global_map[][GLOBAL_COL], Coord& closest_block, MapLocation location_of_interest, Coord start_loc, Orientation &dir_towards);
        bool hasMatchingNeighbors(MapLocation global_map[][GLOBAL_COL], MapLocation location_of_interest, Coord start_loc, Orientation &dir_towards);
        bool neighborMatchesCondition(MapLocation global_map[][GLOBAL_COL], MapLocation location_of_interest, Coord coord);

        // Using Ultrasonic readings, Heading controller
        void moveForwardOneBlock(double distance);
        // Using Encoder readings
        void moveForwardSetDistance(double distance, Orientation orientation);
        void moveBackwardSetDistance(double distance, Orientation orientation);

        // Debuggic Function
        void moveMotorB(int speed);
        void turnLeft(Orientation target_orientation);
        void executeInstructions(Queue<Instruction> instructions, Orientation& orientation);
        void turnRight(Orientation target_orientation);

        double getCurrentOrientation();

        static void updateActualSpeed();
        static double calculateSpeed(long current_encoder_value, long last_encoder_value, long period);

        void stopProgram();

        double m_global_north_heading;
        double m_global_east_heading;
        double m_global_south_heading;
        double m_global_west_heading;

        MapLocation m_global_map[GLOBAL_ROW][GLOBAL_COL];

    private:
        Coord m_start_coord;
        Pose m_current_pose; // updatePose

        int m_sand_block_counts; // TODO: Increment counter when mapping
        Coord m_sand_block_locations[36];

        double true_north;
        double true_south;
        double true_west;
        double true_east;

        // TODO: Add these flags
        bool m_map_discovered;
        bool m_extinguished_fire;
        bool m_found_food;
        bool m_found_people;
        bool m_found_survivor;

        bool m_deliver_food_to_group;

        bool m_food_mapped;
        bool m_fire_mapped; // May not be necessary
        bool m_group_mapped;
        bool m_survivor_mapped;

        int m_sand_blocks_searched;

        Coord m_food_location;
        Coord m_fire_location; // May not be necessary
        Coord m_people_location;
        Coord m_survivor_location;

        MotorPair m_motor_pair;
        Imu m_imu_sensor;

        Color m_color_front;
        Color m_color_down;

        Ultrasonic m_ultrasonic_front;
        Ultrasonic m_ultrasonic_right;
        Ultrasonic m_ultrasonic_left;
        Ultrasonic m_ultrasonic_back;

        Controller m_controller;
};

#endif
