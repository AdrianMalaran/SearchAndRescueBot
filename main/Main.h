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

#include "Encoder.h"

#include <Arduino.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>

/*
    TODO: Add Localization + Speed Controller
*/
/* Core Class */
class Main {
    public:
        Main();
        Main(MotorPair motor_pair, Imu imu_sensor, Color color_front, Color color_down,
            Ultrasonic ultrasonic_front, Ultrasonic ultrasonic_right, Ultrasonic ultrasonic_left,
            Ultrasonic ultrasonic_back, Controller controller);
        void init();
        void run();
        void returnToStart(MapLocation global_map[][GLOBAL_COL], Pose current_pose);
        void engageExploreMode();
        void engageObjectiveMode(Task task);

        bool allTasksCompleted();
        Task getNextTask();
        bool taskIsMapped(Task task);

        bool isLandmarkAhead(MapLocation &location, Pose start_pose);
        Coord getGlobalPosition(Pose pose);
        void mapAdjacentBlocks(MapLocation (&global_map)[GLOBAL_ROW][GLOBAL_COL], Pose start_pose);
        bool isUnexplored(MapLocation global_map[][GLOBAL_COL], Coord coord);
        void mapBlockInFront(MapLocation &location, Pose pose, double start_mag);
        bool isFood(double mag);

        //TODO: Implement these functions
        void findFood(MapLocation global_map[][GLOBAL_COL], Pose current_pose);
        Coord getClosestSandBlock(MapLocation global_map[][GLOBAL_COL], Coord current_loc); //TODO: Implement

        void extinguishFire();

        void travelToBlock(MapLocation map[][GLOBAL_COL], Pose start_pose, Pose finish_pose);

        void updateLocation(); //TODO: Implement

        int getManhattanDistance(Coord c1, Coord c2);

        bool hasUnknownNeighbors(MapLocation global_map[][GLOBAL_COL], Coord start_loc);
        Coord findClosestBlockWithUnknownNeighbors(MapLocation grid[][GLOBAL_COL], Coord start_loc);

        bool hasMatchingNeighbors(MapLocation global_map[][GLOBAL_COL], MapLocation location_of_interest, Coord start_loc);
        Coord findClosestBlockToInterest(MapLocation global_map[][GLOBAL_COL],
            MapLocation location_of_interest,
            Coord start_loc);
        bool neighborMatchesCondition(MapLocation global_map[][GLOBAL_COL], MapLocation location_of_interest, Coord coord);

        void stopProgram();
    private:
        MapLocation m_global_map[GLOBAL_ROW][GLOBAL_COL];

        Queue<Task> tasks;

        Coord m_start_coord;
        Coord m_current_location;
        Orientation m_current_orientation;

        Pose m_current_pose;

        int m_sand_block_counts; // TODO: Increment counter when mapping
        Coord m_sand_block_locations[36];

        // TODO: Add these flags
        bool m_found_food;
        bool m_found_people;
        bool m_found_survivor;
        bool m_delivered_food;
        bool m_extinguished_fire;

        bool m_food_mapped;
        bool m_group_mapped;
        bool m_survivor_mapped;

        Coord m_food_location;
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
