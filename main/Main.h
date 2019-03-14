#ifndef Main_h
#define Main_h

#include "Core.h"

#include "PathPlanning.h"
#include "Controller.h"
#include "Color.h"
#include "Imu.h"
#include "MotorPair.h"
#include "Ultrasonic.h"

#include <Arduino.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>

/* Core Class */
class Main {
    public:
        Main();
        Main(MotorPair motor_pair, Imu imu_sensor, Color color_front, Color color_down,
            Ultrasonic ultrasonic_front, Ultrasonic ultrasonic_right, Ultrasonic ultrasonic_left,
            Ultrasonic ultrasonic_back);
        void init();
        void run();
        void returnToStart(MapLocation global_map[][GLOBAL_COL], Pose current_pose);
        void completeNextTask();

        Coord getGlobalPosition();
        void mapAdjacentBlocks(MapLocation (&global_map)[GLOBAL_ROW][GLOBAL_COL], Pose start_pose);
        bool isUnexplored(MapLocation global_map[][GLOBAL_COL], Coord coord);
        void mapTerrainOfBlockInFront();

        //TODO: Implement these functions
        void findFood(MapLocation global_map[][GLOBAL_COL], Pose current_pose);
        Coord getClosestSandBlock(MapLocation global_map[][GLOBAL_COL], Coord current_loc); //TODO: Implement

        void extinguishFire(); //TODO: Implement

        void travelToBlock(MapLocation map[][GLOBAL_COL], Pose start_pose, Pose finish_pose);

        void updateLocation(); //TODO: Implement

        int getManhattanDistance(Coord c1, Coord c2);

        bool hasUnknownNeighbors(MapLocation global_map[][GLOBAL_COL], Coord start_loc);

        Coord findClosestBlockWithUnknownNeighbors(MapLocation grid[][GLOBAL_COL], Coord start_loc);

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
        // bool m_found_food;
        // bool m_found_people;
        // bool m_found_survivor;
        // bool m_extinguished_fire;

        MotorPair m_motor_pair;
        Imu m_imu_sensor;
        Color m_color_front;
        Color m_color_down;
        Ultrasonic m_ultrasonic_front;
        Ultrasonic m_ultrasonic_right;
        Ultrasonic m_ultrasonic_left;
        Ultrasonic m_ultrasonic_back;
};

#endif
