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
/*
    PRIORITY LIST:
    - Add motor encoders
    - Get point turn working (TURN LEFT/TURN RIGHT)

    PERIPHERAL
    - Localization function
    - Validating/Invalidating the ultra-sonic values

    - Validate electrical components:
        - Ultrasonic sensors
        - Color sensors
        - IMU
        - Motors
        - Motor Encoders
        - LED

    TODO:
    - Add Localization (when lost, use ultrasonic sensors to reorient oneself)
    - Add Speed Controller (requires motor encoders)
    - Add a getHeading() to fix itself to before driving straight, and thats our desired direction
    - Possibly add fail-safe if the current heading is necessary (TEST to see if the heading resets for some reason)
    - Add an initialization function that orients true north, east, south, west within init() function
    - Define locomotion to work with driveStraight(), turnLeft(), turnRight()
    - For findClosestBlockToInterest, augment it to return the heading to where its supposed to get
*/

const int encoderAPin1;
const int encoderAPin2;
const int encoderBPin1;
const int encoderBPin2;

// Global Variables for encoders
extern double m_motor_speed_A;
extern double m_motor_speed_B;
extern long last_encA_value;
extern long last_encB_value;
extern long timer_micro_seconds;  // every 1 millisecond

/* Core Class */
class Main {
    public:
        Main();
        Main(MotorPair motor_pair, Imu imu_sensor, Color color_front, Color color_down,
            Ultrasonic ultrasonic_front, Ultrasonic ultrasonic_right, Ultrasonic ultrasonic_left,
            Ultrasonic ultrasonic_back, Controller controller, Encoder& encoder_A, Encoder& encoder_B);
        void init();
        void run();
        void returnToStart(MapLocation global_map[][GLOBAL_COL], Pose current_pose);
        void engageExploreMode();
        void engageObjectiveMode(Task task);

        bool allTasksCompleted();
        Task getNextTask();
        bool taskIsMapped(Task task);

        void deliverFoodToGroup();
        bool isLandmarkAhead(MapLocation &location, Pose start_pose);
        void checkForLandMark(MapLocation (&global_map)[GLOBAL_ROW][GLOBAL_COL], Coord block_to_map, double start_mag, Pose pose);
        Coord getGlobalPosition(Pose pose);
        void getPossibleLandmarks(MapLocation (&global_map)[GLOBAL_ROW][GLOBAL_COL], Pose pose);
        void mapAdjacentBlocks(MapLocation (&global_map)[GLOBAL_ROW][GLOBAL_COL], Pose start_pose);
        bool isUnexplored(MapLocation global_map[][GLOBAL_COL], Coord coord);
        void mapBlockInFront(MapLocation (&global_map)[GLOBAL_ROW][GLOBAL_COL], Pose pose, double start_mag, Coord block_to_map);
        bool isFood(double mag);

        bool isStabilized(double& last_heading, double current_heading, double end_heading);

        void findCorrectMap();
        void setCorrectMap(MapLocation map[][GLOBAL_COL]);

        //TODO: Implement these functions
        void travelToFood(MapLocation global_map[][GLOBAL_COL], Pose current_pose);
        Coord getClosestSandBlock(MapLocation global_map[][GLOBAL_COL], Coord current_loc); //TODO: Implement

        void findFire();
        void extinguishFire();

        void travelToBlock(MapLocation map[][GLOBAL_COL], Pose start_pose, Pose finish_pose);

        /* Localization */
        void updateLocation(Pose finish_pose);

        int getManhattanDistance(Coord c1, Coord c2);

        Coord findClosestBlockToInterest(MapLocation global_map[][GLOBAL_COL], MapLocation location_of_interest, Coord start_loc, Orientation &dir_towards);
        bool hasMatchingNeighbors(MapLocation global_map[][GLOBAL_COL], MapLocation location_of_interest, Coord start_loc, Orientation &dir_towards);
        bool neighborMatchesCondition(MapLocation global_map[][GLOBAL_COL], MapLocation location_of_interest, Coord coord);

        // Using Ultrasonic readings, Heading controller
        void moveForwardOneBlock(double distance);
        // Using Encoder readings
        void moveForwardSetDistance(double distance);
        // Using Speed Control
        void moveForwardSpeedControl();

        // Debuggic Function
        void moveMotorB(int speed);
        void turnLeft();
        void executeInstructions(Queue<Instruction> instructions, Orientation& orientation);
        void turnRight();

        double getCurrentOrientation();

        static void updateActualSpeed();
        static double calculateSpeed(long current_encoder_value, long last_encoder_value, long period);

        void stopProgram();
    private:
        MapLocation m_global_map[GLOBAL_ROW][GLOBAL_COL];

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
        Encoder m_encoder_A;
        Encoder m_encoder_B;
};

#endif
