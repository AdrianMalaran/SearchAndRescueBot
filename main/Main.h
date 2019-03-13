#ifndef Main_h
#define Main_h

#include "Core.h"
#include "PathPlanning.h"
#include "Controller.h"
#include "Color.h"
#include "Imu.h"

#include <Arduino.h>

/* Core Class */
class Main {
    public:
        Main();
        void init();
        void run();
        void returnToStart();
        void completeNextTask();

        void mapAdjacentBlocks(MapLocation *global_map[][GLOBAL_COL], Coord current_loc);
        bool isUnexplored(MapLocation global_map[][GLOBAL_COL], Coord coord);
        void mapTerrainOfBlockInFront();

        bool isValid(int row, int col); // TODO: Duplicate function

        //TODO: Implement these functions
        void findFood(MapLocation global_map[][GLOBAL_COL], Coord current_loc);
        Coord getClosestSandBlock(MapLocation global_map[][GLOBAL_COL], Coord current_loc); //TODO: Implement

        void extinguishFire(); //TODO: Implement

        void Main::travelToBlock(MapLocation map[][GLOBAL_COL], Coord current_loc, Coord dest,
                                Orientation start_ori, Orientation finish_ori);

        void updateLocation(); //TODO: Implement

        int getManhattanDistance(Coord c1, Coord c2);
    private:
        MapLocation global_map[GLOBAL_ROW][GLOBAL_COL];

        Queue<Task> tasks;

        Coord m_start_coord;
        Coord m_current_location;
        Orientation m_current_orientation;

        int m_sand_block_counts; // TODO: Increment counter when mapping
        Coord m_sand_block_locations[36];

        // TODO: Add these flags
        // bool m_found_food;
        // bool m_found_people;
        // bool m_found_survivor;
        // bool m_extinguished_fire;

};

#endif
