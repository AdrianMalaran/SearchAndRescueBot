#ifndef Main_h
#define Main_h

#include "Core.h"
#include "MotorPair.h"

#include <Arduino.h>

/* Core Class */
class Main {
    public:
        Main();
        Main(MotorPair motor_pair);
        void Init();
        void Run();
        void ReturnToStart();
        void completeNextTask();

        static void mapAdjacentBlocks(BLOCK_TYPE *global_map[][GLOBAL_COL]);
        static bool isUnexplored(BLOCK_TYPE global_map[][GLOBAL_COL], Coord coord);
        static void mapBlockInFrontTerrain();

        static bool isValid(int row, int col); // TODO: Duplicate function

        void extinguishFire();

    private:
        static BLOCK_TYPE global_map[GLOBAL_ROW][GLOBAL_COL];

        Coord start_coord;

        Queue<TASK> tasks;

        MotorPair m_motor_pair;
};

#endif
