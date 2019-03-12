#ifndef Main_h
#define Main_h

#include "Core.h"

#include <Arduino.h>

/* Core Class */
class Main {
    public:
        void InitRobot();
        void Run();
        void ReturnToStart();

    private:
        BLOCK_TYPE global_map[GLOBAL_ROW][GLOBAL_COL];
        Coord start_coord;
};

#endif
