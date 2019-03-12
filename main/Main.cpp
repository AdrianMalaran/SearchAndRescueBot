#include "Main.h"

void Main::InitRobot() {
    // (1) Initialize map;
    // (2) Test Path Planning
    // (3) Calibrate sensors

    // Set start coord
    start_coord = Coord(4, 5);

    // Initialize Start Map
    for (int i = 0; i < GLOBAL_ROW; i ++) {
        for (int j = 0; j < GLOBAL_COL; j++) {
            global_map[i][j] = U;
        }
    }

    global_map[start_coord.row][start_coord.col] = P;
}

//TODO: implement this run function
void Main::Run() {
    // This function will be run in the loop() function of the arduino

}

void Main::ReturnToStart() {
    // Get current location
    // travelToBack();
    // stopToProgram();
}
