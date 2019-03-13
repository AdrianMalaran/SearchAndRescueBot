#ifndef Main_h
#define Main_h

#include "Core.h"
#include "MotorPair.h"
#include "Imu.h"
#include "Ultrasonic.h"
#include "Color.h"

#include <Arduino.h>

/* Core Class */
class Main {
    public:
        Main();
        Main(MotorPair motor_pair, Imu imu_sensor, Color color_front, Color color_down,
            Ultrasonic ultrasonic_front, Ultrasonic ultrasonic_right, Ultrasonic ultrasonic_left,
            Ultrasonic ultrasonic_back);
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
        Imu m_imu_sensor;
        Color m_color_front;
        Color m_color_down;
        Ultrasonic m_ultrasonic_front;
        Ultrasonic m_ultrasonic_right;
        Ultrasonic m_ultrasonic_left;
        Ultrasonic m_ultrasonic_back;
};

#endif
