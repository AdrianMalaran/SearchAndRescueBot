#ifndef PathPlanning_h
#define PathPlanning_h

#include "Arduino.h"

#include "Core.h"

class PathPlanning {
	public:
		PathPlanning();
        static bool isValid(Coord c);
        static bool isUnblocked(MapLocation grid[][GLOBAL_COL], int row, int col);
        static bool isDestination(int row, int col, Coord dest);
        static double calculateHValue(int row, int col, Coord dest);
        static Stack<Coord> tracePath(Cell cellDetails[][GLOBAL_COL], Coord dest);
        static bool analyzeAdjacentCell(
            Stack<Coord>& pathToDest,
            Score newScore,
            Coord currCoord,
            Coord newCoord,
            Coord dest,
            bool closedList[GLOBAL_ROW][GLOBAL_COL],
            Queue<CoordScore>& openList,
            Cell cellDetails[GLOBAL_ROW][GLOBAL_COL],
            MapLocation grid[][GLOBAL_COL]);
        static Stack<Coord> findShortestPath(MapLocation grid[][GLOBAL_COL], Coord start, Coord dest);

        static void addReorientation(Queue<Instruction>& instructions,
                              Orientation curr_orientation,
                              Orientation new_orientation);
        static Orientation convertAngleToOrientation(double angle);

        static Queue<Instruction> generateTrajectories(Stack<Coord> path,
            Orientation start_ori,
            Orientation finish_ori);
		static void executeInstructions(Queue<Instruction> instructions);
};

#endif
