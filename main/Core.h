#ifndef Core_h
#define Core_h

#include "Stack.cpp"
#include "Queue.cpp"

/* GLOBAL CONSTANTS */
const int GLOBAL_ROW = 6;
const int GLOBAL_COL = 6;
const int FLT_MAX = -2;

/* Data Structures */
struct Coord {
  int row;
  int col;

  Coord () : row(0), col(0) {};
  Coord (int row0, int col0) : row(row0), col(col0) {};

  bool operator == (Coord c) {
      return (row == c.row && col == c.col);
  }
};

struct Score {
    double f;
    double h;
    double g;
};

struct CoordScore {
    double f;
    Coord coord;

    CoordScore () : f(0.0), coord(Coord()) {};
    CoordScore (double f0, Coord input) : f(f0), coord(input) {};
};

struct Cell {
  int parent_row;
  int parent_col;

  // Scores: f = g + h
  // g: distance to move from current cell to this cell
  // h: distance of current cell to target cell
  double f;
  double g;
  double h;
};

enum BLOCK_TYPE {
    U = 0, // Unknown
    P = 1, // Particle Board
    W = 2, // Water
    S = 3, // Sand
    G = 5, // Gravel
    X = 6, // Locations of interest
};

enum Instruction {
    MOVE_FORWARD = 0, // 0
    MOVE_BACKWARD = 1, // 1
    ROTATE_RIGHT = 2, // 2
    ROTATE_LEFT = 3 // 3
};

enum Orientation {
    NORTH,
    EAST,
    SOUTH,
    WEST
};

enum TASK {
    EXTINGUISH_FIRE,
    FIND_FOOD,
    FIND_GROUP_OF_PEOPLE,
    DELIVER_FOOD, // Could be eliminated
    FIND_SURVIVOR
};

// Starting Map:
// BLOCK_TYPE global_map[GLOBAL_ROW][GLOBAL_COL] =
// {   //0, 1, 2, 3, 4, 5
//     { U, U, U, U, U, U}, // 0
//     { U, U, U, U, U, U}, // 1
//     { U, U, U, U, U, U}, // 2
//     { U, U, U, U, U, U}, // 3
//     { U, U, U, U, U, U}, // 4
//     { U, U, U, P, U, U}  // 5
// };

#endif
