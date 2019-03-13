#ifndef Core_h
#define Core_h

#include "Stack.cpp"
#include "Queue.cpp"

/* GLOBAL CONSTANTS */
const int GLOBAL_ROW = 6;
const int GLOBAL_COL = 6;
const int FLT_MAX = -2;

const int INT_MAX = 999;
const int INT_MIN = -999;

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

enum Landmark {
    NONE,
    FIRE,
    FOOD,
    PEOPLE,
    SURVIVOR
};

enum BlockType {
    U = 0, // Unknown
    P = 1, // Particle Board
    W = 2, // Water
    S = 3, // Sand
    G = 4, // Gravel
    X = 5, // Locations of interest
};

struct MapLocation {
    BlockType block_type = U;

    bool searched = false;
    bool land_mark_spot = false;

    Landmark landmark = NONE;

    MapLocation () {};
    MapLocation(BlockType blocktype) : block_type(blocktype) {};
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
    WEST,
    DONTCARE // Don't care which orientation
};

enum Task {
    EXTINGUISH_FIRE,
    FIND_FOOD,
    FIND_GROUP_OF_PEOPLE,
    DELIVER_FOOD, // Could be eliminated
    FIND_SURVIVOR
};

struct Pose {
    Coord coord;
    Orientation orientation;

    Pose () {};
    Pose (Coord c, Orientation o) : coord(c), orientation(o) {};
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