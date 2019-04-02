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

const double TRAVEL_SPEED = 200;
const double TURN_SPEED = 160;

const int FRONT_ARM_LENGTH = 6.5;

const int JITTER_DISTANCE = 2;

// const double distance_per_tick = ((3.14*8)/1905)*(16.0/21.0);
// const double distance_per_tick = ((3.14*8)/1905)*(0.60);
const double distance_per_tick = ((3.14*8)/1514.8)*(16.0/21.0);
// const double num_ticks_per_rev = 1905; //

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
    UNKNOWN = 0, // Unknown
    PARTICLE = 1, // Particle Board
    WATER = 2, // Water
    SAND = 3, // Sand
    GRAVEL = 4, // Gravel
    LANDMARK = 5

};

struct MapLocation {
    BlockType block_type = UNKNOWN;

    bool searched = false;
    bool land_mark_spot = false;
    bool food_searched = false;

    Landmark landmark = NONE;

    MapLocation () {};
    MapLocation(BlockType blocktype) : block_type(blocktype) {}

    MapLocation(BlockType blocktype, bool lm_spot, Landmark lm)
        : block_type(blocktype), land_mark_spot(lm_spot), landmark(lm) {}

    MapLocation(BlockType blocktype, bool lm_spot, Landmark lm, bool is_already_searched)
        : block_type(blocktype), land_mark_spot(lm_spot), landmark(lm), searched(is_already_searched) {}
};

enum Instruction {
    MOVE_FORWARD = 0, // 0
    MOVE_BACKWARD = 1, // 1
    ROTATE_RIGHT = 2, // 2
    ROTATE_LEFT = 3 // 3
};

enum Orientation {
    NORTH = 0,
    EAST = 1,
    SOUTH = 2,
    WEST = 3,
    DONTCARE // Don't care which orientation
};

enum Task {
    DISCOVER_MAP,
    EXTINGUISH_FIRE,
    DELIVER_FOOD,
    FIND_FOOD,
    FIND_GROUP_OF_PEOPLE,
    FIND_SURVIVOR,
    INVESTIGATE_CLOSEST_LANDMARK
};

struct Pose {
    Coord coord;
    Orientation orientation;

    Pose () : coord(Coord(-1, -1)), orientation(DONTCARE) {};
    Pose (Coord c, Orientation o) : coord(c), orientation(o) {};

    bool operator==(Pose p1) {
        return coord.row == p1.coord.row && coord.col == p1.coord.col && orientation == p1.orientation;
    }
};

inline bool isUnblocked(MapLocation grid[][GLOBAL_COL], Coord c) {
    // UPDATE: Removed sand and gravel as traversable
     // return grid[c.row][c.col].block_type == PARTICLE ||
     //        grid[c.row][c.col].block_type == SAND ||
     //        grid[c.row][c.col].block_type == GRAVEL;

     // Only see a block as unblocked if it has been scanned to not have
     return grid[c.row][c.col].block_type == PARTICLE && grid[c.row][c.col].searched == true;
}

inline bool isValid(Coord c) {
  return c.row >= 0 && c.col >= 0 && c.row < GLOBAL_ROW && c.col < GLOBAL_COL;
}

inline void printMap(MapLocation global_map[][GLOBAL_COL]) {
    Serial.println("BlockType Map");
    for (int i = 0; i < GLOBAL_ROW; i++) {
        for(int j = 0; j < GLOBAL_COL; j++) {
            BlockType bt = global_map[i][j].block_type;
            if (bt == PARTICLE) {
                Serial.print("O");
            } else if (bt == WATER) {
                Serial.print("X");
            } else if (bt == GRAVEL) {
                Serial.print("X");
            } else if (bt == SAND) {
                Serial.print("S");
            } else if (bt == LANDMARK) {
                Serial.print("L");
            }
            else {
                Serial.print("?");
            }
            Serial.print(" ");
        }
        Serial.println("");
    }
}

inline void printBlocktype(BlockType blocktype) {
    if (blocktype == WATER)
        Serial.print("WATER");
    if (blocktype == PARTICLE)
        Serial.print("PARTICLE");
    if (blocktype == GRAVEL)
        Serial.print("GRAVEL");
    if (blocktype == SAND)
        Serial.print("SAND");
}

inline void printSearchedMap(MapLocation global_map[][GLOBAL_COL]) {
    Serial.println("Searched Block Locations");
    for (int i = 0; i < GLOBAL_ROW; i++) {
        for(int j = 0; j < GLOBAL_COL; j++) {
            Serial.print(global_map[i][j].searched);
            Serial.print(" ");
        }
        Serial.println("");
    }
}

/* Print Functions */
inline void printCoord(Coord coord) {
    Serial.print("("); Serial.print(coord.row); Serial.print(",");
    Serial.print(coord.col); Serial.print(") ");
}
inline void printOrientation(Orientation ori) {
    if (ori == NORTH)
        Serial.print("NORTH");
    else if (ori == SOUTH)
        Serial.print("SOUTH");
    else if (ori == EAST)
        Serial.print("EAST");
    else if (ori == WEST)
        Serial.print("WEST");
    else
        Serial.print("DONTCARE");
}

inline void printPose(Pose pose) {
    Serial.print("("); Serial.print(pose.coord.row); Serial.print(",");
    Serial.print(pose.coord.col); Serial.print(", ");
    printOrientation(pose.orientation);
    Serial.print(")");
}

inline void printInstruction(Instruction ins) {
    if (ins == MOVE_FORWARD)
        Serial.print("MOVE FORWARD");
    else if (ins == MOVE_BACKWARD)
        Serial.print("MOVE BACK");
    else if (ins == ROTATE_RIGHT)
        Serial.print("TURN RIGHT");
    else if (ins == ROTATE_LEFT)
        Serial.print("TURN LEFT");
}

inline void printStack(Stack<Coord> stack) {
    Serial.print("Path Size: ");
    Serial.println(stack.size());

    Serial.print("(START)");
    while (!stack.empty()) {
        Serial.print("(");
        Serial.print(stack.top().row);
        Serial.print(",");
        Serial.print(stack.top().col);
        Serial.print(") ");
        if(stack.size() > 1) Serial.print(" -> ");
        stack.pop();
    }
    Serial.println("(FINISH)");
}

const MapLocation MP(PARTICLE);
const MapLocation MS(SAND);
const MapLocation MW(WATER);
const MapLocation MG(GRAVEL);
const MapLocation MU(UNKNOWN);

const MapLocation potential_map1[GLOBAL_ROW][GLOBAL_COL] =
{
//    0,  1,  2,  3,  4,  5
    { MP, MP, MP, MW, MP, MP}, // 0
    { MP, MS, MP, MP, MG, MP}, // 1
    { MG, MP, MP, MP, MP, MP}, // 2
    { MP, MP, MS, MP, MP, MW}, // 3
    { MP, MW, MP, MP, MS, MP}, // 4
    { MP, MP, MG, MP, MP, MP}  // 5
};

const MapLocation potential_map2[GLOBAL_ROW][GLOBAL_COL] =
{
//    0, 1, 2, 3, 4, 5
    { MP, MP, MP, MG, MP, MP}, // 0
    { MP, MW, MP, MP, MS, MP}, // 1
    { MG, MP, MS, MP, MP, MP}, // 2
    { MP, MP, MP, MP, MP, MW}, // 3
    { MP, MS, MP, MP, MG, MP}, // 4
    { MP, MP, MW, MP, MP, MP}  // 5
};

const MapLocation potential_map3[GLOBAL_ROW][GLOBAL_COL] =
{
//    0, 1, 2, 3, 4, 5
    { MP, MP, MP, MG, MP, MP}, // 0
    { MP, MS, MP, MP, MW, MP}, // 1
    { MW, MP, MP, MS, MP, MP}, // 2
    { MP, MP, MP, MP, MP, MG}, // 3
    { MP, MG, MP, MP, MS, MP}, // 4
    { MP, MP, MW, MP, MP, MP}  // 5
};

const MapLocation potential_map4[GLOBAL_ROW][GLOBAL_COL] =
{
//    0, 1, 2, 3, 4, 5
    { MP, MP, MP, MW, MP, MP}, // 0
    { MP, MG, MP, MP, MS, MP}, // 1
    { MW, MP, MP, MP, MP, MP}, // 2
    { MP, MP, MP, MS, MP, MG}, // 3
    { MP, MS, MP, MP, MW, MP}, // 4
    { MP, MP, MG, MP, MP, MP}  // 5
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
