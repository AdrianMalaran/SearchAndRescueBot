const int GLOBAL_ROW = 6;
const int GLOBAL_COL = 6;

/* Data Structures */
struct Coord {
  int row;
  int col;

  Coord () : row(0), col(0) {};
  Coord (int row0, int col0) : row(row0), col(col0) {};
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

/* Core Class */
class Core {
public:
    // Member Variables
    int globalMap[GLOBAL_ROW][GLOBAL_COL];

    // Member Functions
    static void InitRobot() {
        // (1) Initialize map;
        // (2) Test Path Planning
        // (3) Calibrate sensors
    }
}
