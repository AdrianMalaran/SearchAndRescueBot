#include "PathPlanning.cpp"

/* Print Functions */
void printCoord(Coord coord) {
    Serial.print("("); Serial.print(coord.row); Serial.print(",");
    Serial.print(coord.col); Serial.print(") ");
}

void printStack(Stack<Coord> stack) {
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

/* AStarSearch */
//TODO: Add a check that determines if its a water block
inline bool isValid(int row, int col) {
  return row >= 0 && col >= 0 && row < GLOBAL_ROW && col < GLOBAL_COL;
}

inline bool isUnblocked(int grid[][GLOBAL_COL], int row, int col) {
  return grid[row][col] == 0;
}

inline bool isDestination(int row, int col, Coord dest) {
  return (row == dest.row && col == dest.col);
}

inline double calculateHValue(int row, int col, Coord dest) {
  // Calculate Manhattan Distance
  return row-dest.row + col-dest.col;
}

inline Stack<Coord> tracePath(Cell cellDetails[][GLOBAL_COL], Coord dest) {
    int row = dest.row;
    int col = dest.col;

    Stack<Coord> tempPath;

    while (!(cellDetails[row][col].parent_row == -2 &&
             cellDetails[row][col].parent_col == -2)) {

        tempPath.push(Coord(row, col));

        int temp_row = cellDetails[row][col].parent_row;
        int temp_col = cellDetails[row][col].parent_col;

        row = temp_row;
        col = temp_col;
    }

    tempPath.push(Coord(row, col));
    return tempPath;
}

inline bool analyzeAdjacentCell(
    Stack<Coord>& pathToDest,
    Score newScore,
    Coord currCoord,
    Coord newCoord,
    Coord dest,
    bool closedList[GLOBAL_ROW][GLOBAL_COL],
    Queue<CoordScore>& openList,
    Cell cellDetails[GLOBAL_ROW][GLOBAL_COL],
    int grid[][GLOBAL_COL]) {

    // Only process this cell if this is a valid one
    if (!isValid(newCoord.row, newCoord.col)) {
        return false;
    }

    if (closedList[newCoord.row][newCoord.col]) {
        return false;
    }

    if (!isUnblocked(grid, newCoord.row, newCoord.col)) {
        return false;
    }

    // If the destination cell is the same as the current successor
    if (isDestination(newCoord.row, newCoord.col, dest)) {
        // Set the Parent of the destination cell
        cellDetails[newCoord.row][newCoord.col].parent_row = currCoord.row;
        cellDetails[newCoord.row][newCoord.col].parent_col = currCoord.col;
        pathToDest = tracePath(cellDetails, dest);
        return true;
    }

    newScore.g = cellDetails[newCoord.row][newCoord.col].g + 1.0;
    newScore.h = calculateHValue (newCoord.row, newCoord.col, dest);
    newScore.f = newScore.g + newScore.h;

    // If it isn’t on the open list, add it to
    // the open list. Make the current square
    // the parent of this square. Record the
    // f, g, and h costs of the square cell
    //                OR
    // If it is on the open list already, check
    // to see if this path to that square is better,
    // using 'f' cost as the measure.
    if (cellDetails[newCoord.row][newCoord.col].f == FLT_MAX ||
            cellDetails[newCoord.row][newCoord.col].f > newScore.f) {
        openList.push(CoordScore(newScore.f, Coord(newCoord.row, newCoord.col)));

        // Update the details of this cell
        cellDetails[newCoord.row][newCoord.col].f = newScore.f;
        cellDetails[newCoord.row][newCoord.col].g = newScore.g;
        cellDetails[newCoord.row][newCoord.col].h = newScore.h;
        cellDetails[newCoord.row][newCoord.col].parent_row = currCoord.row;
        cellDetails[newCoord.row][newCoord.col].parent_col = currCoord.col;
    }
    return false;
}

inline Stack<Coord> AStarSearch(int grid[][GLOBAL_COL], Coord start, Coord dest) {
    Stack<Coord> path;

    if (!isValid(start.row, start.col)) {
        Serial.println("Source is invalid\n");
        return path;
    }

    if (!isUnblocked(grid, start.row, start.col) ||
      !isUnblocked(grid, dest.row, dest.col)) {
        Serial.println("Source is blocked!\n");
        return path;
    }

    if (isDestination(start.row, start.col, dest)) {
        Serial.println("Already at Destination");
        path.push(start);
        return path;
    }

    bool closedList[GLOBAL_ROW][GLOBAL_COL] = {false};
    Cell cellDetails[GLOBAL_ROW][GLOBAL_COL];

    for (int i=0; i<GLOBAL_ROW; i++)
    {
        for (int j=0; j<GLOBAL_COL; j++)
        {
            cellDetails[i][j].f = FLT_MAX;
            cellDetails[i][j].g = FLT_MAX;
            cellDetails[i][j].h = FLT_MAX;
            cellDetails[i][j].parent_row = -1;
            cellDetails[i][j].parent_col = -1;
        }
    }

    // Initialising the parameters of the starting node
    cellDetails[start.row][start.col].f = 0.0;
    cellDetails[start.row][start.col].g = 0.0;
    cellDetails[start.row][start.col].h = 0.0;
    cellDetails[start.row][start.col].parent_row = -2;
    cellDetails[start.row][start.col].parent_col = -2;

    Queue<CoordScore> openList;
    openList.push(CoordScore(0.0, start));

    bool foundDest = false;

    while (!openList.empty()) {
        CoordScore p = openList.front();
        openList.pop();

        int row = p.coord.row; // Redo
        int col = p.coord.col;

        closedList[row][col] = true;
        Score newScore;

        // Serial.print("Current Point: ");
        // printCoord(p.coord);

        /* Analyze Children */
        if (analyzeAdjacentCell(path, newScore, p.coord, Coord(row - 1, col), dest, closedList, openList, cellDetails, grid) || // North
            analyzeAdjacentCell(path, newScore, p.coord, Coord(row + 1, col), dest, closedList, openList, cellDetails, grid) || // South
            analyzeAdjacentCell(path, newScore, p.coord, Coord(row, col + 1), dest, closedList, openList, cellDetails, grid) || // East
            analyzeAdjacentCell(path, newScore, p.coord, Coord(row, col - 1), dest, closedList, openList, cellDetails, grid)) // West
            return path;
    }

    // ERROR CODE
    path.push(Coord(-1, -1));
    return path;
}

// End AStarSearch

//TODO: Seperate Files
class Tests {
public:
    static Stack<Coord> TestPathPlanning(int grid[][GLOBAL_COL], Coord start, Coord finish) {
        Serial.print("Start: "); printCoord(start);
        Serial.print("Finish: "); printCoord(finish);

        printStack(AStarSearch(grid, start, finish));
    }
    static Stack<Coord> TestPathPlanning() {
        int testGrid[GLOBAL_ROW][GLOBAL_COL] =
        {
        //    0, 1, 2, 3, 4, 5
            { 0, 0, 0, 0, 0, 0}, // 0
            { 0, 0, 0, 0, 0, 0}, // 1
            { 0, 0, 0, 0, 0, 0}, // 2
            { 0, 0, 0, 0, 0, 0}, // 3
            { 0, 0, 0, 0, 0, 0}, // 4
            { 0, 0, 0, 0, 0, 0}  // 5
        };

        // TestPathPlanning(grid, start, finish)
        TestPathPlanning(testGrid, Coord(4,3), Coord(2,0)); // NORTH WEST
        TestPathPlanning(testGrid, Coord(0,0), Coord(5,5)); // SOUTH EAST
        TestPathPlanning(testGrid, Coord(0,0), Coord(0,5)); // EAST
        TestPathPlanning(testGrid, Coord(0,5), Coord(0,0)); // WEST
        TestPathPlanning(testGrid, Coord(5,5), Coord(0,0)); // WEST

        int testGrid2[GLOBAL_ROW][GLOBAL_COL] =
        {
        //    0, 1, 2, 3, 4, 5
            { 0, 1, 0, 0, 0, 0}, // 0
            { 0, 1, 1, 1, 1, 0}, // 1
            { 0, 0, 0, 0, 0, 0}, // 2
            { 0, 0, 0, 0, 0, 0}, // 3
            { 0, 0, 0, 0, 0, 0}, // 4
            { 0, 0, 0, 0, 0, 0}  // 5
        };
        // Immediate Path is Blocked
        TestPathPlanning(testGrid2, Coord(0,2), Coord(0,0));

        int testGrid3[GLOBAL_ROW][GLOBAL_COL] =
        {
        //    0, 1, 2, 3, 4, 5
            { 0, 1, 0, 0, 0, 1}, // 0
            { 0, 1, 1, 1, 1, 0}, // 1
            { 0, 0, 0, 0, 0, 0}, // 2
            { 0, 0, 0, 0, 0, 0}, // 3
            { 0, 0, 0, 0, 0, 0}, // 4
            { 0, 0, 0, 0, 0, 0}  // 5
        };

        // No Path
        TestPathPlanning(testGrid3, Coord(0,2), Coord(5,5));

        Stack<Coord> path;
        return path;
    }
};

// End Tests
int BAUD_RATE = 9600;

void setup() {
  Serial.begin(BAUD_RATE);
  Serial.println("Initializing ...");

  Stack<Coord> path = Tests::TestPathPlanning();
}

void loop() {
  // put your main code here, to run repeatedly:

}
