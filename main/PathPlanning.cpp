#include <math.h>

#include "Core.cpp"
#include "Stack.cpp"

using namespace std;

/* Constants */
// TODO: Initialize GLOBAL_ROW & GLOBAL_COL
const int ROW = 6;
const int COL = 6;

const int FLT_MAX = -2;

//TODO: Add a row that determines if its a water block
bool isValid(int row, int col) {
  return row >= 0 && col >= 0 && row < ROW && col < COL;
}

bool isUnblocked(int grid[][COL], int row, int col) {
  // Grid cell with value (1) means its unblocked
  return grid[row][col] == 1;
}

bool isDestination(int row, int col, Coord dest) {
  return (row == dest.row && col == dest.col);
}

double calculateHValue(int row, int col, Coord dest) {
  // Calculate h value using euclidean distance
  // Alternatively, we could pre-calculate all grid distances between
  // each cell before use of A*

  // Use Manhattan Distance
  return row-dest.row + col-dest.col;
}

void tracePath(Cell cellDetails[][COL], Coord dest) {
    // printf("Calculated Path:\n");

    int row = dest.row;
    int col = dest.col;

    Stack<Coord> path;

    while (!(cellDetails[row][col].parent_row == row &&
             cellDetails[row][col].parent_col == col)) {
        path.push(Coord(row, col));
        int temp_row = cellDetails[row][col].parent_row;
        int temp_col = cellDetails[row][col].parent_col;

        row = temp_row;
        col = temp_col;
    }

    path.push(Coord(row, col ));
    while (!path.empty()) {
        Coord block = path.top();
        path.pop();
    }
}

void analyzeAdjacentCell(bool& foundDest, int row, int col,
    bool& closedList[ROW][COL],
    Queue<CoordScore>& openList) {

    // Only process this cell if this is a valid one
    if (!isValid(row, col))
        return;

    // If the destination cell is the same as the current successor
    if (isDestination(row, col, dest) == true) {
        // Set the Parent of the destination cell
        cellDetails[row][col].parent_row = row + 1;
        cellDetails[row][col].parent_col = col;
        tracePath (cellDetails, dest);
        foundDest = true;
        return;
    }
    // Else do the following
    else if (closedList[row][col] == false &&
             isUnBlocked(grid, row, col) == true) {
        gNew = cellDetails[i][col].g + 1.0;
        hNew = calculateHValue (row, col, dest);
        fNew = gNew + hNew;

        // If it isn’t on the open list, add it to
        // the open list. Make the current square
        // the parent of this square. Record the
        // f, g, and h costs of the square cell
        //                OR
        // If it is on the open list already, check
        // to see if this path to that square is better,
        // using 'f' cost as the measure.
        if (cellDetails[row][col].f == FLT_MAX ||
                cellDetails[row][col].f > fNew) {
            openList.push(CoordScore(fNew, Coord(row, col)));

            // Update the details of this cell
            cellDetails[row][col].f = fNew;
            cellDetails[row][col].g = gNew;
            cellDetails[row][col].h = hNew;
            cellDetails[row][col].parent_row = row + 1;
            cellDetails[row][col].parent_col = col;
        }
    }
}

void AStarSearch(int grid[][COL], Coord start, Coord dest) {
    if (!isValid(start.row, start.col)) {
        //printf("Source is invalid\n");
        return;
    }

    if (!isUnblocked(grid, start.row, start.col) ||
      !isUnblocked(grid, dest.row, dest.col)) {
        //printf("Source is blocked!\n");
        return;
    }

    if (isDestination(start.row, start.col, dest)) {
        //printf("We are already at the destination!");
        return;
    }

    bool closedList[ROW][COL] = {-1};
    Cell cellDetails[ROW][COL];

    // Copy/Paste starts here
    int i, j;

    for (i=0; i<ROW; i++)
    {
        for (j=0; j<COL; j++)
        {
            cellDetails[i][j].f = FLT_MAX;
            cellDetails[i][j].g = FLT_MAX;
            cellDetails[i][j].h = FLT_MAX;
            cellDetails[i][j].parent_row = -1;
            cellDetails[i][j].parent_col = -1;
        }
    }

    // Initialising the parameters of the starting node
    i = start.row, j = start.col;
    cellDetails[i][j].f = 0.0;
    cellDetails[i][j].g = 0.0;
    cellDetails[i][j].h = 0.0;
    cellDetails[i][j].parent_row = i;
    cellDetails[i][j].parent_col = j;

    Queue<CoordScore> openList;
    openList.push(CoordScore(0.0, Coord(row, col)));

    bool foundDest = false;

    while (!openList.empty()) {
        CoordScore<int> p = openList.front();
        openList.pop();

        int i = p.coord.row; // Redo
        int j = p.coord.col;

        closedList[i][j] = true;

        double gNew, hNew, fNew;

        analyzeAdjacentCell(foundDest, row - 1, col); // North
        analyzeAdjacentCell(foundDest, row + 1, col); // South
        analyzeAdjacentCell(foundDest, row, col + 1); // East
        analyzeAdjacentCell(foundDest, row, col - 1); // West
    }
}
