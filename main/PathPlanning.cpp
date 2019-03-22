#include <math.h>
#include "PathPlanning.h"

using namespace std;

static bool PathPlanning::isDestination(int row, int col, Coord dest) {
  return (row == dest.row && col == dest.col);
}

static double PathPlanning::calculateHValue(int row, int col, Coord dest) {
  // Calculate Manhattan Distance
  return row-dest.row + col-dest.col;
}

static Stack<Coord> PathPlanning::tracePath(Cell cellDetails[][GLOBAL_COL], Coord dest) {
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

static bool PathPlanning::analyzeAdjacentCell(
    Stack<Coord>& pathToDest,
    Score newScore,
    Coord currCoord,
    Coord newCoord,
    Coord dest,
    bool closedList[GLOBAL_ROW][GLOBAL_COL],
    Queue<CoordScore>& openList,
    Cell cellDetails[GLOBAL_ROW][GLOBAL_COL],
    MapLocation grid[][GLOBAL_COL]) {

    // Only process this cell if this is a valid one
    if (!isValid(newCoord)) {
        return false;
    }

    if (closedList[newCoord.row][newCoord.col]) {
        return false;
    }

    if (!isUnblocked(grid, newCoord)) {
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

static Stack<Coord> PathPlanning::findShortestPath(MapLocation grid[][GLOBAL_COL], Coord start, Coord dest) {
    Stack<Coord> path;

    if (!isValid(start)) {
        Serial.println("Source is invalid");
        return path;
    }

    if (!isUnblocked(grid, start) || !isUnblocked(grid, dest)) {
        Serial.println("Source is blocked!");
        path.push(Coord(-1, -1));
        return path;
    }

    if (isDestination(start.row, start.col, dest)) {
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

        /* Analyze Children */
        if (analyzeAdjacentCell(path, newScore, p.coord, Coord(row - 1, col),
                dest, closedList, openList, cellDetails, grid) || // North
            analyzeAdjacentCell(path, newScore, p.coord, Coord(row + 1, col),
                dest, closedList, openList, cellDetails, grid) || // South
            analyzeAdjacentCell(path, newScore, p.coord, Coord(row, col + 1),
                dest, closedList, openList, cellDetails, grid) || // East
            analyzeAdjacentCell(path, newScore, p.coord, Coord(row, col - 1),
                dest, closedList, openList, cellDetails, grid)) // West
            return path;
    }

    // ERROR CODE
    // TODO: How should we handle this?
    path.push(Coord(-1, -1));
    return path;
}

/* Trajectory Generation */

static void PathPlanning::addReorientation(Queue<Instruction>& instructions,
                                           Orientation curr_orientation,
                                           Orientation new_orientation) {
    if (curr_orientation == new_orientation ||
        new_orientation == DONTCARE ||
        curr_orientation == DONTCARE) {
        return;
    }


    int diff = curr_orientation - new_orientation;
    if (fabs(diff) == 2) {
        // Opposite direction
        // TODO: Consider having another maneuver called: turn around
        // TODO: This is so ugly, redo this shit
        instructions.push(ROTATE_LEFT);
        instructions.push(ROTATE_LEFT);
    } else {
        if (curr_orientation == NORTH) {
            if (new_orientation == EAST)
                instructions.push(ROTATE_RIGHT);
            if (new_orientation == WEST)
                instructions.push(ROTATE_LEFT);
        }
        else if (curr_orientation == EAST) {
            if (new_orientation == SOUTH)
                instructions.push(ROTATE_RIGHT);
            if (new_orientation == NORTH)
                instructions.push(ROTATE_LEFT);
        }
        else if (curr_orientation == SOUTH) {
            if (new_orientation == WEST)
                instructions.push(ROTATE_RIGHT);
            if (new_orientation == EAST)
                instructions.push(ROTATE_LEFT);
        }
        else { // West
            if (new_orientation == NORTH)
                instructions.push(ROTATE_RIGHT);
            if (new_orientation == SOUTH)
                instructions.push(ROTATE_LEFT);
        }
    }
}

//TODO: USE OR OMIT ?
static Orientation PathPlanning::convertAngleToOrientation(double angle) {
    return NORTH;
}

/* Inter-coordinate planner: moving from 1 coord to another */
// TODO: Should account for whether start and finish orientation are the same
static Queue<Instruction> PathPlanning::generateTrajectories(Stack<Coord> path, Orientation start_ori, Orientation finish_ori) {
    // Convert starting pose to a list of instructions to end pose
    Queue<Instruction> instructions;

    // Return empty instructions if non-valid path
    if (path.size() <= 1 && path.top().row == -1 && path.top().col == -1)
        return instructions;

    // Assume a starting orientation
    // This will be done by converting the raw values of the orientation
    Orientation curr_orientation = start_ori; // Convert Angle to orientation

    Coord curr_coord = path.top(); // Starting coordinate
    path.pop();

    while (!path.empty()) {
        Coord next_coord = path.top();
        path.pop();

        // Check direction to go based off of last block
        int diff_row = next_coord.row - curr_coord.row;
        int diff_col = next_coord.col - curr_coord.col;

        Orientation new_orientation;
        // Separate to another function: reorientDirection
        if (diff_row > 0)
            new_orientation = SOUTH;
        else if (diff_row < 0)
            new_orientation = NORTH;
        else if (diff_col > 0)
            new_orientation = EAST;
        else if (diff_col < 0)
            new_orientation = WEST;

        addReorientation(instructions, curr_orientation, new_orientation);
        instructions.push(MOVE_FORWARD);

        curr_coord = next_coord;
        curr_orientation = new_orientation;
    }

    addReorientation(instructions, curr_orientation, finish_ori);
}
