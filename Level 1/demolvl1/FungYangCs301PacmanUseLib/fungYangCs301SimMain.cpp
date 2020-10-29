//======================================================================
//Author: 
//Mr. Fung Yang
//Senior Technician Engineer Research and Design,
//Robotics and Control system signal processing Labs,
//Department of Electrical, Computer and Software Engineering,
//The University of Auckland.
//
//Written for teaching design course Compsys301 in ECSE department of UOA.
//
//This example program uses the pacman robot simulation library written by Mr. Fung Yang.
//
//Date 2012~2020
//=======================================================================

#include "mainFungGLAppEngin.h" //a must
#include "data.h" //a must
#include "highPerformanceTimer.h"//just to include if timer function is required by user.
#include <vector>
#include <iostream>
#include <stack>
#include <set>
#include <time.h>

#define ROW 15
#define COL 19

using namespace std;



typedef pair<double, pair<int, int>> pPair;

vector<vector<int>> food_left(sizeof(food_list) / sizeof(*food_list), vector<int>(2));

int pathLength;
bool finalPathFlag = false;

int moveCount = -1;
int pathCount = 0;


bool destinationFlag = false;

int updateCount = 0;

typedef pair<int, int> Pair;

vector<Pair> destinations;
vector<Pair> beenToDestinations;
vector<vector<Pair>> fullPath;
vector<Pair> Path;
vector<char> directions;
vector<Pair> intersections;

vector<Pair> Unvisited; // need to be defined as a global variable
vector<Pair> Visited; // need to be defined as a global variable

bool intersectionFlag = false;


//{=================================================
//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
//these global variables must be defined here with no modification.
float virtualCarLinearSpeed;//can get ands set
float virtualCarAngularSpeed;//can get and set
float currentCarAngle;//can get and set
float currentCarPosCoord_X, currentCarPosCoord_Y;//can get and set
int preCarPosCoord_X = coordToCellX(currentCarPosCoord_X);
int preCarPosCoord_Y = coordToCellY(currentCarPosCoord_Y);

int sensorPopulationAlgorithmID;//can set
float sensorSeparation;//can set
float num_sensors;//can set

vector<int> virtualCarSensorStates; //can get

void LV1(Pair startLoca);

vector<ghostInfoPack> ghostInfoPackList;// can get
//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
//}=================================================

highPerformanceTimer myTimer;


struct cell
{
    // Row and Column index of its parent 
    // Note that 0 <= i <= ROW-1 & 0 <= j <= COL-1 
    int parent_i, parent_j;
    // f = g + h 
    double f, g, h;
};

struct point {
    int x, y;

};

//The Only TWO unctions Students need to modify to add their own sensor guided
//path following control and Map path search calculations.
//{=================================================
float virtualCarLinearSpeed_seed;
float virtualCarAngularSpeed_seed;


//Algorithm Functions:

bool isValid(int row, int col)
{
    // Returns true if row number and column number 
    // is in range 
    return (row >= 0) && (row < ROW) &&
        (col >= 0) && (col < COL);
}

bool isUnBlocked(int grid[][COL], int row, int col)
{
    // Returns true if the cell is not blocked else false 
    if (grid[row][col] == 0)
        return (true);
    else
        return (false);
}

bool isDestination(int row, int col, Pair dest)
{
    if (row == dest.first && col == dest.second)
        return (true);
    else
        return (false);
}

double calculateHValue(int row, int col, Pair dest)
{
    // Return using the distance formula 
    return double(abs(row - dest.first) + abs(col - dest.second));

}

void tracePath(cell cellDetails[][COL], Pair dest)
{
    printf("\nThe Path is ");
    int row = dest.first;
    int col = dest.second;

    Path = {};

    while (!(cellDetails[row][col].parent_i == row
        && cellDetails[row][col].parent_j == col))
    {
        Path.push_back(make_pair(row, col));
        int temp_row = cellDetails[row][col].parent_i;
        int temp_col = cellDetails[row][col].parent_j;
        row = temp_row;
        col = temp_col;
    }

    Path.push_back(make_pair(row, col));
    pathLength = Path.size();

    reverse(Path.begin(), Path.end());

    for (int i = 0; i < pathLength; i++)
    {
        pair<int, int> p = Path[i];
    }

    return;
}

void tracePathFull(cell cellDetails[][COL], Pair dest) {
    int row = dest.first;
    int col = dest.second;

    vector<Pair> path;

    while (!(cellDetails[row][col].parent_i == row
        && cellDetails[row][col].parent_j == col))
    {
        cout << "position added to path: (" << row << ", " << col << ")\n";
        path.push_back(make_pair(row, col));
        int temp_row = cellDetails[row][col].parent_i;
        int temp_col = cellDetails[row][col].parent_j;
        row = temp_row;
        col = temp_col;
    }
    cout << "position added to path: (" << row << ", " << col << ")\n";
    path.push_back(make_pair(row, col));
    reverse(path.begin(), path.end());
    fullPath.push_back(path);
}

void aStarSearch(int grid[][COL], Pair src, Pair dest) {
    bool closedList[ROW][COL];

    memset(closedList, false, sizeof(closedList));

    cell cellDetails[ROW][COL];

    int pathLength = 0;

    int i, j;

    for (i = 0; i < ROW; i++)
    {
        for (j = 0; j < COL; j++)
        {
            cellDetails[i][j].f = FLT_MAX;
            cellDetails[i][j].g = FLT_MAX;
            cellDetails[i][j].h = FLT_MAX;
            cellDetails[i][j].parent_i = -1;
            cellDetails[i][j].parent_j = -1;
        }
    }

    i = src.first, j = src.second;
    cellDetails[i][j].f = 0.0;
    cellDetails[i][j].g = 0.0;
    cellDetails[i][j].h = 0.0;
    cellDetails[i][j].parent_i = i;
    cellDetails[i][j].parent_j = j;

    set<pPair> openList;

    openList.insert(make_pair(0.0, make_pair(i, j)));

    bool foundDest = false;

    while (!openList.empty()) {
        pPair p = *openList.begin();

        openList.erase(openList.begin());

        i = p.second.first;
        j = p.second.second;

        closedList[i][j] = true;

        double gNew, hNew, fNew;

        //----------- 1st Successor (North) ------------ 
        // Only process this cell if this is a valid one 
        if (isValid(i - 1, j) == true)
        {
            // If the destination cell is the same as the 
            // current successor 
            if (isDestination(i - 1, j, dest) == true)
            {
                // Set the Parent of the destination cell 
                cellDetails[i - 1][j].parent_i = i;
                cellDetails[i - 1][j].parent_j = j;
                printf("\nThe destination cell is found\n");
                if (finalPathFlag) {
                    cout << "finalTracePathing";
                    tracePathFull(cellDetails, dest);
                }
                else {
                    tracePath(cellDetails, dest);
                }
                foundDest = true;
                return;
            }
            // If the successor is already on the closed 
            // list or if it is blocked, then ignore it. 
            // Else do the following 
            else if (closedList[i - 1][j] == false &&
                isUnBlocked(grid, i - 1, j) == true)
            {
                gNew = cellDetails[i][j].g + 1.0;
                hNew = calculateHValue(i - 1, j, dest);
                fNew = gNew + hNew;

                // If it isn’t on the open list, add it to 
                // the open list. Make the current square 
                // the parent of this square. Record the 
                // f, g, and h costs of the square cell 
                //                OR 
                // If it is on the open list already, check 
                // to see if this path to that square is better, 
                // using 'f' cost as the measure. 
                if (cellDetails[i - 1][j].f == FLT_MAX ||
                    cellDetails[i - 1][j].f > fNew)
                {
                    openList.insert(make_pair(fNew,
                        make_pair(i - 1, j)));

                    // Update the details of this cell 
                    cellDetails[i - 1][j].f = fNew;
                    cellDetails[i - 1][j].g = gNew;
                    cellDetails[i - 1][j].h = hNew;
                    cellDetails[i - 1][j].parent_i = i;
                    cellDetails[i - 1][j].parent_j = j;
                }
            }
        }

        //----------- 2nd Successor (South) ------------ 

        // Only process this cell if this is a valid one 
        if (isValid(i + 1, j) == true)
        {
            // If the destination cell is the same as the 
            // current successor 
            if (isDestination(i + 1, j, dest) == true)
            {
                // Set the Parent of the destination cell 
                cellDetails[i + 1][j].parent_i = i;
                cellDetails[i + 1][j].parent_j = j;
                printf("\nThe destination cell is found\n");
                if (finalPathFlag) {
                    cout << "finalTracePathing";
                    tracePathFull(cellDetails, dest);
                }
                else {
                    tracePath(cellDetails, dest);
                }
                foundDest = true;
                return;
            }
            // If the successor is already on the closed 
            // list or if it is blocked, then ignore it. 
            // Else do the following 
            else if (closedList[i + 1][j] == false &&
                isUnBlocked(grid, i + 1, j) == true)
            {
                gNew = cellDetails[i][j].g + 1.0;
                hNew = calculateHValue(i + 1, j, dest);
                fNew = gNew + hNew;

                // If it isn’t on the open list, add it to 
                // the open list. Make the current square 
                // the parent of this square. Record the 
                // f, g, and h costs of the square cell 
                //                OR 
                // If it is on the open list already, check 
                // to see if this path to that square is better, 
                // using 'f' cost as the measure. 
                if (cellDetails[i + 1][j].f == FLT_MAX ||
                    cellDetails[i + 1][j].f > fNew)
                {
                    openList.insert(make_pair(fNew, make_pair(i + 1, j)));
                    // Update the details of this cell 
                    cellDetails[i + 1][j].f = fNew;
                    cellDetails[i + 1][j].g = gNew;
                    cellDetails[i + 1][j].h = hNew;
                    cellDetails[i + 1][j].parent_i = i;
                    cellDetails[i + 1][j].parent_j = j;
                }
            }
        }

        //----------- 3rd Successor (East) ------------ 

        // Only process this cell if this is a valid one 
        if (isValid(i, j + 1) == true)
        {
            // If the destination cell is the same as the 
            // current successor 
            if (isDestination(i, j + 1, dest) == true)
            {
                // Set the Parent of the destination cell 
                cellDetails[i][j + 1].parent_i = i;
                cellDetails[i][j + 1].parent_j = j;
                printf("\nThe destination cell is found\n");
                if (finalPathFlag) {
                    cout << "finalTracePathing";
                    tracePathFull(cellDetails, dest);
                }
                else {
                    tracePath(cellDetails, dest);
                }
                foundDest = true;
                return;
            }

            // If the successor is already on the closed 
            // list or if it is blocked, then ignore it. 
            // Else do the following 
            else if (closedList[i][j + 1] == false &&
                isUnBlocked(grid, i, j + 1) == true)
            {
                gNew = cellDetails[i][j].g + 1.0;
                hNew = calculateHValue(i, j + 1, dest);
                fNew = gNew + hNew;

                // If it isn’t on the open list, add it to 
                // the open list. Make the current square 
                // the parent of this square. Record the 
                // f, g, and h costs of the square cell 
                //                OR 
                // If it is on the open list already, check 
                // to see if this path to that square is better, 
                // using 'f' cost as the measure. 
                if (cellDetails[i][j + 1].f == FLT_MAX ||
                    cellDetails[i][j + 1].f > fNew)
                {
                    openList.insert(make_pair(fNew,
                        make_pair(i, j + 1)));

                    // Update the details of this cell 
                    cellDetails[i][j + 1].f = fNew;
                    cellDetails[i][j + 1].g = gNew;
                    cellDetails[i][j + 1].h = hNew;
                    cellDetails[i][j + 1].parent_i = i;
                    cellDetails[i][j + 1].parent_j = j;
                }
            }
        }

        //----------- 4th Successor (West) ------------ 

        // Only process this cell if this is a valid one 
        if (isValid(i, j - 1) == true)
        {
            // If the destination cell is the same as the 
            // current successor 
            if (isDestination(i, j - 1, dest) == true)
            {
                // Set the Parent of the destination cell 
                cellDetails[i][j - 1].parent_i = i;
                cellDetails[i][j - 1].parent_j = j;
                printf("\nThe destination cell is found\n");
                if (finalPathFlag) {
                    cout << "finalTracePathing";
                    tracePathFull(cellDetails, dest);
                }
                else {
                    tracePath(cellDetails, dest);
                }
                foundDest = true;
                return;
            }

            // If the successor is already on the closed 
            // list or if it is blocked, then ignore it. 
            // Else do the following 
            else if (closedList[i][j - 1] == false &&
                isUnBlocked(grid, i, j - 1) == true)
            {
                gNew = cellDetails[i][j].g + 1.0;
                hNew = calculateHValue(i, j - 1, dest);
                fNew = gNew + hNew;

                // If it isn’t on the open list, add it to 
                // the open list. Make the current square 
                // the parent of this square. Record the 
                // f, g, and h costs of the square cell 
                //                OR 
                // If it is on the open list already, check 
                // to see if this path to that square is better, 
                // using 'f' cost as the measure. 
                if (cellDetails[i][j - 1].f == FLT_MAX ||
                    cellDetails[i][j - 1].f > fNew)
                {
                    openList.insert(make_pair(fNew,
                        make_pair(i, j - 1)));

                    // Update the details of this cell 
                    cellDetails[i][j - 1].f = fNew;
                    cellDetails[i][j - 1].g = gNew;
                    cellDetails[i][j - 1].h = hNew;
                    cellDetails[i][j - 1].parent_i = i;
                    cellDetails[i][j - 1].parent_j = j;
                }
            }
        }
    }

    if (foundDest == false)
        printf("Failed to find the Destination Cell\n");

    return;
}



void findFullPath(Pair start) {
    finalPathFlag = true;
    for (int i = 0; i < destinations.size(); i++) {
        if (i == 0) {
            aStarSearch(map, start, destinations[i]);
            cout << "path from: " << start.first << " , " << start.second << " to: " << destinations[i].first << " , " << destinations[i].second << "\n";
            aStarSearch(map, destinations[i], destinations[i + 1]);
            cout << "path from: " << destinations[i].first << " , " << destinations[i].second << " to: " << destinations[i+1].first << " , " << destinations[i+1].second << "\n";
        }
        else if (i == destinations.size() - 1) {
            cout << "Done!\n";
        }
        else {
            aStarSearch(map, destinations[i], destinations[i + 1]);
            cout << "path from: " << destinations[i].first << " , " << destinations[i].second << " to: " << destinations[i + 1].first << " , " << destinations[i + 1].second << "\n";
        }
    }
    

}

void printDestinations() {
    for (int i = 0; i < destinations.size(); i++) {
        cout << "destionations: " << destinations[i].first << ", " << destinations[i].second << "\n";
    }
}

void printFullPath() {
    cout << "FullPath: \n";
    for (int i = 0; i < fullPath.size(); i++) {
        for (int j = 0; j < fullPath[i].size(); j++) {
            cout << "(" << fullPath[i][j].first << ", " << fullPath[i][j].second << ") ";
        }
        cout << "\n";
    }
}

void algorithmIntersectionDetection() {
    int pathCount = 0;
    for (int i = 0; i < ROW; i++) {
        for (int j = 0; j < COL; j++) {
            if (!map[i][j]) {

                if (!(map[i + 1][j])) {

                    pathCount++;
                }
                if (!(map[i - 1][j])) {

                    pathCount++;
                }
                if (!(map[i][j + 1])) {

                    pathCount++;
                }
                if (!(map[i][j - 1])) {

                    pathCount++;
                }
                if (pathCount >= 3) {
                    pair<int, int> p = make_pair(i, j);

                    intersections.push_back(p);

                }
                pathCount = 0;
            }
        }
    }
}


void createDirectionVector() {
    pair<int, int> currentCoords;
    pair<int, int> nextCoords;
    pair<int, int> prevCoords;
    pair<int, int> uturnCheck;

    for (int i = 0; i < fullPath.size(); i++) {
        for (int j = 0; j < fullPath[i].size(); j++) {

            currentCoords = fullPath[i][j];
            nextCoords = fullPath[i][j + 1];
           
            if (j == 0) {

                prevCoords = currentCoords;
                nextCoords = fullPath[i][j + 1];
            }
            else if (j == fullPath[i].size() - 1) {
                nextCoords = currentCoords;
                prevCoords = fullPath[i][j - 1];
                uturnCheck = prevCoords;
            }
            else {
                prevCoords = fullPath[i][j - 1];
                nextCoords = fullPath[i][j + 1];
            }

            for (int p = 0; p < intersections.size(); p++) {
                if (fullPath[i][j] == intersections[p]) {
                    if (currentCoords.first > nextCoords.first && currentCoords.second == nextCoords.second) {
                        directions.push_back('u');
                    }
                    else if (currentCoords.first < nextCoords.first && currentCoords.second == nextCoords.second) {
                        directions.push_back('d');
                    }
                    else if (currentCoords.first == nextCoords.first && currentCoords.second > nextCoords.second) {
                        directions.push_back('l');
                    }
                    else if (currentCoords.first == nextCoords.first && currentCoords.second < nextCoords.second) {
                        directions.push_back('r');

                    }

                }
            }
            if (uturnCheck.first == nextCoords.first && uturnCheck.second == nextCoords.second) {
                directions.push_back('t');
            }
        }
    }
}


void setVirtualCarSpeed(float linearSpeed, float angularSpeed)
{
    virtualCarLinearSpeed = linearSpeed;
    virtualCarAngularSpeed = angularSpeed;
}


int virtualCarInit()
{
    //sensorPopulationAlgorithmID = PLACE_SENSORS_AUTO_SEP;
    sensorPopulationAlgorithmID = PLACE_SENSORS_SEP_USER_DEFINED;
    num_sensors = 7;
    sensorSeparation = 0.08;

    virtualCarLinearSpeed_seed = 1.3;
    virtualCarAngularSpeed_seed = 90;
    currentCarPosCoord_X = cellToCoordX(1);
    currentCarPosCoord_Y = cellToCoordY(1);
    currentCarAngle = -90;

    Pair src = make_pair(coordToCellY(currentCarPosCoord_Y), coordToCellX(currentCarPosCoord_X));

    LV1(src);
    printDestinations();
    algorithmIntersectionDetection();
    findFullPath(src);
    createDirectionVector();

    return 1;
}

bool IntersectionDetection() {

    int sensorCount = 0;
    for (int i = 0; i < num_sensors; i++)
    {
        if (virtualCarSensorStates[i] == 0)
        {

            sensorCount += 1.0;
        }
    }

    if ((virtualCarSensorStates[5] == 0 && virtualCarSensorStates[1] == 0) || sensorCount >= 6) {
        return true;
    }
    else {
        return false;
    }


}

void nextMove() {

}



int virtualCarUpdate()
{
    //{----------------------------------
    //process sensor state information
    float halfTiltRange = (num_sensors - 1.0) / 2.0;
    float tiltSum = 0.0;
    float blackSensorCount = 0.0;
    for (int i = 0; i < num_sensors; i++)
    {
        if (virtualCarSensorStates[i] == 0)
        {
            float tilt = (float)i - halfTiltRange;
            tiltSum += tilt;
            blackSensorCount += 1.0;
        }
    }
    //}------------------------------------


    float savedAngle;

    bool intersection = IntersectionDetection();

    Pair currentPosPair = make_pair(coordToCellX(currentCarPosCoord_X), coordToCellY(currentCarPosCoord_Y));



    //if(coordToCellX(currentCarPosCoord_X) ==  && coordToCellY(currentCarPosCoord_Y)

    //{------------------------------------
    //updat linear and rotational speed based on sensor information


    if (intersection || destinationFlag) {
        if (!intersectionFlag) {
            updateCount = 0;
            moveCount++;
            cout << "moveCount: " << moveCount << "\n";
            cout << "direction: " << directions[moveCount] << "\n";
            savedAngle = currentCarAngle;
            cout << "savedAngle : " << savedAngle << "\n";
        }

        destinationFlag = false;
        intersectionFlag = true;

    }

    if (intersectionFlag == true) {
        updateCount++;
        if (updateCount > 100) {
            setVirtualCarSpeed(0.0, virtualCarAngularSpeed_seed);
            switch (directions[moveCount]) {
            case 'l':

                if (currentCarAngle > 175 && currentCarAngle < 185) {
                    intersectionFlag = false;
                }
                break;
            case 'r':

                if (currentCarAngle > 355 || currentCarAngle < 5) {
                    intersectionFlag = false;
                }
                break;
            case 'u':

                if (currentCarAngle > 85 && currentCarAngle < 95) {
                    intersectionFlag = false;
                }
                break;
            case 'd':


                if (currentCarAngle > 265 && currentCarAngle < 275) {
                    intersectionFlag = false;
                }
                break;
            case 't':
                if (savedAngle > 60 && savedAngle < 120) {
                    if (currentCarAngle > 265 && currentCarAngle < 275) {
                        intersectionFlag = false;

                    }
                }
                else if (savedAngle > 150 && savedAngle < 210) {
                    if (currentCarAngle > 355 || currentCarAngle < 5) {
                        intersectionFlag = false;

                    }
                }
                else if (savedAngle > 240 && savedAngle < 300) {
                    if (currentCarAngle > 85 && currentCarAngle < 95) {
                        intersectionFlag = false;

                    }
                }
                else if (savedAngle > 330 || savedAngle < 30) {
                    if (currentCarAngle > 175 && currentCarAngle < 185) {
                        intersectionFlag = false;

                    }
                }
                break;
            }
        }
        else {
            setVirtualCarSpeed(virtualCarLinearSpeed_seed, 0.0);
        }
    }

    else if (blackSensorCount > 0.0) {
        setVirtualCarSpeed(virtualCarLinearSpeed_seed, virtualCarAngularSpeed_seed * tiltSum);
    }
    else {
        setVirtualCarSpeed(0.0, virtualCarAngularSpeed_seed);
    }


    for (int i = 0; i < destinations.size(); i++) {
        if (coordToCellX(currentCarPosCoord_X) == destinations[i].second && coordToCellY(currentCarPosCoord_Y) == destinations[i].first) {
            if (directions[moveCount + 1] == 't') {
                if (!(find(beenToDestinations.begin(), beenToDestinations.end(), destinations[i]) != beenToDestinations.end())) {
                    destinationFlag = true;
                }
                beenToDestinations.push_back(destinations[i]);
            }
        }
    }


    return 1;
}


void LV1(Pair startLoca) {
    // save all the valid cells into Unvisited List
    vector<Pair>::iterator it;
    bool ignorePathFlag = false;
    Pair desti;
    int randomIndex;
    int sameCount = 0;
    int ranIndex = 0;
    // Visited.insert(startLoca);
    // save all the valid cells into the unvisited set list first
    for (int i = 0; i < ROW; i++) {
        for (int j = 0; j < COL; j++) {
            if (map[i][j] == 0) {
                Unvisited.push_back(make_pair(i, j));
            }
        }
    }
    while (Visited.size() != Unvisited.size()) {
        // keep running until the map is completely visited
        // choose a random index of the destination point
        randomIndex = rand() % (Unvisited.size());
        if (std::find(Visited.begin(), Visited.end(), Unvisited[randomIndex]) != Visited.end()) {
            randomIndex = rand() % (Unvisited.size());
        }
        else {
            cout << "the random index picked for lv1 destination: " << randomIndex << "\n";
            desti = Unvisited[randomIndex];
            cout << " The destination picked: " << desti.first << ", " << desti.second << "\n";

            // run the A*
            aStarSearch(map, startLoca, desti);
            if (Visited.size() < 110) {
                for (int x = 0; x < Path.size(); x++) {
                    if (std::find(Visited.begin(), Visited.end(), Path[x]) != Visited.end()) {
                        sameCount++;
                    }
                }
                if (sameCount > int(Path.size() * 0.7)) {
                    ignorePathFlag = true;
                    cout << "Ignored!\n";
                }
                else {
                    ignorePathFlag = false;
                }
            } else {
            }
            //get the path
            // if the cell's been visited then add to the visited list
            // then remove from the unvisited list
            if (!ignorePathFlag) {
                destinations.push_back(desti);
                for (int k = 0; k < Path.size(); k++) {// The size of the fullPath row
                    for (int a = 0; a < Unvisited.size(); a++) {
                        if (Unvisited[a] == Path[k]) { // renew the unvisited cell
                            if (std::find(Visited.begin(), Visited.end(), Unvisited[a]) != Visited.end()) {
                            }
                            else {
                                Visited.push_back(Unvisited[a]);
                            }
                        }
                    }
                }
                startLoca = desti;
            }
            // renew the start location
            sameCount = 0;

        }
    }
    return;
}

int main(int argc, char** argv)
{
    FungGlAppMainFuction(argc, argv);

    return 0;
}
