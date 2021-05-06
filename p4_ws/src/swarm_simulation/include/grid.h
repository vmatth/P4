#pragma once

#include <iostream>

#include <vector>
#include <cmath>
#include <states.h>

#include <position.h>
#include <index.h>
#include <string>

#include <markers.h>

#include <aStar.h>

struct CellInfo{
    int id;
    Position pos;
};

using namespace std;

struct Grid{
    Position pos; //Starting position for the grid
    double size; //x & y size of the grid
};

/*class Cell{
private:
    Grid grid;
    State state = Unexplored;
public:

    void UpdateState(State status){
        state = status;
    }

    Cell(){
        cout << "Cell" << endl;
    }
    
    Cell(double cellSize, Position pos){
        grid.size = cellSize;
        grid.pos = pos;

    }

    Position GetPosition(){
        return grid.pos;
    }

    int GetState(){
        return state;
    }

    
};*/

/*class SubArea{
private:
    Grid grid;
    Index index;

public:

    vector<Cell> cells;

    double getsubGridSize(){
        return grid.size;
        
    }
    
    //Constructor for Sub Areas. Creates cells inside each subarea.
    SubArea(double size, Position pos, double cellSize, Index _index){
        grid.size = size;
        grid.pos = pos;
        index = _index;

        //Creates cells inside SubArea
        //cout << "Creating new Cell with size: " << cellSize << endl;

        //cout << "Cell Size: " << cellSize << endl;
        int numCells = (size / cellSize) * (size / cellSize); 

       // cout << "Number of cells: " << numCells << endl;

        for(int y = 0; y < sqrt(numCells); y++){
            for(int x = 0; x < sqrt(numCells); x++){
                //cout << "(x: " << x << ", y: " << y << ")" << endl;

                Position newPos;
                newPos.x = (x * cellSize) + (cellSize/2) + grid.pos.x;
                newPos.y = (y * cellSize) + (cellSize/2) + grid.pos.y;

                Cell cell(cellSize, newPos);

                cells.push_back(cell);
                
            }
        }


    }

    // Returns start position of specific sub area
    Position GetPosition(){
        return grid.pos;
    }

    int GetCellIndex(Position cellPos){
        //Loop all cells in the subArea

        //Convert the position to ints or else this doesn't work and no idea why

        float fuckX = 0.0f;
        float fuckY = 0.0f;

        fuckX = cellPos.x * 10;
        fuckY = cellPos.y * 10;

        int x = fuckX;
        int y = fuckY;

        // int x = cellPos.x * 10;
        // int y = cellPos.y * 10;

        //cout << "x: " << x << " y:" << y << endl; 

        for(int i = 0; i < cells.size(); i++){
            //Convert to int again cuz I'm on x-games mode

            float _fuckX = 0.0f;
            float _fuckY = 0.0f;
            _fuckX = cells[i].GetPosition().x * 10;
            _fuckY = cells[i].GetPosition().y * 10;

            int _x = _fuckX;
            int _y = _fuckY;    
            

            if(x == _x && y == _y){
                cout << "_x: " << _x << " _y:" << _y << endl; 
                return i;
            }      
        }
        return -1; //If cell doesn't exist
    }
};*/


struct SubAreaInterval{
    int startX = 0;
    int endX = 0;

    int startY = 0;
    int endY = 0;
};

class SuperArea{
private:
    //Grid grid;
    //vector<SubArea> subAreas;
    //double cellDistance; //Distance between each cell

    //SubArea interval (first is in index, second is in position)
    SubAreaInterval** subAreaIntervals;
    SubAreaInterval** subAreaIntervalsPosition;


    //Grid contains the state of the cell
    int** grid;

    int** newgrid;

    //Grid contains information of which subarea the cell is in
    //The grid will look something like:
    // 2 2 3 3
    // 2 2 3 3
    // 0 0 1 1
    // 0 0 1 1
    Index** gridSubAreas;

    //Add a position (x,y) for each grid
    Position** gridPositions;

    //Add a unique id for each grid. Used in rViz
    int** gridIds;

    int subAreaSize, numSubAreas, numSubAreasSqrt;

    double cellDistance;

    int rows, cols;
public:
    //Super Area constructor. Creates SubAreas inside. numSubAreas can be 2², 3², 4² ...
    SuperArea(int size, int _numSubAreas, double _cellDistance){

        numSubAreas = _numSubAreas;
        cellDistance = _cellDistance;

        cout << "New grid with Size: " << size << ", cell distance: " << cellDistance << " and num of sub areas: " << numSubAreas << endl;
        rows = size / cellDistance;
        cols = size / cellDistance;
        cout << "Rows & Cols: " << rows << endl;

        numSubAreasSqrt = int(sqrt(numSubAreas));
        int subAreaSize = size / numSubAreasSqrt;
        
        cout << "Number of Sub Areas (Sqrt): " << numSubAreasSqrt << endl;
        cout << "Sub Area Size: " << subAreaSize << endl;

        //Creates the array like this because c++ magic
        subAreaIntervals = new SubAreaInterval*[numSubAreasSqrt];
        for(int i = 0; i < numSubAreasSqrt; i++)
            subAreaIntervals[i] = new SubAreaInterval[numSubAreasSqrt];

        subAreaIntervalsPosition = new SubAreaInterval*[numSubAreasSqrt];
        for(int i = 0; i < numSubAreasSqrt; i++)
            subAreaIntervalsPosition[i] = new SubAreaInterval[numSubAreasSqrt];

        //Creates an interval for each subArea
        for(int x = 0; x < numSubAreasSqrt; x++){
            for(int y = 0; y < numSubAreasSqrt; y++){
                subAreaIntervals[x][y].startX = x * (rows/numSubAreasSqrt);
                subAreaIntervals[x][y].endX = (x * (rows/numSubAreasSqrt)) + (rows/numSubAreasSqrt) - 1;
                subAreaIntervals[x][y].startY = y * (cols/numSubAreasSqrt);
                subAreaIntervals[x][y].endY = (y * (cols/numSubAreasSqrt)) + (cols/numSubAreasSqrt) - 1;
                subAreaIntervalsPosition[x][y].startX = x * subAreaSize;
                subAreaIntervalsPosition[x][y].endX = (x * subAreaSize) + subAreaSize;
                subAreaIntervalsPosition[x][y].startY = y * subAreaSize;
                subAreaIntervalsPosition[x][y].endY = y * subAreaSize + subAreaSize;

                cout << "SubArea [" << x << "]" << "[" << y << "]" << " x Interval: (" << subAreaIntervals[x][y].startX << "," << subAreaIntervals[x][y].endX << ") y Interval: (" << subAreaIntervals[x][y].startY << "," << subAreaIntervals[x][y].endY << ")" << endl;
                cout << "SubArea [" << x << "]" << "[" << y << "]" << " x Pos Interval: (" << subAreaIntervalsPosition[x][y].startX << "," << subAreaIntervalsPosition[x][y].endX << ") y Pos Interval: (" << subAreaIntervalsPosition[x][y].startY << "," << subAreaIntervalsPosition[x][y].endY << ")" << endl;
            }
           
        }

        //Create arrays for grids
        grid = new int*[rows];
        for(int i = 0; i < rows; i++)
            grid[i] = new int[rows];

        //Start the grid as unexplored
        for(int x = 0; x < rows; x++){
            for(int y = 0; y < cols; y++){
                grid[x][y] = Unexplored;
            }
        }

        gridSubAreas = new Index*[rows];
        for(int i = 0; i < rows; i++)
            gridSubAreas[i] = new Index[rows];

        gridPositions = new Position*[rows];
        for(int i = 0; i < rows; i++)
            gridPositions[i] = new Position[rows];     

        gridIds = new int*[rows];
        for(int i = 0; i < rows; i++)
            gridIds[i] = new int[rows];          

        //for loop magic do not question
        //Adds a subArea number for each cell
        for(int r = 0; r < rows; r++){
            for(int c = 0; c < cols; c++){
                for(int x = 0; x < numSubAreasSqrt; x++){
                    for(int y = 0; y < numSubAreasSqrt; y++){
                        if(r >= subAreaIntervals[x][y].startX && r <= subAreaIntervals[x][y].endX){
                            if(c >= subAreaIntervals[x][y].startY && c <= subAreaIntervals[x][y].endY){
                                //cout << "Cell (" << r << "," << c << ") is in SubArea [" << x << "][" << y << "]" << endl; 
                                gridSubAreas[r][c].x = x;
                                gridSubAreas[r][c].y = y;
                            }
                        }
                    }
                }
            }
        }

        //Adds a position (x,y) for each cell
        for (int x = 0; x < rows; x++){
            for (int y = 0; y < cols; y++){
                gridPositions[x][y].x = x * cellDistance + (cellDistance/2);
                gridPositions[x][y].y = y * cellDistance + (cellDistance/2);
                //if(x < 20 && y < 20)
                    //cout << "[" << x << "]" << "[" << y << "]: " << "(" << gridPositions[x][y].x << "," << gridPositions[x][y].y << ")" << endl;
            }
        }

        //Gives each cell an unique id
        int id = 0;
        for (int y = 0; y < cols; y++){
            for (int x = 0; x < rows; x++){
                gridIds[x][y] = id;
                id++;
            }
        }
        
    }

    // Returns the index of which sub area the specific robot is in E.g [0,1] (x,y)
    Index GetSubArea(Position pos){
        //cout << "Getting Sub Area for pos (" << pos.x << "," << pos.y << ")" << endl;
        Index subA; //Index value to return later

        //Loops all of the subAreaIntervals array, and checks which subAreaInterval "pos" is located in
        for(int x = 0; x < numSubAreasSqrt; x++){
            for(int y = 0; y < numSubAreasSqrt; y++){

                //cout << "Checking in Subarea[" << x << "][" << y << "]" << endl;


                if(pos.x >= subAreaIntervalsPosition[x][y].startX && pos.x < subAreaIntervalsPosition[x][y].endX){
                    if(pos.y >= subAreaIntervalsPosition[x][y].startY && pos.y < subAreaIntervalsPosition[x][y].endY){
                        cout << "Found Sub Area: [" << x << "][" << y << "] for pos (" << pos.x << "," << pos.y << ")" << endl;
                        Index subA;
                        subA.x = x;
                        subA.y = y;
                        return subA;
                    } 
                }
            }
        }
        //In cases where the position is not located in any subarea
        cout << "SubArea not found for pos (" << pos.x << "," << pos.y << ")" << endl;
        subA.x = -1;
        subA.y = -1;       
        return subA;
    }

    //Gets a position, and finds out the corresponding Cell Index. E.g Pos(5, 0.4) has a Index of [8, 2];
    Index GetCellIndex(Position pos){
        Index cellIndex;
        //Convert the position to int, as it is easier to compare due to decimals
        float xf = pos.x * 10;
        float yf = pos.y * 10;

        int x_ = xf;
        int y_ = yf;

        //Loop all cell positions in "gridPositions" and compare the positions
        for(int x = 0; x < rows; x++){
            for(int y = 0; y < cols; y++){
                //Convert the position to int, as it is easier to compare due to decimals
                float _xf = gridPositions[x][y].x * 10;
                float _yf = gridPositions[x][y].y * 10;

                int _x = _xf;
                int _y = _yf;    

                if(x_ == _x && y_ == _y){
                    cellIndex.x = x;
                    cellIndex.y = y;
                    //cout << "GetCellIndex: [" << x << "][" << y << "] for pos (" << pos.x << "," << pos.y << ")" << endl;;
                    return cellIndex;
                }            
            }
        }
        //If the cell couldn't be found
        cellIndex.x = -1;
        cellIndex.y = -1;
        cout << "Could not find cell index for pos (" << pos.x << "," << pos.y << ")" << endl;
        return cellIndex;
    }

    int GetNumSubAreas(){
        return numSubAreas;
    }

    int GetRows(){
        return rows;
    }

    Position GetCellPosition(Index i){
        return gridPositions[i.x][i.y];
    }

    /*SubArea& GetSubArea(int index){
        return subAreas[index];
    }*/

    //Mark Cell with a State E.g Wall, Unexplored or Free
    void ChangeCellState(Position cellPos, State _state){

        Index cellIndex = GetCellIndex(cellPos);        

        if(cellIndex.x != -1){ //Check if the cell exists
            cout << "Changing cell state(" << cellPos.x << " , " << cellPos.y << "), State: " << _state << endl;
            grid[cellIndex.x][cellIndex.y] = _state;
        }
        else
            cout << "Change Cell State failed, as the cell could not be found" << endl;
        
    }

    bool CompareSubAreas(Position pos1, Position pos2){

        Index subArea1 = GetSubArea(pos1);
        Index subArea2 = GetSubArea(pos2);

        if(subArea1.x == subArea2.x && subArea1.y == subArea2.y){
            cout << "Both positions are in same sub area!" << endl;
            return true;
        }

        cout << "Positions are not in same sub area" << endl;
        return false;
    }
    
    //Checks if the new position "cellPos" is an existing point
    bool CheckIfCellExists(Position cellPos){

        Index cellIndex = GetCellIndex(cellPos);
        if(cellIndex.x != -1){
            cout << "Cell Exists! (" << cellPos.x << "," << cellPos.y << ")" << endl;
            return true;
        }
        else{
            cout << "Cell does not Exist! (" << cellPos.x << "," << cellPos.y << ")" << endl;
            return false;
        }
    }

    //Returns the cell's state E.g Unexplored, Wall or Free
    int GetCellState(Position cellPos){

        Index cellIndex = GetCellIndex(cellPos);        

        if(cellIndex.x != -1){ //Check if the cell exists
            cout << "Get Cell State for pos (" << cellPos.x << " , " << cellPos.y << "): " << grid[cellIndex.x][cellIndex.y] << endl;
            return grid[cellIndex.x][cellIndex.y];
        } 

        cout << "Could not find cell state for pos (" << cellPos.x << " , " << cellPos.y << ")" << endl;
        return -1;
    }

    int GetCellState(Index cellIndex){
        cout << "Get Cell State for index: [" << cellIndex.x << "][" << cellIndex.y << "]: " << grid[cellIndex.x][cellIndex.y] << endl;
        return grid[cellIndex.x][cellIndex.y];
    }

    //Gets the next cell that the turtlebot will travel to, based on where the robot is facing and its currentPosition.
    Position GetNextCell(Position currentPosition, double yaw){
        //Get position of up, down, left and right cells
        Position upPos;
        Position downPos;
        Position rightPos;
        Position leftPos;
        
        upPos.x = currentPosition.x;
        upPos.y = currentPosition.y + cellDistance;
        
        downPos.x = currentPosition.x;
        downPos.y = currentPosition.y - cellDistance;
        
        rightPos.x = currentPosition.x + cellDistance;
        rightPos.y = currentPosition.y;
        
        leftPos.x = currentPosition.x - cellDistance;
        leftPos.y = currentPosition.y;

        //cout << "Robot yaw: " << yaw << endl;

        
        //Check if these cells exist.

        //Finds the angle difference from the robot's rotation to each point (up down left right) 
        //Is used for finding the closest point from the robot.
        double upAngle = 360;
        double downAngle = 360;
        double leftAngle = 360;
        double rightAngle = 360; //Default 360 

        //cout << "Up" << endl;
        if(CheckIfCellExists(upPos)){
            upAngle = abs(90 - yaw);
            cout << "Up angle: " << upAngle << " | ";
        }

        cout << "Down" << endl;
        if (CheckIfCellExists(downPos)){
            downAngle = abs(270 - yaw);
            cout << "Down Angle: " << downAngle << " | ";
        }

       // PrintPosition(leftPos, "LeftPos");

        cout << "Left" << endl;
        if (CheckIfCellExists(leftPos)){
            leftAngle = abs(180 - yaw);
            cout << "Left Angle: " << leftAngle << " | ";
        }

        cout << "Right" << endl;
        if (CheckIfCellExists(rightPos)){
            //quik mafs
            if(yaw > 180){
                rightAngle = abs(360 - yaw);
            }
            else{
                rightAngle = abs(0 - yaw);
            }
            cout << "Right Angle: " << rightAngle;
        }

        //Compare all angles, and find the closest one
        double arr[] = {upAngle, downAngle, rightAngle, leftAngle};

        double temp = arr[0];
        int index = 0;
        for(int i = 0; i < 4; i++){
            if(arr[i] <= temp){
                temp = arr[i];
                index = i;
            }
        }

        //Save the index of the angles, 0: up, 1: down, 2: right, 3: left
        //cout << "INDEX: " << index << endl;

        //Get the front position
        Position front, right, left, back;
        //bruh those if statements
        if (index == 0){
            cout << " || Up is front" << endl;
            front.x = upPos.x;
            front.y = upPos.y;

            right.x = rightPos.x;
            right.y = rightPos.y;

            left.x = leftPos.x;
            left.y = leftPos.y;

            back.x = downPos.x;
            back.y = downPos.y;

        } else if (index == 1){
            cout << " || Down is front" << endl;
            front.x = downPos.x;
            front.y = downPos.y;

            right.x = leftPos.x;
            right.y = leftPos.y;

            left.x = rightPos.x;
            left.y = rightPos.y;

            back.x = upPos.x;
            back.y = upPos.y;

        } else if (index == 2){
            cout << " || Right is front" << endl;
            front.x = rightPos.x;
            front.y = rightPos.y;

            right.x = downPos.x;
            right.y = downPos.y;

            left.x = upPos.x;
            left.y = upPos.y;

            back.x = leftPos.x;
            back.y = leftPos.y;

        } else if (index == 3){
            cout << " || Left is front" << endl;
            front.x = leftPos.x;
            front.y = leftPos.y;

            right.x = upPos.x;
            right.y = upPos.y;

            left.x = downPos.x;
            left.y = downPos.y;

            back.x = rightPos.x;
            back.y = rightPos.y;
        } 

        PrintPosition(front, "Front: ");
        PrintPosition(right, "Right: ");
        PrintPosition(left, "Left: ");
        PrintPosition(back, "Back: ");

        if(GetCellState(front) == Unexplored && CompareSubAreas(front, currentPosition)){
            cout << "Front cell is unexplored!" << endl;
            return front;
        }
        else if(GetCellState(right) == Unexplored && CompareSubAreas(right, currentPosition)){
            cout << "Right cell is unexplored!" << endl; 
            return right;
        }
        else if(GetCellState(left) == Unexplored && CompareSubAreas(left, currentPosition)){
            cout << "Left cell is unexplored!" << endl; 
            return left;
        }
        else if(GetCellState(back) == Unexplored && CompareSubAreas(back, currentPosition)){
            cout << "Back cell is unexplored!" << endl;
            return back;
        } else {
            cout << "Find nearest unexplored cell" << endl;
            return GetNearestCellPosition(currentPosition, Unexplored, true);
        }
    }

    //Checks if the newly marked point is close to any of the cells. If true, then it will nark the cell as "Wall"
    //Returns info on the cell that has been updated.
    CellInfo NewWallPoint(Position wallPos){

        CellInfo cellInfo; //Returns info on the cell that has been updated. Used for rviz marker    

        //Index cellIndex = GetCellIndex(wallPos); //

        Position relativeCell; 

        //Loop all grid positions
        for(int x = 0; x < rows; x++){
            for(int y = 0; y < cols; y++){
                //Get the relative position from the new grid position to the wall position
                relativeCell.x = gridPositions[x][y].x - wallPos.x;
                relativeCell.y = gridPositions[x][y].y - wallPos.y;

                //If the relativePosition is low (the wall is close to this cell position)
                if(abs(relativeCell.x) < 0.2 && abs(relativeCell.y) < 0.2)
                {

                    //Check if the point hasn't already been marked as wall
                    if(grid[x][y] != Wall){
                        //Update state to wall
                        grid[x][y] = Wall;
                        
                        cellInfo.id = gridIds[x][y];
                        cellInfo.pos.x = gridPositions[x][y].x;
                        cellInfo.pos.y = gridPositions[x][y].y;

                        PrintPosition(gridPositions[x][y], "Marking wall at: ");

                        return cellInfo;
                    }
                }
            }
        }

        
        cellInfo.id = -1;
        return cellInfo;
    }

    void PrintPosition(Position pos, string text){
        cout << text << "(" << pos.x << ", " << pos.y << ")" << endl;
    }

    int GetCellId(Position cellPos){
        Index cellIndex = GetCellIndex(cellPos);
        if(cellIndex.x != -1)
            return gridIds[cellIndex.x][cellIndex.y];
        else
            return -1;
    }

    //Checks if the robot will collide with the goalPos, when a new wall has been found
    bool CheckForCollision(Position wallPos, Position goalPos){
        //cout << "Checking for collision" << endl;
       // PrintPosition(wallPos, "wall pos");
        //PrintPosition(goalPos, "goal pos");

        float fuckX = 0.0f;
        float fuckY = 0.0f;

        fuckX = wallPos.x * 10;
        fuckY = wallPos.y * 10;

        int x = fuckX;
        int y = fuckY;

        //cout << "x: " << x << " y: " << y << endl; 

        float _fuckX = 0.0f;
        float _fuckY = 0.0f;

        _fuckX = goalPos.x * 10;
        _fuckY = goalPos.y * 10;

        int _x = _fuckX;
        int _y = _fuckY;

        //cout << "_x: " << _x << " _y: " << _y << endl; 


        if(x == _x && y == _y){
            return true;
        }
        else{
            return false;
        }
        //return true if they are the same
    }

    //Gets the cell that is closest to the the source position, and the cell must be state: "cell state"
    Index GetNearestCellIndex(Position sourcePos, State cellState, State secondState, bool sameSubArea){

        Index nearestCell; //Cell index that will be returned
        nearestCell.x = -1; //Will be overriden if it finds a nearest cell

        double shortestDistance = 1000; //Temporary value for distance

        for(int r = 0; r < rows; r++){
            for(int c = 0; c < cols; c++){
                double distToCell = sqrt(pow(sourcePos.x - gridPositions[r][c].x, 2) 
                                        + pow(sourcePos.y - gridPositions[r][c].y, 2));
            
                //If new cell is closer
                if(distToCell <= shortestDistance && (grid[r][c] == cellState || grid[r][c] == secondState)){
                    //Check if new cell has to be in same subarea
                    if(sameSubArea){
                        if(CompareSubAreas(sourcePos, gridPositions[r][c])){
                            shortestDistance = distToCell;
                            nearestCell.x = r;
                            nearestCell.y = c;
                        }
                    }
                    else{
                        shortestDistance = distToCell;
                        nearestCell.x = r;
                        nearestCell.y = c;
                    }
                }
            }
        }

        if(nearestCell.x == -1)
            cout << "There were no cells near pos (" << sourcePos.x << "," << sourcePos.y << ")" << endl;
        return nearestCell;
    }

    Position GetNearestCellPosition(Position sourcePos, State cellState, bool sameSubArea){
        Position nearestCell; //Position that will be returned

        //Use same function for finding index
        Index i = GetNearestCellIndex(sourcePos, cellState, cellState, sameSubArea);
        //If a nearest cell was found
        if(i.x != 1){
            cout << "Cell (" << gridPositions[i.x][i.y].x << "," << gridPositions[i.x][i.y].y << ") is near the source pos (" << sourcePos.x << "," << sourcePos.y << ")" << endl;
            nearestCell = gridPositions[i.x][i.y];
            return nearestCell;
        }

        nearestCell.x = -1;
        nearestCell.y = -1;
        return nearestCell;
    }
  


    list<Position> AStarPathfinding(Position startPos, Position endPos){

        //GetNearestCellPosition(endPos, Unexplored, false);
        Index endPosIndex = GetNearestCellIndex(endPos, Unexplored, Unexplored, false);
        Index startPosIndex = GetNearestCellIndex(startPos, Unexplored, Free, false); 

        PrintPosition(startPos, "Starting pathfinding from: ");
        PrintPosition(endPos, "to: ");

        cout << "Start Pos INDEX: (" << startPosIndex.x << "," << startPosIndex.y << ")" << endl;
        cout << "End Pos INDEX: (" << endPosIndex.x << "," << endPosIndex.y << ")" << endl;

        //Flip x & y before passing values to A*
        int temp;
        temp = startPosIndex.x;
        startPosIndex.x = startPosIndex.y;
        startPosIndex.y = temp;
        temp = endPosIndex.x;
        endPosIndex.x = endPosIndex.y;
        endPosIndex.y = temp;

        //Converts to grid to a new grid that A* can use
        gridtoaStar();

        list<Index> tempPos; //The positions from A* will be stored in this list (as indexes)
        tempPos = aStarPATH(newgrid, rows, cols, startPosIndex, endPosIndex);

        list<Position> path; //The positions from A* will be stored in this list (as positions)

        //cout << "The path is: " << endl;
        //The path is received in index. Convert to position here
        int i = 0;
        for (auto const& p : tempPos) {
            Position newPos = gridPositions[int(p.x)][int(p.y)];
            //cout << "[" << i << "] Index: (" << p.x << "," << p.y << ")" << endl;
            //cout << "[" << i << "] Position: (" << newPos.x << "," << newPos.y << ")" << endl;
            path.push_back(newPos);
            i++;
        }
        cout << "------------------------------------" << endl;

        //Show the grid in terminal. Create a new grid "terminalGrid"
        int **terminalGrid;
        terminalGrid = new int*[rows];
        for(int i = 0; i < rows; i++)
            terminalGrid[i] = new int[rows];

        //Loop terminalGrid and copy "newGrid's" values to it,
        for(int x = 0; x < rows; x++){
            for(int y = 0; y < cols; y++){
                terminalGrid[x][y] = newgrid[x][y];
                //If the value is a path. Change to path to "2", this will be later visualized in the terminal
                for (auto const& p : tempPos) {
                    if(x == p.x && y == p.y){
                        terminalGrid[x][y] = 2;
                    }
                }
            }
        }

        //Show terminalGrid in the terminal
        for (int i = rows - 1; i >= 0; i--)
        {
            for (int j = 0; j < rows; ++j)
            {
                if(terminalGrid[j][i] == 0)
                    std::cout << "X ";
                else if(terminalGrid[j][i] == 1)
                    std::cout << "- ";
                else
                    std::cout << "# "; //Show "+" if path
            }
            std::cout << std::endl;
        }  

        return path;

    }


    /*int CheckFreeArea(){
        for(int i = 0; i < numRobots; i++){
            for(int i = 0; i < GetNumSubAreas(); i++){
                if(robot position is not in subarea){
                    return i; 
                }
            }
        }
    }*/

    void gridtoaStar(){
        //Create arrays for grids
        newgrid = new int*[rows];
        for(int i = 0; i < rows; i++)
            newgrid[i] = new int[rows];

        for(int x = 0; x < rows; x++){
            for(int y = 0; y < cols; y++){
                if (grid[x][y] == Unexplored || grid[x][y] == Free){
                    newgrid[x][y] = 1;
                }
                else newgrid[x][y] = 0;
            }
        }

        /*for (int i = rows - 1; i >= 0; i--)
        {
            for (int j = 0; j < rows; ++j)
            {
                std::cout << newgrid[j][i] << ' ';
            }
            std::cout << std::endl;
        }  */

    }

    void AAA(){
        int **grod;
        grod = new int*[4];
        for(int i = 0; i < 4; i++)
            grod[i] = new int[4];

        int **grad;
        grad = new int*[4];
        for(int i = 0; i < 4; i++)
            grad[i] = new int[4];
        
        grod[0][0] = 1;
        grod[1][0] = 0;
        grod[2][0] = 1;
        grod[3][0] = 1;

        grod[0][1] = 1;
        grod[1][1] = 0;
        grod[2][1] = 0;
        grod[3][1] = 1;

        grod[0][2] = 1;
        grod[1][2] = 0;
        grod[2][2] = 0;
        grod[3][2] = 1;

        grod[0][3] = 1;
        grod[1][3] = 1;
        grod[2][3] = 1;
        grod[3][3] = 1;


        for (int i = 4 - 1; i >= 0; i--)
        {
            for (int j = 0; j < 4; ++j)
            {
                std::cout << grod[j][i] << ' ';
            }
            std::cout << std::endl;
        }  
        cout << "---------" << endl;

        Index startPosIndex;
        startPosIndex.x = 0;
        startPosIndex.y = 0;

        Index endPosIndex;
        endPosIndex.x = 2;
        endPosIndex.y = 0;  


        //The end and start pos must be flipped before giving them to A*

        endPosIndex.y = 2;
        endPosIndex.x = 0;

        list<Index> tempPos;

        tempPos = aStarPATH(grod, 4, 4, startPosIndex, endPosIndex);

        list<Position> path;

        cout << "The path is: " << endl;
        //The path is received in index. Convert to position here

        int i = 0;
        for (auto const& p : tempPos) {
            Position newPos = gridPositions[int(p.x)][int(p.y)];
            cout << "[" << i << "] Index: (" << p.x << "," << p.y << ")" << endl;
            cout << "[" << i << "] Position: (" << newPos.x << "," << newPos.y << ")" << endl;

            path.push_back(newPos);
            i++;
        }





        cout << "------------------------------------" << endl;

        //Show the grid in terminal. Create a new grid "terminalGrid"
        int **terminalGrid;
        terminalGrid = new int*[4];
        for(int i = 0; i < 4; i++)
            terminalGrid[i] = new int[4];

        //Loop terminalGrid and copy "newGrid's" values to it,
        for(int x = 0; x < 4; x++){
            for(int y = 0; y < 4; y++){
                terminalGrid[x][y] = grod[x][y];
                //If the value is a path. Change to path to "2", this will be later visualized in the terminal
                for (auto const& p : tempPos) {
                    if(x == p.x && y == p.y){
                        terminalGrid[x][y] = 2;
                    }
                }
            }
        }

        for (int i = 4 - 1; i >= 0; i--)
        {
            for (int j = 0; j < 4; ++j)
            {
                if(terminalGrid[j][i] != 2)
                    std::cout << terminalGrid[j][i] << ' ';
                else
                    std::cout << "+ ";
            }
            std::cout << std::endl;
        }  

    }
};