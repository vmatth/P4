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
    // [0,1] [0,1] [1,1] [1,1] 
    // [0,1] [0,1] [1,1] [1,1]
    // [0,0] [0,0] [1,0] [1,0]
    // [0,0] [0,0] [1,0] [1,0]
    Index** gridSubAreas;

    //Add a position (x,y) for each grid
    Position** gridPositions;

    //Add a unique id for each grid. Used in rViz
    int** gridIds;

    int subAreaSize, numSubAreas, numSubAreasSqrt;

    double cellDistance;

    int rows, cols;
public:

    int GetNumSubAreasSqrt(){
        return numSubAreasSqrt;
    }

    Index GetSubAreaIndex(Index i){
        return gridSubAreas[i.x][i.y];
    }

    double GetCellDistance(){
        return cellDistance;
    }

    SubAreaInterval GetSubAreaInterval(int r, int c){
        return subAreaIntervalsPosition[r][c];
    }

    //Super Area constructor. Creates SubAreas inside. numSubAreas can be 2², 3², 4² ...
    SuperArea(int size, int _numSubAreas, double _cellDistance){

        numSubAreas = _numSubAreas;
        cellDistance = _cellDistance;

        cout << "New grid with Size: " << size << ", cell distance: " << cellDistance << " and num of sub areas: " << numSubAreas << endl;
        float calc = size / cellDistance; //Calculate using floats due to decimals in cellDistance
        
        rows = round(calc); //Convert to int like this becuase cass++
        cols = round(calc);
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

        //Print subareas grid
        //Show  in the terminal

        // for (int i = rows - 1; i >= 0; i--)
        // {
        //     for (int j = 0; j < rows; ++j)
        //     {
        //         if(gridSubAreas[j][i].x == 0 && gridSubAreas[j][i].y == 0)
        //             cout << "0 ";
        //         else if(gridSubAreas[j][i].x == 1 && gridSubAreas[j][i].y == 0)
        //             cout << "1 ";
        //         else if(gridSubAreas[j][i].x == 0 && gridSubAreas[j][i].y == 1)
        //             cout << "2 ";                    
        //         else if(gridSubAreas[j][i].x == 1 && gridSubAreas[j][i].y == 1)
        //             cout << "3 ";
        //     }
        //     std::cout << std::endl;
        // }  
        
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
                        //cout << "Found Sub Area: [" << x << "][" << y << "] for pos (" << pos.x << "," << pos.y << ")" << endl;
                        Index subA;
                        subA.x = x;
                        subA.y = y;
                        return subA;
                    } 
                }
            }
        }
        //In cases where the position is not located in any subarea
        //cout << "SubArea not found for pos (" << pos.x << "," << pos.y << ")" << endl;
        subA.x = -1;
        subA.y = -1;       
        return subA;
    }

    //Gets a position, and finds out the corresponding Cell Index. E.g Pos(5, 0.4) has a Index of [8, 2];
    Index GetCellIndex(Position pos){
        Index cellIndex;
        //Convert the position to int, as it is easier to compare due to decimals
        float xf = pos.x * 100;
        float yf = pos.y * 100;

        int x_ = xf;
        int y_ = yf;

        //Loop all cell positions in "gridPositions" and compare the positions
        for(int x = 0; x < rows; x++){
            for(int y = 0; y < cols; y++){
                //Convert the position to int, as it is easier to compare due to decimals
                float _xf = gridPositions[x][y].x * 100;
                float _yf = gridPositions[x][y].y * 100;

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
            //cout << "Changing cell state(" << cellPos.x << " , " << cellPos.y << "), State: " << _state << endl;
            grid[cellIndex.x][cellIndex.y] = _state;
        }
        else
            cout << "Change Cell State failed, as the cell could not be found" << endl;
        
    }

    void ChangeCellState(Index cellIndex, State _state){

        if(cellIndex.x != -1){ //Check if the cell exists
            //cout << "Changing cell state(" << cellPos.x << " , " << cellPos.y << "), State: " << _state << endl;
            grid[cellIndex.x][cellIndex.y] = _state;
        }
        else
            cout << "Change Cell State failed, as the cell could not be found" << endl;
        
    }

    bool CompareSubAreas(Position pos1, Position pos2){

        Index subArea1 = GetSubArea(pos1);
        Index subArea2 = GetSubArea(pos2);

        if(subArea1.x == subArea2.x && subArea1.y == subArea2.y){
            //cout << "Both positions are in same sub area!" << endl;
            return true;
        }

        //cout << "Positions are not in same sub area" << endl;
        return false;
    }
    
    //Checks if the new position "cellPos" is an existing point
    bool CheckIfCellExists(Position cellPos){

        Index cellIndex = GetCellIndex(cellPos);
        if(cellIndex.x != -1){
            //cout << "Cell Exists! (" << cellPos.x << "," << cellPos.y << ")" << endl;
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
            //cout << "Get Cell State for pos (" << cellPos.x << " , " << cellPos.y << "): " << grid[cellIndex.x][cellIndex.y] << endl;
            return grid[cellIndex.x][cellIndex.y];
        } 

        cout << "Could not find cell state for pos (" << cellPos.x << " , " << cellPos.y << ")" << endl;
        return -1;
    }

    int GetCellState(Index cellIndex){
       //cout << "Get Cell State for index: [" << cellIndex.x << "][" << cellIndex.y << "]: " << grid[cellIndex.x][cellIndex.y] << endl;
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
            //cout << "Up angle: " << upAngle << " | ";
        }

        if (CheckIfCellExists(downPos)){
            downAngle = abs(270 - yaw);
            //cout << "Down Angle: " << downAngle << " | ";
        }
        if (CheckIfCellExists(leftPos)){
            leftAngle = abs(180 - yaw);
            //cout << "Left Angle: " << leftAngle << " | ";
        }

        if (CheckIfCellExists(rightPos)){
            //quik mafs
            if(yaw > 180){
                rightAngle = abs(360 - yaw);
            }
            else{
                rightAngle = abs(0 - yaw);
            }
            //cout << "Right Angle: " << rightAngle;
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
            //cout << " || Up is front" << endl;
            front.x = upPos.x;
            front.y = upPos.y;

            right.x = rightPos.x;
            right.y = rightPos.y;

            left.x = leftPos.x;
            left.y = leftPos.y;

            back.x = downPos.x;
            back.y = downPos.y;

        } else if (index == 1){
            //cout << " || Down is front" << endl;
            front.x = downPos.x;
            front.y = downPos.y;

            right.x = leftPos.x;
            right.y = leftPos.y;

            left.x = rightPos.x;
            left.y = rightPos.y;

            back.x = upPos.x;
            back.y = upPos.y;

        } else if (index == 2){
            //cout << " || Right is front" << endl;
            front.x = rightPos.x;
            front.y = rightPos.y;

            right.x = downPos.x;
            right.y = downPos.y;

            left.x = upPos.x;
            left.y = upPos.y;

            back.x = leftPos.x;
            back.y = leftPos.y;

        } else if (index == 3){
            //cout << " || Left is front" << endl;
            front.x = leftPos.x;
            front.y = leftPos.y;

            right.x = upPos.x;
            right.y = upPos.y;

            left.x = downPos.x;
            left.y = downPos.y;

            back.x = rightPos.x;
            back.y = rightPos.y;
        } 

       // PrintPosition(front, "Front: ");
       // PrintPosition(right, "Right: ");
       // PrintPosition(left, "Left: ");
       // PrintPosition(back, "Back: ");

        if(GetCellState(front) == Unexplored && CompareSubAreas(front, currentPosition)){
            //cout << "Front cell is unexplored!" << endl;
            return front;
        }
        else if(GetCellState(right) == Unexplored && CompareSubAreas(right, currentPosition)){
            //cout << "Right cell is unexplored!" << endl; 
            return right;
        }
        else if(GetCellState(left) == Unexplored && CompareSubAreas(left, currentPosition)){
            //cout << "Left cell is unexplored!" << endl; 
            return left;
        }
        else if(GetCellState(back) == Unexplored && CompareSubAreas(back, currentPosition)){
            //cout << "Back cell is unexplored!" << endl;
            return back;
        } else {
            //cout << "Find nearest unexplored cell" << endl;
            Position noPos;
            noPos.x = -1;
            return noPos; //Returns noPos so the program knows that it will switch from PSO to A*
            //GetNearestCellPosition(currentPosition, Unexplored, true);
        }
    }

    //Checks if pos1 and pos2 are "radius" distance from each other
    bool ComparePositions(Position pos1, Position pos2, double distance){
        Position relativePos;
        relativePos.x = pos1.x - pos2.x;
        relativePos.y = pos1.y - pos2.y;

        if(abs(relativePos.x) < distance && abs(relativePos.y) < distance)
        {
            //cout << "Point measured is turtlbot [" << j << "]" << endl;
            return true;
        }  
        return false;
    }

   

    //Checks if the newly marked point is close to any of the cells. If true, then it will nark the cell as "Wall"
    //Returns info on the cell that has been updated.
    CellInfo NewWallPoint(Position wallPos, float cellSpace){

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
                if(abs(relativeCell.x) < (cellSpace/2) && abs(relativeCell.y) < (cellSpace/2))
                {

                    //Check if the point hasn't already been marked as wall
                    if(grid[x][y] != Wall){
                        //Update state to wall
                        grid[x][y] = Wall;
                        
                        cellInfo.id = gridIds[x][y];
                        cellInfo.pos.x = gridPositions[x][y].x;
                        cellInfo.pos.y = gridPositions[x][y].y;

                       // PrintPosition(gridPositions[x][y], "Marking wall at: ");

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


    int GetCellId(Index cellIndex){
        if(cellIndex.x != -1)
            return gridIds[cellIndex.x][cellIndex.y];
        else
            return -1;
    }

    //Checks if the robot will collide with the goalPos, when a new wall has been found
    bool CheckForCollision(Position wallPos, Position goalPos){
        //cout << "Checking for collision" << endl;

        float fuckX = 0.0f;
        float fuckY = 0.0f;

        fuckX = wallPos.x * 100;
        fuckY = wallPos.y * 100;

        int x = fuckX;
        int y = fuckY;

        //cout << "x: " << x << " y: " << y << endl; 

        float _fuckX = 0.0f;
        float _fuckY = 0.0f;

        _fuckX = goalPos.x * 100;
        _fuckY = goalPos.y * 100;

        int _x = _fuckX;
        int _y = _fuckY;

        //cout << "_x: " << _x << " _y: " << _y << endl; 


        if(x == _x && y == _y){
            return true;
        }
        else{
            return false;
        }
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

    Position GetNearestCellPosition(Position sourcePos, State cellState, State secondState, bool sameSubArea){
        Position nearestCell; //Position that will be returned

        //Use same function for finding index
        Index i = GetNearestCellIndex(sourcePos, cellState, secondState, sameSubArea);
        //If a nearest cell was found
        if(i.x != 1){
            //cout << "Cell (" << gridPositions[i.x][i.y].x << "," << gridPositions[i.x][i.y].y << ") is near the source pos (" << sourcePos.x << "," << sourcePos.y << ")" << endl;
            nearestCell = gridPositions[i.x][i.y];
            return nearestCell;
        }

        nearestCell.x = -1;
        nearestCell.y = -1;
        return nearestCell;
    }

    //Finds the nearest cell in the same subarea using AStar
    Position GetNearestCellAStar(Position sourcePos, State cellState){
        int shortestPathSize = 1000; //Temp value
        list<Position> shortestPath;
        for (int r = 0; r < rows; r++){
            for (int c = 0; c < cols; c++){
                if (grid[r][c] == cellState){
                    if (CompareSubAreas(sourcePos, gridPositions[r][c])){

                        Index startPosIndex = GetNearestCellIndex(sourcePos, Unexplored, Free, false);
                        Index endPosIndex;
                        endPosIndex.x = r;
                        endPosIndex.y = c;

                        //Flip (x,y) to (y,x) as A* uses (y,x)
                        startPosIndex = FlipPos(startPosIndex);
                        endPosIndex = FlipPos(endPosIndex);

                        //Converts to grid to a new grid that A* can use
                        gridtoaStar();

                        list<Index> tempPos; //The positions from A* will be stored in this list (as indexes)
                        tempPos = aStarPATH(newgrid, rows, cols, startPosIndex, endPosIndex, false);

                        //Get the path size
                        int pathSize = 0;
                        for(auto const& p : tempPos){
                            pathSize++;
                        }
                        

                        //Compare the new path size to the one that is currently shortest
                        if(pathSize < shortestPathSize && pathSize != 0){ //If new path is shorter than the previous
                            //First check if the new path does not go outside of the subarea.

                            //Loop all of the new path positions
                            bool pathOutsideSubArea = false;
                            for(auto const&p : tempPos){
                                //If one of the new path is outside the subarea
                                if(CompareSubAreas(sourcePos, gridPositions[int(p.x)][int(p.y)]) == false){
                                    pathOutsideSubArea = true;
                                }
                            }
                            
                            //Use this path, if the it is inside the same subarea
                            if(pathOutsideSubArea == false){
                                shortestPath.clear(); //Clear current shortestPath
                                //Store new path
                                shortestPathSize = pathSize;
                                for (auto const& p : tempPos) {
                                    Position newPos = gridPositions[int(p.x)][int(p.y)];
                                    shortestPath.push_back(newPos);  

                                }    
                            }
                        }       
                    }
                }
            }
        }
        if(shortestPathSize == 2){
            shortestPath.pop_front();
        }

        //Print the path
        cout << "Nearest Cell using AStar: " << endl;
        for (auto const& p : shortestPath) {
            cout << "-> ";
            cout << "(" << p.x << "," << p.y << ")"; 
        }          
        cout << " " << endl;

        //If A* could not find any position, return (-1, -1)
        if(shortestPath.empty()){
            cout << "Nearest cell not found using A*" << endl;
            Position emptyPos;
            emptyPos.x = -1;
            emptyPos.y = -1;
            return emptyPos;
        }
        return shortestPath.back();
    }


    //Finds the nearest cell in another subArea
    AStarPathInfo GetNearestCellAStarAnotherSubArea(Position sourcePos, State cellState, list<Position> otherTurtlebotsPositions, bool prioritizeEmptySubArea){
        //Find out which subareas the other turtlebots are in.
        list<Index> otherTurtlebotsSubareas;
        for(auto const& p : otherTurtlebotsPositions){
            otherTurtlebotsSubareas.push_back(GetSubArea(p));
        }

        cout << "Get nearest cell in another SubArea" << endl;
        int shortestPathSize = 1000; //Temp value
        list<Position> shortestPath;
        for (int r = 0; r < rows; r++){
            for (int c = 0; c < cols; c++){
                if (grid[r][c] == cellState){
                    if (CompareSubAreas(sourcePos, gridPositions[r][c]) == false){ //Check if in another subarea
                        //If prioritize empty subareas
                        bool canCheckThisCell = true;
                        if(prioritizeEmptySubArea == true){
                            for(auto const& p : otherTurtlebotsSubareas){
                                Index i; i.x = r; i.y = c;
                                if(p.x == GetSubAreaIndex(i).x && p.y == GetSubAreaIndex(i).y)
                                    canCheckThisCell = false;
                            }
                        }
                        if(canCheckThisCell){

                            Index startPosIndex = GetNearestCellIndex(sourcePos, Unexplored, Free, false);
                            Index endPosIndex;
                            endPosIndex.x = r;
                            endPosIndex.y = c;

                            //Flip (x,y) to (y,x) as A* uses (y,x)
                            startPosIndex = FlipPos(startPosIndex);
                            endPosIndex = FlipPos(endPosIndex);

                            //Converts to grid to a new grid that A* can use
                            gridtoaStar();

                            list<Index> tempPos; //The positions from A* will be stored in this list (as indexes)
                            tempPos = aStarPATH(newgrid, rows, cols, startPosIndex, endPosIndex, false);

                            //Get the path size
                            int pathSize = 0;
                            for(auto const& p : tempPos){
                                pathSize++;
                            }
                            

                            //Compare the new path size to the one that is currently shortest
                            if(pathSize < shortestPathSize && pathSize != 0){ //If new path is shorter than the previous
                                shortestPath.clear(); //Clear current shortestPath
                                //Store new path
                                shortestPathSize = pathSize;
                                for (auto const& p : tempPos) {
                                    Position newPos = gridPositions[int(p.x)][int(p.y)];
                                    shortestPath.push_back(newPos);  

                                }                           
                            } 
                        }      
                    }
                }
            }
        }
        AStarPathInfo pathInfo;

        //If we could not find any cells (Possibility of there only being subareas left WITH other turtlebots.)
        if(shortestPathSize == 0){
            pathInfo = GetNearestCellAStarAnotherSubArea(sourcePos, cellState, otherTurtlebotsPositions, false);
        }

        //Print the path
        /*cout << "Nearest Cell in another sub area using AStar: " << endl;
        for (auto const& p : shortestPath) {
            cout << "-> ";
            cout << "(" << p.x << "," << p.y << ")"; 
        }          
        cout << " " << endl;*/

        //If A* could not find any position, return (-1, -1)
        if(shortestPath.empty()){
            cout << "Nearest cell outside sub area not found using A*" << endl;
            Position emptyPos;
            emptyPos.x = -1;
            emptyPos.y = -1;
            pathInfo.cellPos = emptyPos;
            return pathInfo;
        }
        //If the A* path exists
        else{
            pathInfo.cellPos = shortestPath.back();
            pathInfo.path = shortestPath;
            return pathInfo;
        }

    }

    list<Position> AStarPathfinding(Position startPos, Position endPos, bool print){

        //GetNearestCellPosition(endPos, Unexplored, false);
        Index endPosIndex = GetNearestCellIndex(endPos, Unexplored, Unexplored, false);
        Index startPosIndex = GetNearestCellIndex(startPos, Unexplored, Free, false); 

        PrintPosition(startPos, "Starting pathfinding from: ");
        PrintPosition(endPos, "to: ");

        //cout << "Start Pos INDEX: (" << startPosIndex.x << "," << startPosIndex.y << ")" << endl;
        //cout << "End Pos INDEX: (" << endPosIndex.x << "," << endPosIndex.y << ")" << endl;

        //Flip (x,y) to (y,x) as A* uses (y,x)
        startPosIndex = FlipPos(startPosIndex);
        endPosIndex = FlipPos(endPosIndex);

        //Converts to grid to a new grid that A* can use
        gridtoaStar();

        list<Index> tempPos; //The positions from A* will be stored in this list (as indexes)
        tempPos = aStarPATH(newgrid, rows, cols, startPosIndex, endPosIndex, false); //was true

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
        if(i == 0){ //First check if there is a path. i = size of path list
            cout << "The A* path is empty" << endl;
        }
        else if(print){
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
    }


    //The only difference between this path planning and the previous, is the endPosIndex also finds the nearest "Free" cell which is needed.
    list<Position> AStarPathfindingToStart(Position startPos, Position endPos, bool print){

        //GetNearestCellPosition(endPos, Unexplored, false);
        Index endPosIndex = GetNearestCellIndex(endPos, Unexplored, Free, false);
        Index startPosIndex = GetNearestCellIndex(startPos, Unexplored, Free, false); 

        PrintPosition(startPos, "Starting pathfinding from: ");
        PrintPosition(endPos, "to: ");

        //cout << "Start Pos INDEX: (" << startPosIndex.x << "," << startPosIndex.y << ")" << endl;
        //cout << "End Pos INDEX: (" << endPosIndex.x << "," << endPosIndex.y << ")" << endl;

        //Flip (x,y) to (y,x) as A* uses (y,x)
        startPosIndex = FlipPos(startPosIndex);
        endPosIndex = FlipPos(endPosIndex);

        //Converts to grid to a new grid that A* can use
        gridtoaStar();

        list<Index> tempPos; //The positions from A* will be stored in this list (as indexes)
        tempPos = aStarPATH(newgrid, rows, cols, startPosIndex, endPosIndex, false); //was true

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
        if(i == 0){ //First check if there is a path. i = size of path list
            cout << "The A* path is empty" << endl;
        }
        else if(print){
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

    }

    //Flip x & y for AStar, as AStar uses (y,x) instead of (x,y)
    Index FlipPos(Index pos){
        //Flip x & y before passing values to A*
        int temp;
        temp = pos.x;
        pos.x = pos.y;
        pos.y = temp;
        return pos;
    }

};