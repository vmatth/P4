#pragma once

#include <iostream>

#include <vector>
#include <cmath>
#include <states.h>

#include <position.h>
#include <index.h>
#include <string>

#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

#include <markers.h>

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


    //Grid contains 0 and 1. 0: Wall, 1: Free
    int grid[][0];

    //Grid contains information of which subarea the cell is in
    //The grid will look something like:
    // 2 2 3 3
    // 2 2 3 3
    // 0 0 1 1
    // 0 0 1 1
    Index gridSubAreas[][0];

    //Add a position (x,y) for each grid
    Position gridPositions[][0];

    int subAreaSize;
    int numSubAreas;
    int numSubAreasSqrt;

    int rows, cols;
public:
    //Super Area constructor. Creates SubAreas inside. numSubAreas can be 2², 3², 4² ...
    SuperArea(int size, int _numSubAreas, double cellSize){

        numSubAreas = _numSubAreas;

        NewGrid(size, _numSubAreas, cellSize);

        /*cout << "Creating new SuperArea with size: " << size << endl;
        grid.size = size;
        cellDistance = cellSize;

        double subAreaSize = grid.size / sqrt(numSubAreas);
        cout << "Sub Area Size:  " << subAreaSize << endl;

        for(int y = 0; y < sqrt(numSubAreas); y++){
            for(int x = 0; x < sqrt(numSubAreas); x++){
                //cout << "(x: " << x << ", y: " << y << ")";

                Position newPos;
                newPos.x = x * subAreaSize;
                newPos.y = y * subAreaSize;

                Index newIndex;
                newIndex.x = x;
                newIndex.y = y;

                SubArea subArea(subAreaSize, newPos, cellSize, newIndex);

                subAreas.push_back(subArea);

                //cout << " NewPos: (" << newPos.x << ", " << newPos.y << ")" << endl;
            }
        }*/
    }

    // Returns the index of which sub area the specific robot is in E.g [0,1] (x,y)
    Index GetSubArea(Position pos){
        Index subA; //Index value to return later

        for(int x = 0; x < numSubAreasSqrt; x++){
            for(int y = 0; y < numSubAreasSqrt; y++){
                if(pos.x >= subAreaIntervalsPosition[x][y].startX && pos.x < subAreaIntervalsPosition[x][y].endX){
                    if(pos.y >= subAreaIntervalsPosition[x][y].startY && pos.y < subAreaIntervalsPosition[x][y].endY){
                        cout << "GetSubArea: [" << x << "][" << y << "] for pos (" << pos.x << "," << pos.y << ")" << endl;
                        Index subA;
                        subA.x = x;
                        subA.y = y;
                        return subA;
                    } 
                }
            }
        }

        cout << "SubArea not found for pos (" << pos.x << "," << pos.y << ")" << endl;
        subA.x = -1;
        subA.y = -1;       
        return subA;
    }

    //Gets a position, and finds out the corresponding Cell Index E.g Pos(5, 0.4) has a Index of [8, 2];
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
                    cout << "GetCellIndex: [" << x << "][" << y << "] for pos (" << pos.x << "," << pos.y << ")" << endl;;
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

    /*SubArea& GetSubArea(int index){
        return subAreas[index];
    }*/

    //Mark Cell with a State
    void ChangeCellState(Position cellPos, State _state){

        Index cellIndex = GetCellIndex(cellPos);        

        if(cellIndex.x != -1){ //Check if the cell exists
            cout << "Marking cell (" << cellPos.x << " , " << cellPos.y << "), State: " << _state << endl;
        }

        grid[cellIndex.x][cellIndex.y] = _state;
    }

    bool CompareSubAreas(Position pos1, Position pos2){

        Index subArea1 = GetSubArea(pos1);
        Index subArea2 = GetSubArea(pos2);

        if(subArea1.x == subArea2.x && subArea1.y == subArea2.y){
            return true;
        }

        return false;
    }
    
    //Checks if the new position "cellPos" is an existing point, and it is in the same subArea as the turtlebot
    bool CheckIfCellExists(Position cellPos, Position turtlebotPos){

        Index cellIndex = GetCellIndex(cellPos);
        if(cellIndex.x != -1){
            return true;
        }
        else{
            return false;
        }
    }

    //Returns the cell's state E.g Unexplored, Wall or Free
    int GetCellState(Position cellPos){

        Index cellIndex = GetCellIndex(cellPos);        

        if(cellIndex.x != -1){ //Check if the cell exists
            return grid[cellIndex.x][cellIndex.y];
        }      
    }

    int GetCellState(Index cellIndex){
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

        //Fix yaw if over 180

        /*if(yaw > 180 && yaw <= 360){
            yaw = 180 - yaw;
        }*/

        cout << "Robot yaw: " << yaw << endl;

        
        //Check if these cells exist.

        //Finds the angle difference from the robot's rotation to each point (up down left right) 
        //Is used for finding the closest point from the robot.
        double upAngle = 360;
        double downAngle = 360;
        double leftAngle = 360;
        double rightAngle = 360; //Default 360 

        cout << "Up" << endl;
        if(CheckIfCellExists(upPos, currentPosition)){
            upAngle = abs(90 - yaw);
            cout << "Up angle: " << upAngle << endl;
        }

        cout << "Down" << endl;
        if (CheckIfCellExists(downPos, currentPosition)){
            downAngle = abs(270 - yaw);
            cout << "Down Angle: " << downAngle << endl;
        }

        PrintPosition(leftPos, "LeftPos");
        cout << "Left" << endl;
        if (CheckIfCellExists(leftPos, currentPosition)){
            leftAngle = abs(180 - yaw);
            cout << "Left Angle: " << leftAngle << endl;
        }

        cout << "Right" << endl;
        if (CheckIfCellExists(rightPos, currentPosition)){
            //quik mafs
            if(yaw > 180){
                rightAngle = abs(360 - yaw);
            }
            else{
                rightAngle = abs(0 - yaw);
            }
            cout << "Right Angle: " << rightAngle << endl;
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
            cout << "Up is front" << endl;
            front.x = upPos.x;
            front.y = upPos.y;

            right.x = rightPos.x;
            right.y = rightPos.y;

            left.x = leftPos.x;
            left.y = leftPos.y;

            back.x = downPos.x;
            back.y = downPos.y;

        } else if (index == 1){
            cout << "Down is front" << endl;
            front.x = downPos.x;
            front.y = downPos.y;

            right.x = leftPos.x;
            right.y = leftPos.y;

            left.x = rightPos.x;
            left.y = rightPos.y;

            back.x = upPos.x;
            back.y = upPos.y;

        } else if (index == 2){
            cout << "Right is front" << endl;
            front.x = rightPos.x;
            front.y = rightPos.y;

            right.x = downPos.x;
            right.y = downPos.y;

            left.x = upPos.x;
            left.y = upPos.y;

            back.x = leftPos.x;
            back.y = leftPos.y;

        } else if (index == 3){
            cout << "Left is front" << endl;
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

        if(GetCellState(front) == Unexplored && GetSubArea(front) == GetSubArea(currentPosition)){
            cout << "Front cell is unexplored!" << endl;
            return front;
        }
        else if(GetCellState(right) == Unexplored && GetSubArea(right) == GetSubArea(currentPosition)){
            cout << "Right cell is unexplored!" << endl; 
            return right;
        }
        else if(GetCellState(left) == Unexplored && GetSubArea(left) == GetSubArea(currentPosition)){
            cout << "Left cell is unexplored!" << endl; 
            return left;
        }
        else if(GetCellState(back) == Unexplored && GetSubArea(back) == GetSubArea(currentPosition)){
            cout << "Back cell is unexplored!" << endl;
            return back;
        } else {
            cout << "Find nearest unexplored cell" << endl;
            return GetNearestCell(currentPosition, Unexplored);
        }




    }


    
    //Checks if the newly marked point is close to any of the cells. If true, then it will nark the cell as "Wall"
    //Returns info on the cell that has been updated.
    CellInfo MarkWallCells(Position wallPos, Position robotPos){

        CellInfo cellInfo; //Returns info on the cell that has been updated. Used for rviz marker    

        Index cellIndex = GetCellIndex(wallPos);

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
                    //Update state to wall
                    grid[x][y] = Wall;

                    
                    cellInfo.id = x + y;
                    cellInfo.pos.x = gridPositions[x][y].x;
                    cellInfo.pos.y = gridPositions[x][y].y;

                    return cellInfo;
                }
            }
        }

        
        cellInfo.id = -1;
        return cellInfo;
    }

    void PrintPosition(Position pos, string text){
        cout << text << "(" << pos.x << ", " << pos.y << ")" << endl;
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

    //Gets the cell that is closest to the turtlebot
    Position GetNearestCell(Position turtlebotPos, State CellState){

        Index nearestCell; //Cell index that will be returned

        //Cell nearestCell; //Cell class that will be returned
        double shortestDistance = 1000; //Temporary value for distance

        SubArea subArea = subAreas[GetSubArea(turtlebotPos)]; //Get the current subarea that the turtlebot is located in.
        
        //Loop all cells in the subArea
        for(int i = 0; i < subArea.cells.size(); i++){
            //Distance formula
            double distToCell = sqrt(pow(turtlebotPos.x - subArea.cells[i].GetPosition().x, 2) 
                                   + pow(turtlebotPos.y - subArea.cells[i].GetPosition().y, 2));
            //If new cell is closer
            if(distToCell <= shortestDistance && subArea.cells[i].GetState() == CellState ){
                shortestDistance = distToCell;
                nearestCell = subArea.cells[i];
            }
        }

        //cout << "Nearest cell has position: (" << nearestCell.GetPosition().x << ", " << nearestCell.GetPosition().y << ")" << endl;

        return nearestCell.GetPosition();
    }




    //Creates a new grid. All sizes are in metres
    void NewGrid(int size, int _numSubAreas, double cellDistance){
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

        grid[rows][cols];
        gridSubAreas[rows][cols];
        gridPositions[rows][cols];


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


    }

    int CheckFreeArea(){
        for(int i = 0; i < numRobots; i++){
            for(int i = 0; i < GetNumSubAreas(); i++){
                if(robot position is not in subarea){
                    return i; 
                }
            }
        }
    }


};