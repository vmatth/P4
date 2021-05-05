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

class Cell{
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

    
};

class SubArea{
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
};

class SuperArea{
private:
    Grid grid;
    vector<SubArea> subAreas;
    double cellDistance; //Distance between each cell

public:
    //Super Area constructor. Creates SubAreas inside. numSubAreas can be 2², 3², 4² ...
    SuperArea(int size, int numSubAreas, double cellSize){
        cout << "Creating new SuperArea with size: " << size << endl;
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
        }
    }

    // Returns the index of which sub area the specific robot is in
    int GetSubArea(Position pos){
        double subAreaSize = subAreas[0].getsubGridSize();
        
        for (int i = 0; i < subAreas.size(); i++)
        {
            double x = subAreas[i].GetPosition().x;
            double y = subAreas[i].GetPosition().y;
            
            if (pos.x > x && pos.x < (x+subAreaSize)){
                if (pos.y > y && pos.y < (y+subAreaSize)){
                    return i;
                }
            }
        }
        ROS_FATAL("No robot in any sub area!!!!");
        return -1;
    }

    //Gets the cell that is closest to the turtlebot
    /*Position GetNearestCell(Position turtlebotPos){
        Cell nearestCell; //Cell class that will be returned
        double shortestDistance = 1000; //Temporary value for distance

        SubArea subArea = subAreas[GetSubArea(turtlebotPos)]; //Get the current subarea that the turtlebot is located in.
        
        //Loop all cells in the subArea
        for(int i = 0; i < subArea.cells.size(); i++){
            //Distance formula
            double distToCell = sqrt(pow(turtlebotPos.x - subArea.cells[i].GetPosition().x, 2) 
                                   + pow(turtlebotPos.y - subArea.cells[i].GetPosition().y, 2));
            //If new cell is closer
            if(distToCell <= shortestDistance){
                shortestDistance = distToCell;
                nearestCell = subArea.cells[i];
            }
        }

        //cout << "Nearest cell has position: (" << nearestCell.GetPosition().x << ", " << nearestCell.GetPosition().y << ")" << endl;

        return nearestCell.GetPosition();
        
    }*/


    int GetNumSubAreas(){
        return subAreas.size();
    }

    int GetNumCells(){
        return subAreas[0].cells.size();
    }

    SubArea& GetSubArea(int index){
        return subAreas[index];
    }

    //Mark Cell with a State
    void MarkCell(Position cellPos, State _state){
        //Find which subArea the cell is in
        int subAreaIndex = GetSubArea(cellPos);
        //cout << "Marking cell in SubArea: " << subAreaIndex << endl;
        //Find the id of the cell (index in the "cells" array)
        int cellIndex = subAreas[subAreaIndex].GetCellIndex(cellPos);
        cout << "Marking cell (" << cellPos.x << " , " << cellPos.y << ")" << " [cellId: " << cellIndex << "], State: " << _state << endl;
        //Update the state
        subAreas[subAreaIndex].cells[cellIndex].UpdateState(_state);
    }
    
    //Checks if the new position "cellPos" is an existing point, and it is in the same subArea as the turtlebot
    bool CheckIfCellExists(Position cellPos, Position turtlebotPos){
        //cout << "Checking if Cell (" << cellPos.x << ", " << cellPos.y << ") exists." << endl;
         //Find which subArea the cell is in
        int subAreaIndex = GetSubArea(cellPos);
        int turtlebotSubAreaIndex = GetSubArea(turtlebotPos);
        //cout << "The cell is located in subArea: " << subAreaIndex << endl;
        if(subAreaIndex == -1)
        {
            cout << "The cell is not in any subareas" << endl;
            return false;
        }

        if(subAreaIndex != turtlebotSubAreaIndex){
            cout << "The cell is not in the same subArea as the turtlebot" << endl;
            return false;
        }

        int cellIndex = subAreas[subAreaIndex].GetCellIndex(cellPos);

        if(cellIndex == -1)
        {
            cout << "The cell cannot be found" << endl;
            return false;
        }
        else
        {
            return true;
        }

    }

    int GetCellState(Position cellPos){
        //cout << "Get Cell State for (" << cellPos.x << "," << cellPos.y << ")" << endl;
        //Find which subArea the cell is in
        int subAreaIndex = GetSubArea(cellPos);
        //Find the id of the cell (index in the "cells" array)
        int cellIndex = subAreas[subAreaIndex].GetCellIndex(cellPos);
        //Get the cell state
        //cout << "STATE: " << subAreas[subAreaIndex].cells[cellIndex].GetState() << endl; //Get State
        return subAreas[subAreaIndex].cells[cellIndex].GetState(); //Get State
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
        //PrintPosition(wallPos, "Checking if pos is near a wall");
        //Find the subArea that the turtlebot is in.
        int subAreaIndex = GetSubArea(robotPos);
        Position relativeCell; 
        for(int j = 0; j < subAreas.size(); j++){
            for(int i = 0; i < subAreas[j].cells.size(); i++)
            {
                Position cellPos = subAreas[subAreaIndex].cells[i].GetPosition();
                relativeCell.x = cellPos.x - wallPos.x;
                relativeCell.y = cellPos.y - wallPos.y;
                if(abs(relativeCell.x) < 0.2 && abs(relativeCell.y) < 0.2)
                {
                    //PrintPosition(cellPos, "Updating cell to wall");
                    subAreas[subAreaIndex].cells[i].UpdateState(Wall);

                    //Calculate the marker Id for rviz
                    CellInfo cellInfo;
                    cellInfo.id = subAreaIndex * subAreas[subAreaIndex].cells.size() + i + 1;
                    //cout << "The marker id is: " << cellInfo.id;

                    cellInfo.pos = cellPos;

                    return cellInfo;
                }
            }
        }
    }

    void PrintPosition(Position pos, string text){
        cout << text << "(" << pos.x << ", " << pos.y << ")" << endl;
    }

    void PrintAllCells(int subAreaIndex){
        for(int i = 0; i < subAreas[subAreaIndex].cells.size(); i++){
            PrintPosition(subAreas[subAreaIndex].cells[i].GetPosition(), "");
        }
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
   /* Position GetNearestUnexploredCell(Position turtlebotPos){
        Cell nearestCell; //Cell class that will be returned
        double shortestDistance = 1000; //Temporary value for distance

        SubArea subArea = subAreas[GetSubArea(turtlebotPos)]; //Get the current subarea that the turtlebot is located in.
        
        //Loop all cells in the subArea
        for(int i = 0; i < subArea.cells.size(); i++){
            //Distance formula
            double distToCell = sqrt(pow(turtlebotPos.x - subArea.cells[i].GetPosition().x, 2) 
                                   + pow(turtlebotPos.y - subArea.cells[i].GetPosition().y, 2));
            //If new cell is closer
            if(distToCell <= shortestDistance && subArea.cells[i].GetState() == Unexplored ){
                shortestDistance = distToCell;
                nearestCell = subArea.cells[i];
            }
        }

        //cout << "Nearest cell has position: (" << nearestCell.GetPosition().x << ", " << nearestCell.GetPosition().y << ")" << endl;

        return nearestCell.GetPosition();
    }*/

    //Gets the cell that is closest to the turtlebot
    Position GetNearestCell(Position turtlebotPos, State CellState){
        Cell nearestCell; //Cell class that will be returned
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
};