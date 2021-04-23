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


using namespace std;

struct Grid{
    Position pos; //Starting position for the grid
    double size; //x & y size of the grid
};

class Cell{
private:
    Grid grid;
    State state;
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
    SubArea(int size, Position pos, double cellSize, Index _index){
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
                //cout << "(x: " << x << ", y: " << y << ")";

                Position newPos;
                newPos.x = (x * cellSize) + (cellSize/2) + grid.pos.x;
                newPos.y = (y * cellSize) + (cellSize/2) + grid.pos.y;

                Cell cell(cellSize, newPos);

                cells.push_back(cell);

                //cout << " NewPos: (" << newPos.x << ", " << newPos.y << ")" << endl;
            }
        }
    }

    // Returns start position of specific sub area
    Position GetPosition(){
        return grid.pos;
    }
};

class SuperArea{
private:
    Grid grid;
    vector<SubArea> subAreas;

public:
    //Super Area constructor. Creates SubAreas inside
    SuperArea(int size, int numSubAreas, double cellSize){
        cout << "Creating new SuperArea with size: " << size << endl;
        grid.size = size;

        double subAreaSize = grid.size / sqrt(numSubAreas);
       // cout << "SubAreaSize: " << subAreaSize << endl;
        //numSubAreas can be 2², 3², 4² ...

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
    int GetTurtlebotSubArea(Position turtlebotPos){
        double subAreaSize = subAreas[0].getsubGridSize();
        
        for (int i = 0; i < subAreas.size(); i++)
        {
            double x = subAreas[i].GetPosition().x;
            double y = subAreas[i].GetPosition().y;
            
            if (turtlebotPos.x > x && turtlebotPos.x < (x+subAreaSize)){
                if (turtlebotPos.y > y && turtlebotPos.y < (y+subAreaSize)){
                    return i;
                }
            }
        }
        ROS_FATAL("No robot in any sub area!!!!");
    }

    //Gets the cell that is closest to the turtlebot
    Position GetNearestCell(Position turtlebotPos){
        Cell nearestCell; //Cell class that will be returned
        double shortestDistance = 1000; //Temporary value for distance

        SubArea subArea = subAreas[GetTurtlebotSubArea(turtlebotPos)]; //Get the current subarea that the turtlebot is located in.
        
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
        
    }

    
};




//// Go to cell
// brug move to goal function (fra P1)







