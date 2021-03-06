#include <ros/ros.h>

#include <turtlebot.h>

#include <grid.h>

#include <markers.h>
#include <unistd.h>

using namespace std;
using namespace ros;

//Variables and functions for initializing the turtlebot swarm
namespace TurtlebotManager{

    int numRobots = 3;

    vector<Position> robotStartPositions;

    vector<std::unique_ptr<Turtlebot>> turtlebots;

    void InitializeTurtlebots(){ //Initializes a specified amount of turtlebots for the swarm

        //Specify the robot start position
        Position pos0; pos0.x = 1.75; pos0.y = 1.75;
        Position pos1; pos1.x = 3; pos1.y = 1.75;
        Position pos2; pos2.x = 1.75; pos2.y = 3;

        robotStartPositions.push_back(pos0);
        robotStartPositions.push_back(pos1);
        robotStartPositions.push_back(pos2);

        //Loop number of wanted turtlebots, and initialize a new turtlebot class using unique_ptr. 
        //The turtlebots are stored in the "turtlebots" vector.
        for(int i = 0; i < numRobots; i++){
            unique_ptr<Turtlebot> turtlebot(new Turtlebot(i, robotStartPositions[i])); //Create new turtlebot class with id as argument.
            turtlebots.push_back(move(turtlebot));
        }

        ROS_INFO("Turtlebots initialized!");
    }

    void MoveTurtlebots(){
        for(int i = 0; i < numRobots; i++){
            turtlebots[i]->Move();
        }
    }

    //Gets the positions of other robots except this one
    list<Position> GetOtherTurtlebotsPosition(int turtlebotId){
        list<Position> otherPos;
        for(int i = 0; i < numRobots; i++){
            if (i != turtlebotId){
                otherPos.push_back(turtlebots[i]->GetPosition());
            }
        }
        return otherPos;
    }

    list<Position> GetOtherTurtlebotsGoalPosition(int turtlebotId){
        list<Position> otherPos;
        for(int i = 0; i < numRobots; i++){
            if (i != turtlebotId){
                if(turtlebots[i]->GetMovementsSize() != 0){
                    otherPos.push_back(turtlebots[i]->GetMovements().back());
                }
            }
        }
        return otherPos;
    }
}

struct AvoidingInfo{ //Struct used for the avoid algorithm.
    Position cellPos; //The cell that will be marked as tempWall
    int masterId; //The other robot that is the master (Lower Id robot)
    bool avoiding = false; //If avoiding or not.
    Index upIndex, downIndex, leftIndex, rightIndex, upLeftIndex, upRightIndex, downLeftIndex, downRightIndex;
    int upState, downState, leftState, rightState, upLeftState, upRightState, downLeftState, downRightState;
};

namespace MarkersManager{
    Markers markers;
    //(Super Area size / cellDistance) must be divideable by numSubAreas.
    //Super are size must be even
    //float cellSpace = 1.0f/3.0f;

    float cellSpace = 0.5;
    //SuperArea superArea(10, 4, cellSpace); //Smallbox

    //SuperArea superArea(6, 1, cellSpace); //miniBox

    //SuperArea superArea(8, 1, cellSpace); //CurveOffice

    SuperArea superArea(16, 16, cellSpace); //Newoffice


    AvoidingInfo avoidingInfo[3];//Array of all tempWall cells that will be reverted.

    void DrawAllCells(){
        usleep(400000);
        //Create marker for each cell

        //Draw Lines in grid
        for(int r = 0; r < superArea.GetNumSubAreasSqrt(); r++){
            for(int c = 0; c < superArea.GetNumSubAreasSqrt(); c++){
                SubAreaInterval a = superArea.GetSubAreaInterval(r, c);
                //Vertical lines
                Position startLinePos; startLinePos.x = a.startX; startLinePos.y = a.startY;
                Position endLinePos; endLinePos.x = a.startX; endLinePos.y = a.endY;
                markers.MLine(startLinePos, endLinePos, -1);
                //Horizontal lines
                startLinePos; startLinePos.x = a.startX; startLinePos.y = a.startY;
                endLinePos; endLinePos.x = a.endX; endLinePos.y = a.startY;
                markers.MLine(startLinePos, endLinePos, -1);
            }            
        }

        //Draw top line
        SubAreaInterval a = superArea.GetSubAreaInterval(superArea.GetNumSubAreasSqrt()-1, superArea.GetNumSubAreasSqrt()-1);
        Position startLinePos; startLinePos.x = 0; startLinePos.y = a.endY;
        Position endLinePos; endLinePos.x = a.endX; endLinePos.y = a.endY;
        markers.MLine(startLinePos, endLinePos, -1);
        //Draw right line
        a = superArea.GetSubAreaInterval(superArea.GetNumSubAreasSqrt()-1, superArea.GetNumSubAreasSqrt()-1);
        startLinePos.x = a.endX; startLinePos.y = 0;
        endLinePos.x = a.endX; endLinePos.y = a.endY;
        markers.MLine(startLinePos, endLinePos, -1);

        //Loops all the cells and creates a marker for them
        for(int c = 0; c < superArea.GetRows(); c++){
            for(int r = 0; r < superArea.GetRows(); r++){
                Index i;
                i.x = r;
                i.y = c;
                //Gets the cell's position from the cell's index
                Position cellPos = superArea.GetCellPosition(i);
                Index subAreaIndex = superArea.GetSubAreaIndex(i);
                markers.CellMarker(cellPos, Unexplored, subAreaIndex, superArea.GetNumSubAreasSqrt());
                usleep(3000); //Small delay (in microseconds) to give rViz time.
            }
        }


        
       
        

    }

    int stuckTimer[3]; //Counted using 10 hz loop. So if stuckTimer[0] = 10 means turtlebot 0 has been stuck for 1 second.
    //Used in FreeRobots() function
        
    void InitializeMarkers(){

        //Setups each different topic
        markers.SetupMarker(); 
        markers.SetupRobotMarker();
        markers.SetupCellMarker();
        markers.SetupMlineMarker();

        ROS_INFO("Markers initialized");   

        DrawAllCells();

        stuckTimer[0] = 0; 
        stuckTimer[1] = 0; 
        stuckTimer[2] = 0; 
    }


    void DrawPoints(){
        for(int i = 0; i < TurtlebotManager::numRobots; i++){
            if(TurtlebotManager::turtlebots[i]->GetPoint().x != 0){ //Check if the point exists
                usleep(100);//Wait 1 ms so rviz can follow
                markers.NewMarker(TurtlebotManager::turtlebots[i]->GetPoint(), TurtlebotManager::turtlebots[i]->GetId());
            }
        }
    }

    void DrawRobotMarkers()
    {
        for(int i = 0; i < TurtlebotManager::numRobots; i++)
        {
            usleep(100); //Wait 1 ms so rviz can follow
            markers.RobotMarker(TurtlebotManager::turtlebots[i]->GetPosition(), TurtlebotManager::turtlebots[i]->GetId());    
        }
    }

    void DrawMLine(int ID, Position goalPos)
    {
        //Position startPos
        //startPos.x = 1; startPos.y = 1;
        markers.MLine(TurtlebotManager::turtlebots[ID]->GetPosition(), goalPos, ID); //Remember to change start and goal pos
        cout << "Drawing MLine" << endl;
        //cout << "StartPos: (" << TurtlebotManager::turtlebots[ID]->GetPosition().x << "," <<TurtlebotManager::turtlebots[ID]->GetPosition().y << ")"<< endl;
        //cout << "goalPos: (" << goalPos.x << "," << goalPos.y << ")"<< endl;
    }

    //Finds a new path using A* and moves the robot to it.
    bool StartAStarPathfinding(int turtlebotId){
        if(TurtlebotManager::turtlebots[turtlebotId]->finished == true)
            return false;
        if(TurtlebotManager::turtlebots[turtlebotId]->GetAvoidingSlave() == true){
            cout << turtlebotId << " tried to AStar while slave." << endl;
            return false;
        }
        //Stop the current movement
        TurtlebotManager::turtlebots[turtlebotId]->EmptyList();
        //Find a new point using A*
        Position turtlebotPos = TurtlebotManager::turtlebots[turtlebotId]->GetPosition();
        Position goalPos;
        
        Index robotCell = superArea.GetCellFromPosition(turtlebotPos); //Find the cell that the robot is in

        bool freeCell = false;

        //If the robot is inside a wall cell (can happen if the robot detects the wall late.), then A* to nearest Free
        if(superArea.GetCellState(robotCell) == Wall || superArea.GetCellState(robotCell) == TempWall){
            //Go back to prev pos
            goalPos = superArea.GetNearestCellAStar(turtlebotPos, Free); 
            superArea.PrintPosition(goalPos, "GOAL POS: ");
            if(goalPos.x != -1){
                TurtlebotManager::turtlebots[turtlebotId]->NewMovement(traverse, goalPos);
                cout << "goal pos exists" << endl;
            }
            else{
                TurtlebotManager::turtlebots[turtlebotId]->NewMovement(traverse, TurtlebotManager::turtlebots[turtlebotId]->GetPrevPosition());
                cout << "to prev pos" << endl;
            }
            cout << "A* to free." << endl;
            return true;
            //TurtlebotManager::turtlebots[0]->PrintPosition(goalPos, "Free area goalpos: ");
        }
        else{
            //cout << "A* to unexplored." << endl;
            goalPos = superArea.GetNearestCellAStar(turtlebotPos, Unexplored); 
            //Do not use this goalPos, if it is in the same subarea of another turtlebot, or in the same subarea that another robot is navigating to
            /*for(auto const& p : TurtlebotManager::GetOtherTurtlebotsPosition(turtlebotId)){
                if(superArea.GetSubArea(p).x == superArea.GetSubArea(goalPos).x && superArea.GetSubArea(p).y == superArea.GetSubArea(goalPos).y){
                    goalPos.x = -1;
                }
            }*/
            for(auto const& p : TurtlebotManager::GetOtherTurtlebotsGoalPosition(turtlebotId)){
                if(superArea.GetSubArea(p).x == superArea.GetSubArea(goalPos).x && superArea.GetSubArea(p).y == superArea.GetSubArea(goalPos).y){
                    goalPos.x = -1;
                }
            }
        }

        bool pathFound = false; //Return value
        
        if(goalPos.x != -1){ //If goalPos was using A*
            list<Position> path = superArea.AStarPathfinding(turtlebotPos, goalPos, false); //Set to true to show path in terminal
            //Set pathfinding variables
            TurtlebotManager::turtlebots[turtlebotId]->SetPathfinding(true);
            TurtlebotManager::turtlebots[turtlebotId]->SetPathfindingPoint(goalPos);
            //Give the turtlebot movements
            for (auto const& p : path) {
                TurtlebotManager::turtlebots[turtlebotId]->NewMovement(traverse, p); 
            }
            pathFound = true;
        }
        else{ //If no path can be found in the turtlebots subarea, find next subarea
            cout << "No movements for turtlebot " << turtlebotId << " in this subarea. New subarea will be found " << endl;
            AStarPathInfo pathInfo;
            pathInfo = superArea.GetNearestCellAStarAnotherSubArea(turtlebotPos, Unexplored, TurtlebotManager::GetOtherTurtlebotsPosition(turtlebotId), TurtlebotManager::GetOtherTurtlebotsGoalPosition(turtlebotId), true);

            if(pathInfo.cellPos.x != -1) { //Check if A* could find a cell in another subarea
                //Set pathfinding variables
                TurtlebotManager::turtlebots[turtlebotId]->SetPathfinding(true);
                TurtlebotManager::turtlebots[turtlebotId]->SetPathfindingPoint(pathInfo.cellPos);
                //Give the turtlebot movements
                for (auto const& p : pathInfo.path) {
                    TurtlebotManager::turtlebots[turtlebotId]->NewMovement(traverse, p); 
                }
                Index newSubArea = superArea.GetSubArea(pathInfo.cellPos);
                //cout << "New Sub Area: ";
                //cout << "(" << newSubArea.x << " , " << newSubArea.y << ")" << endl;
                pathFound = true;
            }
            else{ //No valid points could be found in any other subarea. The robot will then return to start position
                //Only traverse to start pos once.
                //if(TurtlebotManager::turtlebots[turtlebotId]->GetPathfindingStartPos() == false){
                    TurtlebotManager::turtlebots[turtlebotId]->SetPathfindingStartPos(true); //Set this variable to true, so it only happens once!                                                              
                    cout << "[" << turtlebotId << "] Cannot find new subareas to explore. Will return to start position" << endl;
                    goalPos = TurtlebotManager::turtlebots[turtlebotId]->GetStartPos();
                    list<Position> path = superArea.AStarPathfindingToStart(turtlebotPos, goalPos, false);

                    if(path.empty()){ //If A* to goal was not found, return false
                        cout << "Could not find path to goal." << endl;
                        return false;
                    }

                    //Set pathfinding variables
                    TurtlebotManager::turtlebots[turtlebotId]->SetPathfinding(true);
                    TurtlebotManager::turtlebots[turtlebotId]->SetPathfindingPoint(pathInfo.cellPos);
                    TurtlebotManager::turtlebots[turtlebotId]->SetForcePathfind(true);
                    //Give the turtlebot movements
                    for (auto const& p : path) {
                        TurtlebotManager::turtlebots[turtlebotId]->NewMovement(traverse, p); 
                    }
                    TurtlebotManager::turtlebots[turtlebotId]->SetPathfindingPoint(goalPos);
                    cout << "-------------" << endl;
                    pathFound = true;
                //}
            }
        }

        return pathFound;
    }

    //When the robot finds a new wall, checks if the wall is the next goal when using PSO. If true, use A*
    void UpdatePSOPathfinding(Position wallPos, int turtlebotId){
        //cout << "Update PSO Pathfinding!" << endl;
        if(TurtlebotManager::turtlebots[turtlebotId]->GetPathfinding() == false){
            //cout << "Using PSO!" << endl;
            //Check if the robot will collide with its goal pos
            bool collision = superArea.CheckForCollision(wallPos, TurtlebotManager::turtlebots[turtlebotId]->GetGoalPos());
            if(collision){ //if collide
                //cout << "Robot [" << turtlebotId << "]'s goal position will collide with a wall. Use A*" << endl; 
                StartAStarPathfinding(turtlebotId);
            }
        }          
    }

    //When the robot finds a new wall, update the pathfinding to check if the wall is in the path.
    void UpdateAStarPathfinding(Position wallPos, int turtlebotId){
        if(TurtlebotManager::turtlebots[turtlebotId]->GetPathfinding() == true){ //If the robot is pathfinding
            //Check if the wall is a part of the path
            //cout << "New Wall found, check if its a part of PATHS" << endl;
            //cout << "New wall found by [" << turtlebotId << "] when traversing with A*. Updating A* path to see if the wall is in the way." << endl;
            bool inPath = false;

            //Checks if the robot's current path will collide with the newly found wall
            list<Position> turtlebotPath = TurtlebotManager::turtlebots[turtlebotId]->GetMovements();
            for (auto const& p : turtlebotPath) {
                if(superArea.ComparePositions(p, wallPos, 0.1)){
                    inPath = true;
                }
            }            
            //If the wall is a part of the PATH, update the robot's path.
            if(inPath){
                //If the robot is pathfinding to goal, then find a new path to goal
                if(TurtlebotManager::turtlebots[turtlebotId]->GetPathfindingStartPos() == true){                                                           
                    cout << "[" << turtlebotId << "] Found new wall on way to start pos. Will update path" << endl;
                    Position goalPos = TurtlebotManager::turtlebots[turtlebotId]->GetStartPos();
                    list<Position> path = superArea.AStarPathfindingToStart(TurtlebotManager::turtlebots[turtlebotId]->GetPosition(), goalPos, false);

                    if(path.empty()){ //If A* to goal was not found, return false
                        TurtlebotManager::turtlebots[turtlebotId]->EmptyList();
                    }

                    //Set pathfinding variables
                    TurtlebotManager::turtlebots[turtlebotId]->SetPathfinding(true);
                    TurtlebotManager::turtlebots[turtlebotId]->SetForcePathfind(true);
                    //Give the turtlebot movements
                    for (auto const& p : path) {
                        TurtlebotManager::turtlebots[turtlebotId]->NewMovement(traverse, p); 
                    }
                    TurtlebotManager::turtlebots[turtlebotId]->SetPathfindingPoint(goalPos);
                }
                if(TurtlebotManager::turtlebots[turtlebotId]->GetForcePathfind() == false) //If not forcing the goalPos, find the nearest cell and pathfind to it
                    StartAStarPathfinding(turtlebotId);
                else{ //If forcing the turtlebot to move to a specific point (E.g at the start when spreading out)
                    //cout << "Forcing pathfind !!!" << endl;
                    TurtlebotManager::turtlebots[turtlebotId]->EmptyList();
                    //Find a new point using A*
                    Position turtlebotPos = TurtlebotManager::turtlebots[turtlebotId]->GetPosition();
                    Position goalPos = TurtlebotManager::turtlebots[turtlebotId]->GetPathfindingPoint();
                
                    //If the robot hasn't reached its goal yet
                    if(superArea.ComparePositions(turtlebotPos, goalPos, cellSpace) == false){
                        list<Position> path = superArea.AStarPathfinding(turtlebotPos, goalPos, false);
                        //Give the turtlebot movements
                        for (auto const& p : path) {
                            TurtlebotManager::turtlebots[turtlebotId]->NewMovement(traverse, p); 
                        }
                    }
                }
            }
        }
    }

    //Marks the wall point in rViz, and updates the cell state to wall.
    void MarkWallPoint(Position wallPos, int turtlebotId){
        //Checks if the point is a wall.
        CellInfo cellInfo = superArea.NewWallPoint(wallPos, MarkersManager::cellSpace); //Changes the state of the cell to Wall if its a wall
        //Check if the wall has been found
        if(cellInfo.id != -1){
            //Gets the cell's position from the cell's index
            Index cellIndex = superArea.GetCellIndex(cellInfo.pos);
            Index subAreaIndex = superArea.GetSubAreaIndex(cellIndex);
            markers.CellMarkerUpdate(cellInfo.id, Wall, cellInfo.pos, subAreaIndex, superArea.GetNumSubAreasSqrt()); //Updates the point in rViz
        }
//       markers.CellMarkerUpdate(cellInfo.id, Wall, cellInfo.pos, subAreaIndex, superArea.GetNumSubAreasSqrt()); //Updates the point in rViz
        UpdatePSOPathfinding(cellInfo.pos, turtlebotId); //Updates path if using PSO
        UpdateAStarPathfinding(cellInfo.pos, turtlebotId); //Update path if using A*
        TurtlebotManager::turtlebots[turtlebotId]->EmptyNewPoint();
    }
   
    void StopAvoiding(int i){    
        avoidingInfo[i].avoiding = false;
        
        //ugly code. sorry
        #pragma region badcode
        
        //superArea.ChangeCellState(avoidingInfo[i].cellPos, Free); //Change the cell state to free
        superArea.ChangeCellState(avoidingInfo[i].upIndex, (State)avoidingInfo[i].upState);
        superArea.ChangeCellState(avoidingInfo[i].downIndex, (State)avoidingInfo[i].downState);
        superArea.ChangeCellState(avoidingInfo[i].leftIndex, (State)avoidingInfo[i].leftState);
        superArea.ChangeCellState(avoidingInfo[i].rightIndex, (State)avoidingInfo[i].rightState);
        superArea.ChangeCellState(avoidingInfo[i].upRightIndex, (State)avoidingInfo[i].upRightState);
        superArea.ChangeCellState(avoidingInfo[i].upLeftIndex, (State)avoidingInfo[i].upLeftState);
        superArea.ChangeCellState(avoidingInfo[i].downRightIndex, (State)avoidingInfo[i].downRightState);
        superArea.ChangeCellState(avoidingInfo[i].downLeftIndex, (State)avoidingInfo[i].downLeftState);
        
        //UP
        int cellId = superArea.GetCellId(avoidingInfo[i].upIndex);
        if(cellId != -1){ //If the cell has a valid Id
            Index subAreaIndex = superArea.GetSubAreaIndex(avoidingInfo[i].upIndex);
            //Draw in Rviz
            markers.CellMarkerUpdate(cellId, (State)avoidingInfo[i].upState, superArea.GetCellPosition(avoidingInfo[i].upIndex), subAreaIndex, superArea.GetNumSubAreasSqrt());
        }
            usleep(200);
        // //DOWN
        cellId = superArea.GetCellId(avoidingInfo[i].downIndex);
        if(cellId != -1){ //If the cell has a valid Id
            Index subAreaIndex = superArea.GetSubAreaIndex(avoidingInfo[i].downIndex);
            //Draw in Rviz
            markers.CellMarkerUpdate(cellId, (State)avoidingInfo[i].downState, superArea.GetCellPosition(avoidingInfo[i].downIndex), subAreaIndex, superArea.GetNumSubAreasSqrt());
        }
            usleep(200);
        // //RIGHT
        cellId = superArea.GetCellId(avoidingInfo[i].rightIndex);
        if(cellId != -1){ //If the cell has a valid Id
            Index subAreaIndex = superArea.GetSubAreaIndex(avoidingInfo[i].rightIndex);
            //Draw in Rviz
            markers.CellMarkerUpdate(cellId, (State)avoidingInfo[i].rightState, superArea.GetCellPosition(avoidingInfo[i].rightIndex), subAreaIndex, superArea.GetNumSubAreasSqrt());
        }
            usleep(200);
        // //LEFT
        cellId = superArea.GetCellId(avoidingInfo[i].leftIndex);
        if(cellId != -1){ //If the cell has a valid Id
            Index subAreaIndex = superArea.GetSubAreaIndex(avoidingInfo[i].leftIndex);
            //Draw in Rviz
            markers.CellMarkerUpdate(cellId, (State)avoidingInfo[i].leftState, superArea.GetCellPosition(avoidingInfo[i].leftIndex), subAreaIndex, superArea.GetNumSubAreasSqrt());
        }
            usleep(200);
        // //UP-RIGHT
        cellId = superArea.GetCellId(avoidingInfo[i].upRightIndex);
        if(cellId != -1){ //If the cell has a valid Id
            Index subAreaIndex = superArea.GetSubAreaIndex(avoidingInfo[i].upRightIndex);
            //Draw in Rviz
            markers.CellMarkerUpdate(cellId, (State)avoidingInfo[i].upRightState, superArea.GetCellPosition(avoidingInfo[i].upRightIndex), subAreaIndex, superArea.GetNumSubAreasSqrt());
        }
            usleep(200);
        // //UP-LEFT
        cellId = superArea.GetCellId(avoidingInfo[i].upLeftIndex);
        if(cellId != -1){ //If the cell has a valid Id
            Index subAreaIndex = superArea.GetSubAreaIndex(avoidingInfo[i].upLeftIndex);
            //Draw in Rviz
            markers.CellMarkerUpdate(cellId, (State)avoidingInfo[i].upLeftState, superArea.GetCellPosition(avoidingInfo[i].upLeftIndex), subAreaIndex, superArea.GetNumSubAreasSqrt());
        }
            usleep(200);
        // //DOWN-RIGHT
        cellId = superArea.GetCellId(avoidingInfo[i].downRightIndex);
        if(cellId != -1){ //If the cell has a valid Id
            Index subAreaIndex = superArea.GetSubAreaIndex(avoidingInfo[i].downRightIndex);
            //Draw in Rviz
            markers.CellMarkerUpdate(cellId, (State)avoidingInfo[i].downRightState, superArea.GetCellPosition(avoidingInfo[i].downRightIndex), subAreaIndex, superArea.GetNumSubAreasSqrt());
        }
            usleep(200);
        // //DOWN-LEFT
        cellId = superArea.GetCellId(avoidingInfo[i].downLeftIndex);
        if(cellId != -1){ //If the cell has a valid Id
            Index subAreaIndex = superArea.GetSubAreaIndex(avoidingInfo[i].downLeftIndex);
            //Draw in Rviz
            markers.CellMarkerUpdate(cellId, (State)avoidingInfo[i].downLeftState, superArea.GetCellPosition(avoidingInfo[i].downLeftIndex), subAreaIndex, superArea.GetNumSubAreasSqrt());
        }      
            usleep(200);                 
        #pragma endregion

                                
        //Allow movement again
        TurtlebotManager::turtlebots[i]->SetAvoidingSlave(false);
        cout << "Stopping avoid for " << i << endl;
        //Resume movement for slave bot
        TurtlebotManager::turtlebots[i]->ResumeMovement();
        if(TurtlebotManager::turtlebots[i]->GetPathfindingStartPos() == true && TurtlebotManager::turtlebots[i]->GetMovementsSize() == 0){
            cout << "Finding path to start pos after avoiding " << i << endl;
                TurtlebotManager::turtlebots[i]->SetPathfindingStartPos(true); //Set this variable to true, so it only happens once!                                                              
                cout << "[" << i << "] is avoiding!. Will return to start position" << endl;
                Position goalPos = TurtlebotManager::turtlebots[i]->GetStartPos();
                list<Position> path = superArea.AStarPathfindingToStart(TurtlebotManager::turtlebots[i]->GetPosition(), goalPos, true);

                //Set pathfinding variables
                TurtlebotManager::turtlebots[i]->SetPathfinding(true);
                //Give the turtlebot movements
                for (auto const& p : path) {
                    TurtlebotManager::turtlebots[i]->NewMovement(traverse, p); 
                }
                TurtlebotManager::turtlebots[i]->SetPathfindingPoint(goalPos);
                cout << "-------------" << endl;
        }
    }



    void StartAvoiding(int turtlebotId, int otherTurtlebotId){
        //TurtlebotManager::turtlebots[turtlebotId]->EmptyList(); //Cancel this turtlebots movement
        TurtlebotManager::turtlebots[otherTurtlebotId]->PauseMovement(); //Pause the other turtlebots movement, and resume after the avoid algorithm
        
        TurtlebotManager::turtlebots[otherTurtlebotId]->SetAvoidingSlave(true);

        //Mark as tempWall
        Position otherTurtlebotPos = TurtlebotManager::turtlebots[otherTurtlebotId]->GetPosition();
        Position cellPos = superArea.GetNearestCellPosition(otherTurtlebotPos, Unexplored, Free, true);

        //AvoidingInfo
        //Variables for when the avoid algorithm ends
        AvoidingInfo a;
        a.cellPos = cellPos;
        a.avoiding = true;
        a.masterId = turtlebotId;

        #pragma region uglycode
        //UP
        Index upIndex = superArea.GetCellIndex(cellPos);
        upIndex.y = upIndex.y + 1;
        a.upState = superArea.GetCellState(upIndex);
        a.upIndex = upIndex;
        superArea.ChangeCellState(upIndex, TempWall);
        int cellId = superArea.GetCellId(upIndex);
        if(cellId != -1){ //If the cell has a valid Id
            Index subAreaIndex = superArea.GetSubAreaIndex(upIndex);
            //Draw in Rviz
            markers.CellMarkerUpdate(cellId, TempWall, superArea.GetCellPosition(upIndex), subAreaIndex, superArea.GetNumSubAreasSqrt());
        }
        usleep(200);
        //DOWN
        Index downIndex = superArea.GetCellIndex(cellPos);
        downIndex.y = downIndex.y - 1;
        a.downState = superArea.GetCellState(downIndex);
        a.downIndex = downIndex;
        superArea.ChangeCellState(downIndex, TempWall);
        cellId = superArea.GetCellId(downIndex);
        if(cellId != -1){ //If the cell has a valid Id
            Index subAreaIndex = superArea.GetSubAreaIndex(downIndex);
            //Draw in Rviz
            markers.CellMarkerUpdate(cellId, TempWall, superArea.GetCellPosition(downIndex), subAreaIndex, superArea.GetNumSubAreasSqrt());
        }
            usleep(200);
        //RIGHT
        Index rightIndex = superArea.GetCellIndex(cellPos);
        rightIndex.x = rightIndex.x + 1;
        a.rightState = superArea.GetCellState(rightIndex);
        a.rightIndex = rightIndex;
        superArea.ChangeCellState(rightIndex, TempWall);
        cellId = superArea.GetCellId(rightIndex);
        if(cellId != -1){ //If the cell has a valid Id
            Index subAreaIndex = superArea.GetSubAreaIndex(rightIndex);
            //Draw in Rviz
            markers.CellMarkerUpdate(cellId, TempWall, superArea.GetCellPosition(rightIndex), subAreaIndex, superArea.GetNumSubAreasSqrt());
        }
            usleep(200);
        //LEFT
        Index leftIndex = superArea.GetCellIndex(cellPos);
        leftIndex.x = leftIndex.x - 1;
        a.leftState = superArea.GetCellState(leftIndex);
        a.leftIndex = leftIndex;
        superArea.ChangeCellState(leftIndex, TempWall);
        cellId = superArea.GetCellId(leftIndex);
        if(cellId != -1){ //If the cell has a valid Id
            Index subAreaIndex = superArea.GetSubAreaIndex(leftIndex);
            //Draw in Rviz
            markers.CellMarkerUpdate(cellId, TempWall, superArea.GetCellPosition(leftIndex), subAreaIndex, superArea.GetNumSubAreasSqrt());
        }
            usleep(200);
        //UP-RIGHT
        Index upRightIndex = superArea.GetCellIndex(cellPos);
        upRightIndex.y = upRightIndex.y + 1;
        upRightIndex.x = upRightIndex.x + 1;
        a.upRightState = superArea.GetCellState(upRightIndex);
        a.upRightIndex = upRightIndex;
        superArea.ChangeCellState(upRightIndex, TempWall);
        cellId = superArea.GetCellId(upRightIndex);
        if(cellId != -1){ //If the cell has a valid Id
            Index subAreaIndex = superArea.GetSubAreaIndex(upRightIndex);
            //Draw in Rviz
            markers.CellMarkerUpdate(cellId, TempWall, superArea.GetCellPosition(upRightIndex), subAreaIndex, superArea.GetNumSubAreasSqrt());
        }
            usleep(200);
        //UP-LEFT
        Index upLeftIndex = superArea.GetCellIndex(cellPos);
        upLeftIndex.y = upLeftIndex.y + 1;
        upLeftIndex.x = upLeftIndex.x - 1;
        a.upLeftState = superArea.GetCellState(upLeftIndex);
        a.upLeftIndex = upLeftIndex;
        superArea.ChangeCellState(upLeftIndex, TempWall);
        cellId = superArea.GetCellId(upLeftIndex);
        if(cellId != -1){ //If the cell has a valid Id
            Index subAreaIndex = superArea.GetSubAreaIndex(upLeftIndex);
            //Draw in Rviz
            markers.CellMarkerUpdate(cellId, TempWall, superArea.GetCellPosition(upLeftIndex), subAreaIndex, superArea.GetNumSubAreasSqrt());
        }
            usleep(200);
        //DOWN-RIGHT   
        Index downRightIndex = superArea.GetCellIndex(cellPos);
        downRightIndex.x = downRightIndex.x + 1;
        downRightIndex.y = downRightIndex.y - 1;
        a.downRightState = superArea.GetCellState(downRightIndex);
        a.downRightIndex = downRightIndex;
        superArea.ChangeCellState(downRightIndex, TempWall);
        cellId = superArea.GetCellId(downRightIndex);
        if(cellId != -1){ //If the cell has a valid Id
            Index subAreaIndex = superArea.GetSubAreaIndex(downRightIndex);
            //Draw in Rviz
            markers.CellMarkerUpdate(cellId, TempWall, superArea.GetCellPosition(downRightIndex), subAreaIndex, superArea.GetNumSubAreasSqrt());
        }
            usleep(200);
        //DOWN-LEFT
        Index downLeftIndex = superArea.GetCellIndex(cellPos);
        downLeftIndex.x = downLeftIndex.x - 1;
        downLeftIndex.y = downLeftIndex.y - 1;
        a.downLeftState = superArea.GetCellState(downLeftIndex);
        a.downLeftIndex = downLeftIndex;
        superArea.ChangeCellState(downLeftIndex, TempWall);
        cellId = superArea.GetCellId(downLeftIndex);
        if(cellId != -1){ //If the cell has a valid Id
            Index subAreaIndex = superArea.GetSubAreaIndex(downLeftIndex);
            //Draw in Rviz
            markers.CellMarkerUpdate(cellId, TempWall, superArea.GetCellPosition(downLeftIndex), subAreaIndex, superArea.GetNumSubAreasSqrt());
        }          


        #pragma endregion
        //Move avoidingInfo to the array       
        avoidingInfo[otherTurtlebotId] = a;

    }


    //Compares two indexes, and returns if they are equal or not
    bool EqualIndexes(Index i, Index o){
        if(i.x == o.x && i.y == o.y){
            return true;
        }
        return false;
    }

    //When one robot begins avoiding (tempwalls around them), update all other robot's paths.
    void UpdateOtherRobotsPaths(int slaveRobot, int masterRobot){
        for(int i = 0; i < TurtlebotManager::numRobots; i++){
            if(i != slaveRobot){
                //New A* Pathfinding
                //If turtlebot was on its way to goalPos, then DO NOTHING
                if(TurtlebotManager::turtlebots[i]->GetPathfindingStartPos() == true){
                    /*TurtlebotManager::turtlebots[i]->EmptyList();
                    TurtlebotManager::turtlebots[i]->SetPathfindingStartPos(true); //Set this variable to true, so it only happens once!                                                              
                    cout << "[" << i << "] is avoiding!. Will return to start position" << endl;
                    Position goalPos = TurtlebotManager::turtlebots[i]->GetStartPos();
                    list<Position> path = superArea.AStarPathfindingToStart(TurtlebotManager::turtlebots[i]->GetPosition(), goalPos, true);

                    //Set pathfinding variables
                    TurtlebotManager::turtlebots[i]->SetPathfinding(true);
                    TurtlebotManager::turtlebots[i]->SetForcePathfind(true);
                    //Give the turtlebot movements
                    for (auto const& p : path) {
                        TurtlebotManager::turtlebots[i]->NewMovement(traverse, p); 
                    }
                    TurtlebotManager::turtlebots[i]->SetPathfindingPoint(goalPos);
                    cout << "-------------" << endl;*/

                } 
                else{
                    //Update the path of other robot, if their path will collide with the tempwall
                    bool willCollideWithTempWall = false;

                    for(auto const& p : TurtlebotManager::turtlebots[i]->GetMovements()){
                        Index pathIndex = superArea.GetCellIndex(p);
                        if(EqualIndexes(pathIndex, avoidingInfo[slaveRobot].upIndex)) willCollideWithTempWall = true;
                        else if(EqualIndexes(pathIndex, avoidingInfo[slaveRobot].downIndex)) willCollideWithTempWall = true;
                        else if(EqualIndexes(pathIndex, avoidingInfo[slaveRobot].rightIndex)) willCollideWithTempWall = true;
                        else if(EqualIndexes(pathIndex, avoidingInfo[slaveRobot].leftIndex)) willCollideWithTempWall = true;
                        else if(EqualIndexes(pathIndex, avoidingInfo[slaveRobot].upLeftIndex)) willCollideWithTempWall = true;
                        else if(EqualIndexes(pathIndex, avoidingInfo[slaveRobot].upRightIndex)) willCollideWithTempWall = true;
                        else if(EqualIndexes(pathIndex, avoidingInfo[slaveRobot].downRightIndex)) willCollideWithTempWall = true;
                        else if(EqualIndexes(pathIndex, avoidingInfo[slaveRobot].downLeftIndex)) willCollideWithTempWall = true;
                    }
                    if(willCollideWithTempWall){
                        if(i != masterRobot){
                            cout << i << " will collide with temp wall." << endl;
                            StartAStarPathfinding(i);
                        } 
                        else{
                            //If the master robot is stuck, then swap slave and master
                            cout << "New path for Master robot" << endl;
                            if(StartAStarPathfinding(i) == false){
                                cout << "STOPPING AVOID AS NO PATH WAS FOUND" << endl;
                                StopAvoiding(slaveRobot);
                                
                                StartAvoiding(slaveRobot, i);
                                UpdateOtherRobotsPaths(i, slaveRobot);
                            }
                        }
                    }
                }
            }
        }
    }


    //If a turtlebot meets another turtlebot in a given radius
    void AvoidTurtlebots(int turtlebotId, int otherTurtlebotId){
        //If turtlebot isn't currently a slave (standing still with tempwalls around)
        if(TurtlebotManager::turtlebots[turtlebotId]->GetAvoidingSlave() == true)
            return;

        //If the Other turtlebot is already a slave, do nothing.
        if(TurtlebotManager::turtlebots[otherTurtlebotId]->GetAvoidingSlave() == true)
            return;

        //The turtlebot with the lowest id controls the avoidance algorithm
        if(turtlebotId > otherTurtlebotId){
            cout << "[" << turtlebotId << "] will be controlled by: [" << otherTurtlebotId << "]" << endl;
        }
        else{ //This turtlebot has the lowest id.
            cout << "[" << turtlebotId << "] will control: [" << otherTurtlebotId << "]" << endl;
            StartAvoiding(turtlebotId, otherTurtlebotId);

            UpdateOtherRobotsPaths(otherTurtlebotId, turtlebotId);

            TurtlebotManager::turtlebots[turtlebotId]->EmptyNewPoint();
        }
        cout << "" << endl;
    }
 
    void CheckIfStopAvoiding(){ //Stops avoiding after x amounts of seconds. 
        for(int i = 0; i < TurtlebotManager::numRobots; i++){
            if(TurtlebotManager::turtlebots[i]->GetAvoidingSlave() == true){ //If robot i is avoiding
                //Check if avoiding array is available.
                if(avoidingInfo[i].avoiding == true){
                    //Check the disstance between two robots (If they are over x m)
                    if(superArea.ComparePositions(TurtlebotManager::turtlebots[i]->GetPosition(), 
                                                  TurtlebotManager::turtlebots[avoidingInfo[i].masterId]->GetPosition(), (cellSpace * 2) + 0.05) == false){
                                                      StopAvoiding(i);
                    }                                   
                }
            }
        }
    }



    //Gets a new point from each turtlebot. Uses this point for path planning if its a wall
    void GetPoints(){
        //Array containting info on which robot foudn another
        int foundOtherTurtlebot[3];
        foundOtherTurtlebot[0] = false;
        foundOtherTurtlebot[1] = false;
        foundOtherTurtlebot[2] = false;
        //E.g if foundOtherTurtlebot[1] == true, then turtlebot with Id 1 found another robot 
        for(int i = 0; i < TurtlebotManager::numRobots; i++){
            Position newPoint = TurtlebotManager::turtlebots[i]->GetPoint();
            if(newPoint.x != 0 && newPoint.y != 0){
                //Check if the point is another turtlebot
                bool isTurtlebot = false;
                for(int j = 0; j < TurtlebotManager::numRobots; j++){
                    if(i != j && foundOtherTurtlebot[i] == false){ //Do not compare positions if it's the same robot, and if it has already found another robot
                        foundOtherTurtlebot[i] = superArea.ComparePositions(TurtlebotManager::turtlebots[j]->GetPosition(), newPoint, cellSpace);
                    }
                }
                if(foundOtherTurtlebot[i] == false){ //If its a wall, mark the point and begin path planning if needed
                    MarkWallPoint(newPoint, i);
                }
                else{
                    TurtlebotManager::turtlebots[i]->EmptyNewPoint(); //Reset point :)
                }
                //New turtlebot was found. 
            }
        }
    }


    //Checks if a turtlebot has reached a free cell. 
    void CheckFreeCell()
    {
        for(int i = 0; i < TurtlebotManager::numRobots; i++)
        {
            Position cellPos = TurtlebotManager::turtlebots[i]->GetfreeCell(); //Freeccell point is updated each time the robot reaches its goal position
            if(cellPos.x != 0 && cellPos.y != 0){

                //Check if the cell is valid
                if(superArea.CheckIfCellExists(cellPos)){

                    //rViz marker
                    int cellId = superArea.GetCellId(cellPos);
                    if(cellId != -1){ //If the cell has a valid Id
                        Index i = superArea.GetCellIndex(cellPos);
                        Index subAreaIndex = superArea.GetSubAreaIndex(i);
                        markers.CellMarkerUpdate(cellId, Free, cellPos, subAreaIndex, superArea.GetNumSubAreasSqrt());
                    }

                    superArea.ChangeCellState(cellPos, Free); //Changes the cell state to Free
                    TurtlebotManager::turtlebots[i]->EmptyfreeCell(); //Resets the "freecell" pos to (0,0)

                    //The robot could have finished A*. Check if using pathfinding and the movements list is empty
                    if(TurtlebotManager::turtlebots[i]->GetPathfinding() == true && TurtlebotManager::turtlebots[i]->GetMovementsSize() == 0){
                        TurtlebotManager::turtlebots[i]->SetPathfinding(false);
                        TurtlebotManager::turtlebots[i]->SetForcePathfind(false);
                        if(TurtlebotManager::turtlebots[i]->GetPathfindingStartPos() == true)
                            TurtlebotManager::turtlebots[i]->finished = true;
                        //cout << "Stopping pathfinding for [" << i << "]" << endl;

                        //The code below forces the robot to go to a new point after returning to start pos. (Used for pathfinding test)
                        //If the robot reached its start pos destination
                       // if(TurtlebotManager::turtlebots[i]->GetPathfindingStartPos() == true){

                       // }

                    }

                    
                    //If the robot isn't pathfinding (using A*), then find a new cell for the robot to travel to

                    if(TurtlebotManager::turtlebots[i]->GetPathfinding() == false){

                        //Find new cell for turtlebot
                        Position newCell = superArea.GetNextCell(cellPos, TurtlebotManager::turtlebots[i]->GetRotation());

                        //If "newCell" was found using PSO
                        if(newCell.x != -1){

                            //cout << "Robot [" << i << "]" << "'s next pos: (" << newCell.x << "," << newCell.y << ")" << endl;
                            //cout << "----------------------------------" << endl;   
                            TurtlebotManager::turtlebots[i]->NewMovement(traverse, newCell);
                        }
                        //"NewCell could not be found, and A* will then find the next position"
                        else{
                            //cout << "New cell could not be found using PSO, A* will be used" << endl;
                            StartAStarPathfinding(i);
                        }
                    }
                    //If the robot is pathfinding with A*, then check if the path hasn't been changed by another turtlebot to "Wall" or "Free"
                    //Do not do this if pahtfinding back to startPos, as startPos will always be free.
                    else if(TurtlebotManager::turtlebots[i]->GetPathfinding() == true && TurtlebotManager::turtlebots[i]->GetPathfindingStartPos() == false){
                        //Check the A* final path point, to see if it has been changed
                        if(TurtlebotManager::turtlebots[i]->GetMovementsSize() > 0){
                            //Check if the path has been changed to a wall.
                            for(auto const& p : TurtlebotManager::turtlebots[i]->GetMovements()){
                                int cellState = superArea.GetCellState(p);
                                if(cellState == Wall || cellState == TempWall){
                                    StartAStarPathfinding(i);
                                }
                            }
                            //Check if the goal pos has been changed to free (Another bot reached this cell first.)
                            if(superArea.GetCellState(TurtlebotManager::turtlebots[i]->GetMovements().back()) == Free){
                                StartAStarPathfinding(i);
                            }
                        }                       
                    }
                }         
            }
        }
    }

    void GetTurtlebotPositions(){
        //Check if two turtlebots are near each other.
        for(int i = 0; i < TurtlebotManager::numRobots; i++){
            for(int j = 0; j <TurtlebotManager::numRobots; j++){
                if(i != j){
                    Position turtleBotPos = TurtlebotManager::turtlebots[i]->GetPosition();
                    Position otherTurtleBotPos = TurtlebotManager::turtlebots[j]->GetPosition();

                    if(superArea.ComparePositions(turtleBotPos, otherTurtleBotPos, cellSpace * 2)){
                        MarkersManager::AvoidTurtlebots(i, j);
                    }
                }
            }
        }
    }



    void FreeRobots(){ //iff robots are stuck. attempt to free
        for(int i = 0; i < TurtlebotManager::numRobots; i++){ //Change back to two.
            //Reset stuck timer.
            if(TurtlebotManager::turtlebots[i]->GetAvoidingSlave() == true || TurtlebotManager::turtlebots[i]->finished == true || TurtlebotManager::turtlebots[i]->GetMovementsSize() > 0)
                stuckTimer[i] = 0;
            if(TurtlebotManager::turtlebots[i]->GetMovementsSize() == 0 ){
                stuckTimer[i]++;
                if(stuckTimer [i] > 30){//Stuck over 3 seconds
                    cout << "Freeing robot [" << i << "]" << endl;
                    StartAStarPathfinding(i);
                    stuckTimer[i] = 0;
                }
            }
        }
    }

    void SpreadOutFixedPositions(Index goalIndex1, Index goalIndex2){
        //Start PSO
        Position goalPos = superArea.GetNearestCellAStar(TurtlebotManager::turtlebots[0]->GetPosition(), Unexplored);
        TurtlebotManager::turtlebots[0]->NewMovement(traverse, goalPos);

        goalPos = superArea.GetCellPosition(goalIndex1);
        list<Position> path = superArea.AStarPathfinding(TurtlebotManager::turtlebots[1]->GetPosition(), goalPos, false); //Set to true to show path in terminal
        //Set pathfinding variables
        TurtlebotManager::turtlebots[1]->SetPathfinding(true);
        TurtlebotManager::turtlebots[1]->SetPathfindingPoint(goalPos);
       // TurtlebotManager::turtlebots[1]->SetForcePathfind(true);
        //Give the turtlebot movements
        for (auto const& p : path) {
            TurtlebotManager::turtlebots[1]->NewMovement(traverse, p); 
        }
        Index newSubArea = superArea.GetSubArea(goalPos);
        cout << "[1] will traverse to SubArea:(" << newSubArea.x << " , " << newSubArea.y << ")" << endl;

        goalPos = superArea.GetCellPosition(goalIndex2);
        path = superArea.AStarPathfinding(TurtlebotManager::turtlebots[2]->GetPosition(), goalPos, false); //Set to true to show path in terminal
        //Set pathfinding variables
        TurtlebotManager::turtlebots[2]->SetPathfinding(true);
        TurtlebotManager::turtlebots[2]->SetPathfindingPoint(goalPos);
       // TurtlebotManager::turtlebots[2]->SetForcePathfind(true);
        //Give the turtlebot movements
        for (auto const& p : path) {
            TurtlebotManager::turtlebots[2]->NewMovement(traverse, p); 
        }
        newSubArea = superArea.GetSubArea(goalPos);
        cout << "[2] will traverse to SubArea:(" << newSubArea.x << " , " << newSubArea.y << ")" << endl;

    }

    void SpreadOutNearestSubAreas(){
        //Turtlebot 0
        Position goalPos = superArea.GetNearestCellAStar(TurtlebotManager::turtlebots[0]->GetPosition(), Unexplored);
        TurtlebotManager::turtlebots[0]->NewMovement(traverse, goalPos);

        //Turtlebot 1
        AStarPathInfo pathInfo;
        pathInfo = superArea.GetNearestCellAStarAnotherSubArea(TurtlebotManager::turtlebots[1]->GetPosition(), Unexplored, TurtlebotManager::GetOtherTurtlebotsPosition(1), TurtlebotManager::GetOtherTurtlebotsGoalPosition(1), false);
 
        if(pathInfo.cellPos.x != -1) { //Check if A* could find a cell in another subarea
            //Set pathfinding variables
            TurtlebotManager::turtlebots[1]->SetPathfinding(true);
            TurtlebotManager::turtlebots[1]->SetPathfindingPoint(pathInfo.cellPos);
            TurtlebotManager::turtlebots[1]->SetForcePathfind(true);
            //Give the turtlebot movements
            for (auto const& p : pathInfo.path) {
                TurtlebotManager::turtlebots[1]->NewMovement(traverse, p); 
            }
            Index newSubArea = superArea.GetSubArea(pathInfo.cellPos);
            cout << "[1] will traverse to SubArea:(" << newSubArea.x << " , " << newSubArea.y << ")" << endl;
        }

        // //Turtlebot 2
        // pathInfo = superArea.GetNearestCellAStarAnotherSubArea(TurtlebotManager::turtlebots[2]->GetPosition(), Unexplored,  TurtlebotManager::GetOtherTurtlebotsPosition(2), TurtlebotManager::GetOtherTurtlebotsGoalPosition(2), false);
 
        // if(pathInfo.cellPos.x != -1) { //Check if A* could find a cell in another subarea
        //     //Set pathfinding variables
        //     TurtlebotManager::turtlebots[2]->SetPathfinding(true);
        //     TurtlebotManager::turtlebots[2]->SetPathfindingPoint(pathInfo.cellPos);
        //     TurtlebotManager::turtlebots[2]->SetForcePathfind(true);
        //     //Give the turtlebot movements
        //     for (auto const& p : pathInfo.path) {
        //         TurtlebotManager::turtlebots[2]->NewMovement(traverse, p); 
        //     }
        //     Index newSubArea = superArea.GetSubArea(pathInfo.cellPos);
        //     cout << "[2] will traverse to SubArea:(" << newSubArea.x << " , " << newSubArea.y << ")" << endl;
        // }

    }
}



int main(int argc, char *argv[])
{

    init(argc, argv, "Swarm_Robots");
    //Wait until ros is initialized before creating a nodehandle.
    ROS_INFO("Hello World");

    TurtlebotManager::InitializeTurtlebots();
    MarkersManager::InitializeMarkers();

    //Main loop
    Rate loop_rate(10);

    Index goalIndex1; goalIndex1.x = 24; goalIndex1.y = 3;
    Index goalIndex2; goalIndex2.x = 11; goalIndex2.y = 16;

    //MarkersManager::SpreadOut(goalIndex1, goalIndex2);

    MarkersManager::SpreadOutNearestSubAreas();

    while (ok())
    {
        MarkersManager::DrawRobotMarkers();
        TurtlebotManager::MoveTurtlebots();
        MarkersManager::DrawPoints(); //todo create callback function for get points
        MarkersManager::CheckFreeCell();
        MarkersManager::GetPoints(); //Gets a new point from each turtlebot. Uses this point for path planning if its a wall
        MarkersManager::GetTurtlebotPositions();
        MarkersManager::CheckIfStopAvoiding(); //Reverts all "TempWall" cells back to "Free" after x amount of seconds
        MarkersManager::FreeRobots(); //Attemps to unstuck a robot if they have been idle for 10 seconds.
    

        ros::spinOnce(); //Spin for callback functions 
        loop_rate.sleep();
    }
    return 0;    



}