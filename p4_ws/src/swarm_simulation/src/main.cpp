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
        Position pos0; pos0.x = 2; pos0.y = 2;
        Position pos1; pos1.x = 1; pos1.y = 3;
        Position pos2; pos2.x = 4; pos2.y = 1;

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

    // void ExploreNextSubArea(){
    //     Position robotPosition;
    //     Position 0Pos;
    //     Position 1Pos;           //This could be better
    //     Position 2Pos;
    //     Position 3Pos;
    //     for(int i = 0; i < numRobots; i++){
    //             robotPosition = turtlebots[i]->GetPosition();
    //             int robotSubArea = superArea.GetSubArea(robotPosition);
    //         if(all cells in current subarea are explored){
    //             int freeSubArea = CheckFreeArea();
    //             0Pos.x = 1.2;
    //             0Pos.y = 4;
    //             1Pos.x = 40;
    //             1Pos.y = 4;
    //             2Pos.x = 1.2;
    //             2Pos.y = 40;
    //             3Pos.x = 40;
    //             3Pos.y = 40;
    //             switch(freeSubArea){
    //                  Case 0:
                            // Turtlebot::NewMovement(traverse, 0Pos);
                            // break;
    //                  Case 1: 
                            // Turtlebot::NewMovement(traverse, 1Pos);
                            // break;
    //                  Case 2:
                            // Turtlebot::NewMovement(traverse, 2Pos);
                            // break;
    //                  Case 3:
                            // Turtlebot::NewMovement(traverse, 3Pos);
                            // break;
    //                  default:
    //                      cout << "No such subarea" << endl;
    //                      break;
    //             }

                

                // Get next unexplored subarea
                // Go to next unexplored subarea
                // Call colorchanging function from marker

            //}
        //}
    //}

   /* int CheckFreeArea(){
        Position robotPos = turtlebots[i]->GetPosition();
        for(int i = 0; i < numRobots; i++){
                    Position robotPos = turtlebots[i]->GetPosition();
            for(int i = 0; i < superArea.GetNumSubAreas(); i++){
                if(robot position is not in subarea){
                    return i; 
                }
            }
        }
    }*/
}

struct AvoidingTimer{ //Struct used for the avoid algorithm.
    Position cellPos; //The cell that will be marked as tempWall
    double revertTime; //The time when the cell will go back to Free (in seconds)
    int turtlebotId;
    int otherTurtlebotId;
    bool available = false;
};

namespace MarkersManager{
    Markers markers;
    //(Super Area size / cellDistance) must be divideable by numSubAreas.
    //Super are size must be even
    float cellSpace = 0.3f;
   // SuperArea superArea(24, 16, cellSpace); //Prev: 24, 16, 0.3
    SuperArea superArea(18, 4, cellSpace);
    AvoidingTimer avoidingTimer[3];//Array of all tempWall cells that will be reverted to Free after x amount of seconds
    //list<TempWallTimer> tempWalls; //List of all tempWall cells that will be reverted to Free after x amount of seconds

    void DrawAllCells(){
        usleep(1000000);
        //Create marker for each cell

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
                usleep(2000); //Small delay (in microseconds) to give rViz time.
            }
        }
    }

    void InitializeMarkers(){

        //Setups each different topic
        markers.SetupMarker(); 
        markers.SetupRobotMarker();
        markers.SetupCellMarker();
        markers.SetupMlineMarker();

        ROS_INFO("Markers initialized");   

        DrawAllCells();
    }


    void DrawPoints(){
        for(int i = 0; i < TurtlebotManager::numRobots; i++){
            if(TurtlebotManager::turtlebots[i]->GetPoint().x != 0){ //Check if the point exists
                usleep(1000);//Wait 1 ms so rviz can follow
                markers.NewMarker(TurtlebotManager::turtlebots[i]->GetPoint(), TurtlebotManager::turtlebots[i]->GetId());
            }
        }
    }

    void DrawRobotMarkers()
    {
        for(int i = 0; i < TurtlebotManager::numRobots; i++)
        {
            usleep(1000); //Wait 1 ms so rviz can follow
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
    void StartAStarPathfinding(int turtlebotId){
        //Stop the current movement
        TurtlebotManager::turtlebots[turtlebotId]->EmptyList();
        //Find a new point using A*
        Position turtlebotPos = TurtlebotManager::turtlebots[turtlebotId]->GetPosition();
        Position goalPos = superArea.GetNearestCellAStar(turtlebotPos, Unexplored);
        
        if(goalPos.x != -1){ //If goalPos was found using A*
            list<Position> path = superArea.AStarPathfinding(turtlebotPos, goalPos, true);
            //Set pathfinding variables
            TurtlebotManager::turtlebots[turtlebotId]->SetPathfinding(true);
            TurtlebotManager::turtlebots[turtlebotId]->SetPathfindingPoint(goalPos);
            //Give the turtlebot movements
            for (auto const& p : path) {
                TurtlebotManager::turtlebots[turtlebotId]->NewMovement(traverse, p); 
            }
        }
        else{ //If no path can be found in the turtlebots subarea, find next subarea
            cout << "No movements for turtlebot " << turtlebotId << " in this subarea. New subarea will be found " << endl;
            AStarPathInfo pathInfo;
            pathInfo = superArea.GetNearestCellAStarAnotherSubArea(turtlebotPos, Unexplored);

            if(pathInfo.cellPos.x != -1) { //Check if A* could find a cell in another subarea
                //Set pathfinding variables
                TurtlebotManager::turtlebots[turtlebotId]->SetPathfinding(true);
                TurtlebotManager::turtlebots[turtlebotId]->SetPathfindingPoint(pathInfo.cellPos);
                //Give the turtlebot movements
                for (auto const& p : pathInfo.path) {
                    TurtlebotManager::turtlebots[turtlebotId]->NewMovement(traverse, p); 
                }
                Index newSubArea = superArea.GetSubArea(pathInfo.cellPos);
                cout << "New Sub Area: ";
                cout << "(" << newSubArea.x << " , " << newSubArea.y << ")" << endl;
            }
            else{ //No valid points could be found in any other subarea. The robot will then return to start position
                cout << "[" << turtlebotId << "] Cannot find new subareas to explore. Will return to start position" << endl;
                goalPos = TurtlebotManager::turtlebots[turtlebotId]->GetStartPos();
                list<Position> path = superArea.AStarPathfinding(turtlebotPos, goalPos, true);
                //Set pathfinding variables
                TurtlebotManager::turtlebots[turtlebotId]->SetPathfinding(true);
                TurtlebotManager::turtlebots[turtlebotId]->SetPathfindingPoint(goalPos);
                //Give the turtlebot movements
                for (auto const& p : path) {
                    TurtlebotManager::turtlebots[turtlebotId]->NewMovement(traverse, p); 
                }
            }
        }
    }

    //When the robot finds a new wall, checks if the wall is the next goal when using PSO. If true, use A*
    void UpdatePSOPathfinding(Position wallPos, int turtlebotId){
        //cout << "Update PSO Pathfinding!" << endl;
        if(TurtlebotManager::turtlebots[turtlebotId]->GetPathfinding() == false){
            //cout << "Using PSO!" << endl;
            //Check if the robot will collide with its goal pos
            bool collision = superArea.CheckForCollision(wallPos, TurtlebotManager::turtlebots[turtlebotId]->GetGoalPos());
            if(collision){ //if collide
                cout << "Robot [" << turtlebotId << "]'s goal position will collide with a wall. Use A*" << endl; 
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
                cout << "Wall is a part of A* PATH, new path made" << endl;

                if(TurtlebotManager::turtlebots[turtlebotId]->GetForcePathfind() == false) //If not forcing the goalPos, find the nearest cell and pathfind to it
                    StartAStarPathfinding(turtlebotId);
                else{ //If forcing the turtlebot to move to a specific point (E.g at the start when spreading out)
                    cout << "Forcing pathfind !!!" << endl;
                    TurtlebotManager::turtlebots[turtlebotId]->EmptyList();
                    //Find a new point using A*
                    Position turtlebotPos = TurtlebotManager::turtlebots[turtlebotId]->GetPosition();
                    Position goalPos = TurtlebotManager::turtlebots[turtlebotId]->GetPathfindingPoint();
                    
                    if(goalPos.x != -1){ //If goalPos was found using A*
                        list<Position> path = superArea.AStarPathfinding(turtlebotPos, goalPos, true);
                        //Give the turtlebot movements
                        for (auto const& p : path) {
                            TurtlebotManager::turtlebots[turtlebotId]->NewMovement(traverse, p); 
                        }
                    }
                    else{
                        TurtlebotManager::turtlebots[turtlebotId]->SetForcePathfind(false);
                        StartAStarPathfinding(turtlebotId);
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

    //If a turtlebot meets another turtlebot in a given radius
    void AvoidTurtlebots(int turtlebotId, int otherTurtlebotId){
        //If the turtlebot isn't currently avoiding.
        if(TurtlebotManager::turtlebots[turtlebotId]->GetAvoiding() == true)
            return;

        //The turtlebot with the lowest id controls the avoidance algorithm
        if(turtlebotId > otherTurtlebotId){
            cout << "[" << otherTurtlebotId << "] will be controlled by: [" << turtlebotId << "]" << endl;
        }
        else{ //This turtlebot has the lowest id.
            cout << "[" << turtlebotId << "] will control: [" << otherTurtlebotId << "]" << endl;
            TurtlebotManager::turtlebots[turtlebotId]->EmptyList(); //Cancel this turtlebots movement
            TurtlebotManager::turtlebots[otherTurtlebotId]->PauseMovement(); //Pause the other turtlebots movement, and resume after the avoid algorithm
            
            TurtlebotManager::turtlebots[turtlebotId]->SetAvoiding(true);
            TurtlebotManager::turtlebots[otherTurtlebotId]->SetAvoiding(true);

            //Mark as tempWall(johnny boy)
            Position otherTurtlebotPos = TurtlebotManager::turtlebots[otherTurtlebotId]->GetPosition();
            Position cellPos = superArea.GetNearestCellPosition(otherTurtlebotPos, Unexplored, Free, true);
            superArea.ChangeCellState(cellPos, TempWall);
            int cellId = superArea.GetCellId(cellPos);
            if(cellId != -1){ //If the cell has a valid Id
                Index i = superArea.GetCellIndex(cellPos);
                Index subAreaIndex = superArea.GetSubAreaIndex(i);
                //Draw in Rviz
                markers.CellMarkerUpdate(cellId, TempWall, cellPos, subAreaIndex, superArea.GetNumSubAreasSqrt());
            }


            //New A* Pathfinding
            StartAStarPathfinding(turtlebotId);
            TurtlebotManager::turtlebots[turtlebotId]->EmptyNewPoint();
            //Start timer to end the avoid algorithm 
            AvoidingTimer a;
            a.cellPos = cellPos;
            a.revertTime = Time::now().toSec() +10;
            a.available = true;
            a.turtlebotId = turtlebotId;
            a.otherTurtlebotId = otherTurtlebotId;
            cout << "Time now: " << Time::now().toSec() << endl;
            avoidingTimer[turtlebotId] = a;
        }

        cout << "" << endl;
    }

    void StopAvoiding(){ //Stops avoiding after x amounts of seconds. 
        for(int i = 0; i < TurtlebotManager::numRobots; i++){
            if(Time::now().toSec() >= avoidingTimer[i].revertTime && avoidingTimer[i].available){ //Checks if x amount of seconds have passed
                avoidingTimer[i].available = false;
                superArea.ChangeCellState(avoidingTimer[i].cellPos, Free); //Change the cell state to free
                //Allow movement again
                TurtlebotManager::turtlebots[avoidingTimer[i].turtlebotId]->SetAvoiding(false);
                TurtlebotManager::turtlebots[avoidingTimer[i].otherTurtlebotId]->SetAvoiding(false);
                //Resume movement for otherTurtlebot
                TurtlebotManager::turtlebots[avoidingTimer[i].otherTurtlebotId]->ResumeMovement();

                //Get cell Id to draw in Rviz
                int cellId = superArea.GetCellId(avoidingTimer[i].cellPos);
                if(cellId != -1){ //If the cell has a valid Id
                    Index index = superArea.GetCellIndex(avoidingTimer[i].cellPos);
                    Index subAreaIndex = superArea.GetSubAreaIndex(index);
                    //Draw in Rviz
                    markers.CellMarkerUpdate(cellId, Free, avoidingTimer[i].cellPos, subAreaIndex, superArea.GetNumSubAreasSqrt());
                }
            }
        }
    }

    //Gets a new point from each turtlebot. Uses this point for path planning if its a wall
    void GetPoints(){
        //FIX LATER TO NUMROBOTS
        for(int i = 0; i < TurtlebotManager::numRobots; i++){
            Position newPoint = TurtlebotManager::turtlebots[i]->GetPoint();
            if(newPoint.x != 0 && newPoint.y != 0){
                //Check if the point is another turtlebot
                bool isTurtlebot = false;
                for(int j = 0; j < TurtlebotManager::numRobots; j++){
                    if(i != j){
                        isTurtlebot = superArea.ComparePositions(TurtlebotManager::turtlebots[j]->GetPosition(), newPoint, 0.3);
                        if(isTurtlebot){
                            //cout << "Robot [" << i <<"] found turtlebot [" << j << "]" << endl;
                            //AvoidTurtlebots(i, j);
                            break;
                        }
                    }
                }
                if(isTurtlebot == false){ //If its a wall, mark the point and begin path planning if needed
                    MarkWallPoint(newPoint, i);
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

                    //cout << "Found new cell pos at (" << cellPos.x << ", " << cellPos.y << ")" << endl;
                    
                    int cellId = superArea.GetCellId(cellPos);
                    if(cellId != -1){ //If the cell has a valid Id
                        Index i = superArea.GetCellIndex(cellPos);
                        Index subAreaIndex = superArea.GetSubAreaIndex(i);
                        markers.CellMarkerUpdate(cellId, Free, cellPos, subAreaIndex, superArea.GetNumSubAreasSqrt());
                    }
//                        markers.CellMarkerUpdate(cellId, Free, cellPos, subAreaIndex, superArea.GetNumSubAreasSqrt());

                    superArea.ChangeCellState(cellPos, Free); //Changes the cell state to Free
                    TurtlebotManager::turtlebots[i]->EmptyfreeCell(); //Resets the "freecell" pos to (0,0)



                    //The robot could have finished A*. Check if using pathfinding and the movements list is empty
                    if(TurtlebotManager::turtlebots[i]->GetPathfinding() == true && TurtlebotManager::turtlebots[i]->GetMovementsSize() == 0){
                        TurtlebotManager::turtlebots[i]->SetPathfinding(false);
                        cout << "Stopping pathfinding for [" << i << "]" << endl;
                    }
                    
                    //If the robot isn't pathfinding (using A*), then find a new cell for the robot to travel to

                    if(TurtlebotManager::turtlebots[i]->GetPathfinding() == false){

                        //Find new cell for turtlebot
                        Position newCell = superArea.GetNextCell(cellPos, TurtlebotManager::turtlebots[i]->GetRotation());

                        //If "newCell" was found using PSO
                        if(newCell.x != -1){

                            cout << "Robot [" << i << "]" << "'s next pos: (" << newCell.x << "," << newCell.y << ")" << endl;
                            cout << "----------------------------------" << endl;   
                            TurtlebotManager::turtlebots[i]->NewMovement(traverse, newCell);
                        }
                        //"NewCell could not be found, and A* will then find the next position"
                        else{
                            cout << "New cell could not be found using PSO, A* will be used" << endl;
                            StartAStarPathfinding(i);
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

                    if(superArea.ComparePositions(turtleBotPos, otherTurtleBotPos, 0.8)){
                        MarkersManager::AvoidTurtlebots(i, j);
                    }
                    
                }
            }
        }
    }

    void SpreadOut(){
        //Start PSO
        Position goalPos = superArea.GetNearestCellAStar(TurtlebotManager::turtlebots[0]->GetPosition(), Unexplored);
        AStarPathInfo pathInfo;
        pathInfo = superArea.GetNearestCellAStarAnotherSubArea(TurtlebotManager::turtlebots[1]->GetPosition(), Unexplored);
        AStarPathInfo pathInfo2;
        pathInfo2 = superArea.GetNearestCellAStarAnotherSubArea(TurtlebotManager::turtlebots[2]->GetPosition(), Unexplored);

        TurtlebotManager::turtlebots[0]->NewMovement(traverse, goalPos);

    


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

 
        if(pathInfo.cellPos.x != -1) { //Check if A* could find a cell in another subarea
            //Set pathfinding variables
            TurtlebotManager::turtlebots[2]->SetPathfinding(true);
            TurtlebotManager::turtlebots[2]->SetPathfindingPoint(pathInfo2.cellPos);
            //Give the turtlebot movements
            for (auto const& p : pathInfo2.path) {
                TurtlebotManager::turtlebots[2]->NewMovement(traverse, p); 
            }
            Index newSubArea = superArea.GetSubArea(pathInfo2.cellPos);
            cout << "[2] will traverse to SubArea:(" << newSubArea.x << " , " << newSubArea.y << ")" << endl;
        }
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

    MarkersManager::SpreadOut();

    //Move a turtlebot
    //Position cellPos = MarkersManager::superArea.GetNearestCellPosition(TurtlebotManager::turtlebots[0]->GetPosition(), Unexplored, false);

    //MarkersManager::DrawMLine(i, cell);  //Goal should be "cell"
    //TurtlebotManager::turtlebots[0]->NewMovement(traverse, cellPos);


    // Position p;
    // p.x = 4.2;
    // p.y = 4.2;
    // MarkersManager::superArea.GetNearestCellAStar(p, Unexplored);
   
//    cout << "x         x" << endl << endl << "-----------" << endl;


    while (ok())
    {
        MarkersManager::DrawRobotMarkers();
        TurtlebotManager::MoveTurtlebots();
        MarkersManager::DrawPoints(); //todo create callback function for get points
        MarkersManager::CheckFreeCell();
        MarkersManager::GetPoints(); //Gets a new point from each turtlebot. Uses this point for path planning if its a wall
        MarkersManager::GetTurtlebotPositions();
        MarkersManager::StopAvoiding(); //Reverts all "TempWall" cells back to "Free" after x amount of seconds
        
        ros::spinOnce(); //Spin for callback functions 
        loop_rate.sleep();
    }
    return 0;    



}