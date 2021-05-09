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
        Position pos0; pos0.x = 1; pos0.y = 1;
        Position pos1; pos1.x = 1; pos1.y = 6.5;
        Position pos2; pos2.x = 5; pos2.y = 9.5;

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

namespace MarkersManager{
    Markers markers;
    //(Super Area size / cellDistance) must be divideable by numSubAreas.
    SuperArea superArea(12, 9, 0.4); //Prev: 15, 9, 0.4

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
                markers.CellMarker(cellPos, Unexplored);
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
            usleep(1000);//Wait 1 ms so rviz can follow
            markers.NewMarker(TurtlebotManager::turtlebots[i]->GetPoint(), TurtlebotManager::turtlebots[i]->GetId());
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
        list<Position> path = superArea.AStarPathfinding(TurtlebotManager::turtlebots[turtlebotId]->GetPosition(), superArea.GetNearestCellPosition(TurtlebotManager::turtlebots[turtlebotId]->GetPosition(), Unexplored, true));
        TurtlebotManager::turtlebots[turtlebotId]->SetPathfinding(true);
        TurtlebotManager::turtlebots[turtlebotId]->SetPathfindingPoint(superArea.GetNearestCellPosition(TurtlebotManager::turtlebots[turtlebotId]->GetPosition(), Unexplored, true));
        for (auto const& p : path) {
            TurtlebotManager::turtlebots[turtlebotId]->NewMovement(traverse, p); 
        }
    }

    //When the robot finds a new wall, checks if the wall is the next goal when using PSO
    void UpdatePSOPathfinding(Position wallPos, int turtlebotId){
        //cout << "Update PSO Pathfinding!" << endl;
        if(TurtlebotManager::turtlebots[turtlebotId]->GetPathfinding() == false){
            //cout << "Using PSO!" << endl;
            //Check if the robot will collide with its goal pos
            bool collision = superArea.CheckForCollision(wallPos, TurtlebotManager::turtlebots[turtlebotId]->GetGoalPos());
            if(collision){ //if collide
                cout << "The robot's goal position will collide with a wall. Use A*" << endl; 
                StartAStarPathfinding(turtlebotId);
            }
        }          
    }

    //When the robot finds a new wall, update the pathfinding to check if the wall is in the path.
    void UpdateAStarPathfinding(Position wallPos, int turtlebotId){
        if(TurtlebotManager::turtlebots[turtlebotId]->GetPathfinding() == true){ //If the robot is pathfinding
            //Check if the wall is a part of the path
            //cout << "New Wall found, check if its a part of PATHS" << endl;
            cout << "New wall found when traversing with A*. Updating A* path to see if the wall is in the way." << endl;
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

                StartAStarPathfinding(turtlebotId);
            }
        }
    }

    //Marks the wall point in rViz, and updates the cell state to wall.
    void MarkWallPoint(Position wallPos, int turtlebotId){
        //Checks if the point is a wall.
        CellInfo cellInfo = superArea.NewWallPoint(wallPos); //Changes the state of the cell to Wall if its a wall
        //Check if the wall has been found
        if(cellInfo.id != -1){
            markers.CellMarkerUpdate(cellInfo.id, Wall, cellInfo.pos); //Updates the point in rViz
            UpdatePSOPathfinding(cellInfo.pos, turtlebotId); //Updates path if using PSO
            UpdateAStarPathfinding(cellInfo.pos, turtlebotId); //Update path if using A*
            TurtlebotManager::turtlebots[turtlebotId]->EmptyNewPoint();
        }
    }

    void GetPoints(){
        //Change to numrobots later
        for(int i = 0; i < 1; i++){
            Position newPoint = TurtlebotManager::turtlebots[i]->GetPoint();
            if(newPoint.x != 0 && newPoint.y != 0){
                //Check if the point is another turtlebot
                bool isTurtlebot = false;
                for(int j = 0; j < TurtlebotManager::numRobots; j++){
                    if(i != j){
                        isTurtlebot = superArea.ComparePositions(TurtlebotManager::turtlebots[j]->GetPosition(), newPoint, 0.3);
                    }
                }
                if(isTurtlebot == false){
                    MarkWallPoint(newPoint, i);
                }
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
                    if(cellId != -1) //If the cell has a valid Id
                        markers.CellMarkerUpdate(cellId, Free, cellPos);

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

    //Move a turtlebot
    //Position cellPos = MarkersManager::superArea.GetNearestCellPosition(TurtlebotManager::turtlebots[0]->GetPosition(), Unexplored, false);

    //MarkersManager::DrawMLine(i, cell);  //Goal should be "cell"
    //TurtlebotManager::turtlebots[0]->NewMovement(traverse, cellPos);

    //Start PSO
    Position nearestCell = MarkersManager::superArea.GetNearestCellPosition(TurtlebotManager::turtlebots[0]->GetPosition(), Unexplored, true);
    TurtlebotManager::turtlebots[0]->NewMovement(traverse, nearestCell);

    while (ok())
    {
        MarkersManager::DrawRobotMarkers();
        TurtlebotManager::MoveTurtlebots();
        //MarkersManager::DrawPoints(); //todo create callback function for get points
        MarkersManager::CheckFreeCell();
        MarkersManager::GetPoints(); //Gets a new point from each turtlebot. Uses this point for path planning if its a wall

        ros::spinOnce(); //Spin for callback functions 
        loop_rate.sleep();
    }
    return 0;    



}