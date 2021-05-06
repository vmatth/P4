#include <ros/ros.h>

#include <turtlebot.h>

#include <grid.h>

#include <markers.h>
#include <unistd.h>
#include <aStar.h>

using namespace std;
using namespace ros;

//Variables and functions for initializing the turtlebot swarm
namespace TurtlebotManager{
    SuperArea superArea(12, 4, 0.4); //Prev: 15, 9, 0.4

    int numRobots = 3;

    vector<Position> robotStartPositions;

    vector<std::unique_ptr<Turtlebot>> turtlebots;

    void InitializeTurtlebots(){ //Initializes a specified amount of turtlebots for the swarm

        //Specify the robot start position
        Position pos0; pos0.x = 6; pos0.y = 8;
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
    //     Position min;
    //     Position max;
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

    int CheckFreeArea(){
        for(int i = 0; i < numRobots; i++){
            for(int i = 0; i < superArea.GetNumSubAreas(); i++){
                if(robot position is not in subarea){
                    return i; 
                }
            }
        }
    }
}

namespace MarkersManager{
    Markers markers;
    SuperArea superArea(12, 4, 0.4); //Prev: 15, 9, 0.4


    void DrawAllCells(){
        usleep(2000000);
        //Create marker for each cell
        cout << "Number of sub Areas:" << superArea.GetNumSubAreas() << endl;
        cout << "Number of cells: " << superArea.GetNumCells() << endl;
        for(int i = 0; i < superArea.GetNumSubAreas(); i++){
            for(int j = 0; j < superArea.GetNumCells(); j++){
                usleep(2000); //Small delay between each marker so rviz can follow     
                markers.CellMarker(superArea.GetSubArea(i).cells[j].GetPosition(), Unexplored);
                //cout << "Added cell at: (" << superArea.GetSubArea(i).cells[j].GetPosition().x << ", " << superArea.GetSubArea(i).cells[j].GetPosition().y << ")" << endl;
            }
        }
    }

    void InitializeMarkers(){
        markers.SetupMarker();
        markers.SetupRobotMarker();
        markers.SetupCellMarker();
        markers.SetupMlineMarker();

        ROS_INFO("Markers initialized");   

        //Create the superarea, subarea and cells
       

        ROS_INFO("Super Area Initialized!");
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

    void CheckFreeCell()
    {
        for(int i = 0; i < TurtlebotManager::numRobots; i++)
        {
            Position cellPos = TurtlebotManager::turtlebots[i]->GetfreeCell();
            if(cellPos.x != 0 && cellPos.y != 0){
                cout << "Found new cell pos at (" << cellPos.x << ", " << cellPos.y << ")" << endl;
                //Call marker function to change color
                markers.CellMarker(cellPos, Free);
                TurtlebotManager::turtlebots[i]->EmptyfreeCell();
                //Changes the cell state to Free
                superArea.MarkCell(cellPos, Free);
                //Find new cell for turtlebot
                Position newCell = superArea.GetNextCell(cellPos, TurtlebotManager::turtlebots[i]->GetRotation());
                cout << "Robot [" << i << "]" << "'s next pos: (" << newCell.x << "," << newCell.y << ")" << endl;
                cout << "----------------------------------" << endl;   

                //cout << "Input key to go to resume" << endl;     
                //char x;

                //cin >> x;  

                TurtlebotManager::turtlebots[i]->NewMovement(traverse, newCell);
                
            }
        }
    }

    //When the turtlebot receives a new point, it will mark the nearby cells as "Wall" or check if its a turtlebot
    void MarkPoints(){
        //loop alle turtlebots og kald funktionen markcells
        //for (int i = 0; i < TurtlebotManager::numRobots; i++){
        for(int i = 0; i < 1; i++){
            Position newPoint = TurtlebotManager::turtlebots[i]->GetPoint();
            if(newPoint.x != 0 && newPoint.y != 0){
                cout << "check if new point is a turtlebot" << endl;
                //Check if a new point is not another turtlebot
                //Loop all turtlebots
                bool pointIsTurtlebot = false;
                for(int j = 0; j < TurtlebotManager::numRobots; j++){
                    //Dont check its own point
                    if(i != j){
                        cout << "Checking bot: " << j << endl;
                        Position relativePos;
                        relativePos.x = newPoint.x - TurtlebotManager::turtlebots[j]->GetPosition().x;
                        relativePos.y = newPoint.y - TurtlebotManager::turtlebots[j]->GetPosition().y;

                        TurtlebotManager::turtlebots[i]->PrintPosition(relativePos, "Relative pos ");
                        
                        if(abs(relativePos.x) < 0.3 && abs(relativePos.y) < 0.3)
                        {
                            cout << "Point measured is turtlbot [" << j << "]" << endl;
                            pointIsTurtlebot = true;
                        }   
                    }                 
                }

                if(pointIsTurtlebot == false){
                    //Changes the state of the cell to wall
                    CellInfo cellInfo = superArea.MarkWallCells(newPoint, TurtlebotManager::turtlebots[i]->GetPosition());
                    //Updates the rviz point to wall
                    markers.CellMarkerUpdate(cellInfo.id, Wall, cellInfo.pos);

                    bool collision = superArea.CheckForCollision(cellInfo.pos, TurtlebotManager::turtlebots[i]->GetGoalPos());
                    if(collision == true){
                        cout << "The robot's goal position will collide with a wall" << endl;
                        //Stop the current movement
                        TurtlebotManager::turtlebots[i]->EmptyList();
                        //Move back the the previous cell
                        TurtlebotManager::turtlebots[i]->PrintPosition(TurtlebotManager::turtlebots[i]->GetPrevPosition(), "Turtlebot will move back to: ");
                        //TurtlebotManager::turtlebots[i]->NewMovement(traverse, TurtlebotManager::turtlebots[i]->GetPrevPosition());
                        TurtlebotManager::turtlebots[i]->NewMovement(traverse, superArea.GetNearestCell(TurtlebotManager::turtlebots[i]->GetPosition(), Free));
                    }
                                    
                    TurtlebotManager::turtlebots[i]->EmptyNewPoint();
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

    //Main loop
    Rate loop_rate(10);
//    markers_sub();

    MarkersManager::InitializeMarkers();
    loop_rate.sleep();

    MarkersManager::superArea.NewGrid(20, 4, 0.4);
