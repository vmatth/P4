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
        Position pos0; pos0.x = 7; pos0.y = 9.5;
        Position pos1; pos1.x = 9; pos1.y = 9.5;
        Position pos2; pos2.x = 11; pos2.y = 9.5;

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
}

namespace MarkersManager{
    Markers markers;
    SuperArea superArea(15, 9, 0.4);

    void InitializeMarkers(){
        markers.SetupMarker();
       // markers.SetupRobotMarker();
       // markers.SetupCellMarker();

        ROS_INFO("Markers initialized");   

        //Create the superarea, subarea and cells
       

        ROS_INFO("Super Area Initialized!");
        // usleep(2000000);
        // //Create marker for each cell
        // cout << "Number of sub Areas:" << superArea.GetNumSubAreas() << endl;
        // cout << "Number of cells: " << superArea.GetNumCells() << endl;
        // for(int i = 0; i < superArea.GetNumSubAreas(); i++){
        //     for(int j = 0; j < superArea.GetNumCells(); j++){
        //         usleep(2000); //Small delay between each marker so rviz can follow     
        //         markers.CellMarker(superArea.GetSubArea(i).cells[j].GetPosition());
        //         //cout << "Added cell at: (" << superArea.GetSubArea(i).cells[j].GetPosition().x << ", " << superArea.GetSubArea(i).cells[j].GetPosition().y << ")" << endl;
        //     }
        // }
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

    cout << "Find nearest cell" << endl;
    Position cell = MarkersManager::superArea.GetNearestCell(TurtlebotManager::turtlebots[0]->GetPosition());
    cout << "Nearest cell is at: (" << cell.x << " , " << cell.y << ")" << endl;

    MarkersManager::DrawMLine(0, cell);
    TurtlebotManager::turtlebots[0]->NewMovement(traverse, cell);

    //TurtlebotManager::turtlebots[0]->NewMovement(turn, 90, absolute);
    //TurtlebotManager::turtlebots[0]->NewMovement(turn, 90, relative);
    //TurtlebotManager::turtlebots[0]->NewMovement(turn, 90, absolute);
    /*Position goalPos;
    goalPos.x = 13;
    goalPos.y = 5;
    
    //TurtlebotManager::turtlebots[0]->NewMovement(turn, 90, absolute);
    //goalPos.x = 1;
    //goalPos.y = 6;
    //TurtlebotManager::turtlebots[0]->NewMovement(traverse, goalPos);
    */
    
    while (ok())
    {
        MarkersManager::DrawRobotMarkers();
        TurtlebotManager::MoveTurtlebots();
        MarkersManager::DrawPoints(); //todo create callback function for get points
        
    
        ros::spinOnce(); //Spin for callback functions 
        loop_rate.sleep();
    }


    return 0;
}