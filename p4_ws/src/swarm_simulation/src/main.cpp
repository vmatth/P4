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

    void InitializeMarkers(){
        markers.SetupMarker();
       // markers.SetupRobotMarker();
       // markers.SetupCellMarker();

        ROS_INFO("Markers initialized");   

        //Create the superarea, subarea and cells
        SuperArea superArea(20, 9, 0.2);

        ROS_INFO("Super Area Initialized!");


       /* sleep(7);

        //Create marker for each cell
        cout << "Number of sub Areas:" << superArea.GetNumSubAreas() << endl;
        cout << "Number of cells: " << superArea.GetNumCells() << endl;
        int oooo = 0;
        for(int i = 0; i < superArea.GetNumSubAreas(); i++){
            for(int j = 0; j < superArea.GetNumCells(); j++){
                usleep(2000); //Small delay between each marker so rviz can follow     
                markers.CellMarker(superArea.GetSubArea(i).cells[j].GetPosition());
                cout << "Added cell at: (" << superArea.GetSubArea(i).cells[j].GetPosition().x << ", " << superArea.GetSubArea(i).cells[j].GetPosition().y << ")" << endl;
            }
            
        }*/
    }

    void DrawPoints(){
        for(int i = 0; i < TurtlebotManager::numRobots; i++){
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

    void DrawMLine(){
        Position startPos, goalPos;
        startPos.x = 1; startPos.y = 1;
        markers.MLine(startPos, goalPos); //Remember to change start and goal pos
        cout << "Drawing MLine" << endl;
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
    MarkersManager::DrawMLine();

    Position newPos;
    newPos.x = 6;
    newPos.y = 0;
    TurtlebotManager::turtlebots[0]->MoveToGoal(newPos);
    
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