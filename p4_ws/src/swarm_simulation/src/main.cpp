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

        Position goalPos;
        goalPos.x = 7;
        goalPos.y = 5;
        turtlebots[0]->MoveToGoal(goalPos);
        Position goalPos2;
        goalPos2.x = 10;
        goalPos2.y = 6;
        turtlebots[1]->MoveToGoal(goalPos2);
        Position goalPos3;
        goalPos3.x = 9;
        goalPos3.y = 8;
        turtlebots[2]->MoveToGoal(goalPos3);

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
            markers.RobotMarker(TurtlebotManager::turtlebots[i]->GetPosition(), TurtlebotManager::turtlebots[i]->GetId());    
        }
    }

    void DrawMLine(){

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

    MarkersManager::DrawMLine();

    while (ok())
    {
        TurtlebotManager::MoveTurtlebots();
        MarkersManager::DrawPoints();
        MarkersManager::DrawRobotMarkers();
    
        ros::spinOnce(); //Spin for callback functions 
        loop_rate.sleep();
    }


    return 0;
}