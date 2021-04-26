#include <ros/ros.h>

#include <turtlebot.h>

#include <markers.h>

using namespace std;
using namespace ros;

//Variables and functions for initializing the turtlebot swarm
namespace TurtlebotManager{

    int numRobots = 3;

    vector<Position> robotStartPositions;

    vector<std::unique_ptr<Turtlebot>> turtlebots;

    Markers markers;

    void InitializeTurtlebots(){ //Initializes a specified amount of turtlebots for the swarm

        //Specify the robot start position
        Position pos0; pos0.x = 0; pos0.y = 0;
        Position pos1; pos1.x = 2; pos1.y = 0;
        Position pos2; pos2.x = 4; pos2.y = 0;

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

        markers.SetupMarker();
        markers.SetupRobotMarker();

        ROS_INFO("Markers initialized");
    }

    void MoveTurtlebots(){

        Position goalPos;
        goalPos.x = -5;
        goalPos.y = -10;
        turtlebots[0]->MoveToGoal(goalPos);

        for (int i = 0; i < numRobots; i++){ //Foreach turtlebot in the turtlebot vector
            //turtlebots[i]->Publish();
        }
    }
    void GetPoints(){
        for(int i = 0; i < numRobots; i++){
            markers.NewMarker(turtlebots[i]->GetPoint());
        }
    }
    int GetRobotId()
    {
        for(int i = 0; i < numRobots; i++)
        {
            turtlebots[i];
        }
    }
    void RobotMarker()
    {
        for(int i = 0; i < numRobots; i++)
        {
            markers.robotMarker(turtlebots[i]->GetPosition(), turtlebots[i]->GetId());    
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
    while (ok())
    {

        TurtlebotManager::MoveTurtlebots();
        TurtlebotManager::GetPoints();
        TurtlebotManager::RobotMarker();
    
        ros::spinOnce(); //Spin for callback functions 
        loop_rate.sleep();
    }


    return 0;
}