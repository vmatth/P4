#include <iostream>
#include <string>
#include <vector>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>


using namespace std;
using namespace ros;
using namespace geometry_msgs;


class Turtlebot{

private:
    NodeHandle n;
    Publisher cmd_vel_pub;
    Twist cmd_vel_message;

public:

     Turtlebot(int id){ //Sets up the turtlebot by publishing and subscribing to the robot topics.
        cout << "Setting up turlebot " << id << endl;
    
        stringstream sstm;
        sstm << "/turtlebot" << id << "/mobile_base/commands/velocity"; //Adds the turtlebot id to the path
        string turtlePublisher = sstm.str();

        cmd_vel_pub = n.advertise<Twist>(turtlePublisher, 1); //Publishes     
    }

    void Publish(){
        cout << "Publishing!!!!!" << endl;
        cmd_vel_message.angular.z = 1.0;
        cmd_vel_message.linear.x = 1.0;
        cmd_vel_pub.publish(cmd_vel_message);
    }

};

//Variables and functions for initializing the turtlebot swarm
namespace TurtlebotManager{
    int numRobots = 2;
    vector<Turtlebot> turtlebots;

    void InitializeTurtlebots(){ //Initializes a specified amount of turtlebots for the swarm
        for(int i = 0; i < numRobots; i++){
            Turtlebot t(i); //Create new turtlebot class and set its id
            turtlebots.push_back(t); //Push new turtlebot to the vector
        }
        cout << "Turtlebots initialized!" << endl;
    }

    void MoveTurtlebots(){
        for (int i = 0; i < numRobots; i++){ //Foreach turtlebot in the turtlebot vector
            turtlebots[i].Publish();
        }
    }
}

int main(int argc, char *argv[])
{

    init(argc, argv, "Multi_Robot_Test");

    cout << "Hello World" << endl;

    TurtlebotManager::InitializeTurtlebots();

    //Main loop
    Rate loop_rate(10);
    while (ok())
    {
        TurtlebotManager::MoveTurtlebots();
        loop_rate.sleep();
    }


    return 0;
}