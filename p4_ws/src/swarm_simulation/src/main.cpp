#include <iostream>
#include <string>
#include <vector>
#include <utility>
#include <memory>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include "nav_msgs/Odometry.h"


using namespace std;
using namespace ros;
using namespace geometry_msgs;

struct Position{
    double x;
    double y;
};

class Turtlebot{

private:

    //Ros variables
    NodeHandle n;
    Publisher cmd_vel_pub;
    Subscriber odom_sub;
    Twist cmd_vel_message;

    Position pos; //The robot's current absolute position
    Position startPos; //The robot's start position

    int id; //Id so we know which turtlebot this is
public:

    //Callback function that is called each time odometry is updated
    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg){
        //ROS_INFO("Seq: [%d]", msg->header.seq);
        ROS_INFO("Odometry for turtlebot %d", id);
        ROS_INFO("Position-> x: [%f], y: [%f], z: [%f]", msg->pose.pose.position.x,msg->pose.pose.position.y, msg->pose.pose.position.z);
        //ROS_INFO("Orientation-> x: [%f], y: [%f], z: [%f], w: [%f]", msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
        //ROS_INFO("Vel-> Linear: [%f], Angular: [%f]", msg->twist.twist.linear.x,msg->twist.twist.angular.z);
        
        //Update the robot position
        UpdatePos(msg->pose.pose.position.x, msg->pose.pose.position.y);
    }

    Turtlebot(int _id, Position _startPos){ //Sets up the turtlebot by storing variables and publishing/subscribing to relevant robot topics.
        ROS_INFO("Setting up turlebot %d", id); 

        //Setup variables
        id = _id;
        startPos.x = _startPos.x;
        startPos.y = _startPos.y;

        //Publish & Subscribe
        cmd_vel_pub = n.advertise<Twist>(Topic("/mobile_base/commands/velocity"), 1); //Publishes 
        odom_sub = n.subscribe(Topic("/odom"), 1000, &Turtlebot::odomCallback, this); //Subscribes to turtlebot odometry
    }

    void Publish(){
        //cout << "Publishing!!!!!" << endl;
       // cmd_vel_message.angular.z = 1.0;
        cmd_vel_message.linear.x = 1.0;
        cmd_vel_pub.publish(cmd_vel_message);
    }

    //Automatically create the topic name using the turtlebot id
    string Topic(string topic){
        stringstream sstm; //Stringstream allows to add variables to text
        sstm << "/turtlebot" << id << topic; //Creates a topic with the turtlebot id
        cout << "Topic: " << sstm.str() << endl;
        return sstm.str();
    }

    void UpdatePos(double xOdom, double yOdom){
        pos.x = startPos.x + xOdom;
        pos.y = startPos.y + yOdom;
    }


};

//Variables and functions for initializing the turtlebot swarm
namespace TurtlebotManager{

    int numRobots = 2;

    vector<unique_ptr<Turtlebot>> turtlebots;

    //vector<Turtlebot> turtlebots;

    void InitializeTurtlebots(){ //Initializes a specified amount of turtlebots for the swarm

        //Loop number of wanted turtlebots, and initialize a new turtlebot class using unique_ptr. 
        //The turtlebots are stored in the "turtlebots" vector.
        for(int i = 0; i < numRobots; i++){
            Position newRobotPos;
            newRobotPos.x = 0;
            newRobotPos.y = 0;
            unique_ptr<Turtlebot> turtlebot(new Turtlebot(i, newRobotPos)); //Create new turtlebot class with id as argument.
            turtlebots.push_back(move(turtlebot));
        }

        ROS_INFO("Turtlebots initialized!");
    }

    void MoveTurtlebots(){

        turtlebots[0]->Publish();

        for (int i = 0; i < numRobots; i++){ //Foreach turtlebot in the turtlebot vector
            //turtlebots[i]->Publish();
        }
    }
}

int main(int argc, char *argv[])
{

    init(argc, argv, "Multi_Robot_Test");

    ROS_INFO("Hello World");

    TurtlebotManager::InitializeTurtlebots();
    //Main loop
    Rate loop_rate(10);
    while (ok())
    {
        TurtlebotManager::MoveTurtlebots();
        ros::spinOnce(); //Spin for callback functions 
        loop_rate.sleep();
    }


    return 0;
}