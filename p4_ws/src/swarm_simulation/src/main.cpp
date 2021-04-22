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

class Turtlebot{

private:

    NodeHandle n;
    Publisher cmd_vel_pub;
    Subscriber odom_sub;
    Twist cmd_vel_message;

    int id; //Id so we know which turtlebot this is
public:

    //Callback function that is called each time odometry is updated
    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg){
        //ROS_INFO("Seq: [%d]", msg->header.seq);
        ROS_INFO("Odometry for turtlebot %d", id);
        ROS_INFO("Position-> x: [%f], y: [%f], z: [%f]", msg->pose.pose.position.x,msg->pose.pose.position.y, msg->pose.pose.position.z);
        //ROS_INFO("Orientation-> x: [%f], y: [%f], z: [%f], w: [%f]", msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
        //ROS_INFO("Vel-> Linear: [%f], Angular: [%f]", msg->twist.twist.linear.x,msg->twist.twist.angular.z);
    }

    Turtlebot(int _id){ //Sets up the turtlebot by publishing and subscribing to the robot topics.
        id = _id;
        ROS_INFO("Setting up turlebot %d", id);

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
            unique_ptr<Turtlebot> turtlebot(new Turtlebot(i)); //Create new turtlebot class with id as argument.
            turtlebots.push_back(move(turtlebot));
        }



       /* for(int i = 0; i < numRobots; i++){
            Turtlebot t(i); //Create new turtlebot class and set its id
            turtlebots.push_back(t); //Push new turtlebot to the vector
        }*/
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
        loop_rate.sleep();
        ros::spinOnce(); //Spin for callback functions 
    }


    return 0;
}