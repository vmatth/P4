#include <iostream>
#include <string>
#include <vector>
#include <utility>
#include <memory>
#include <math.h>  

#include <ros/ros.h>

#include <geometry_msgs/Twist.h>
#include "nav_msgs/Odometry.h"

#include "std_msgs/Float64.h"
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Float64MultiArray.h"

#include <tf/transform_datatypes.h>

#define PI 3.14159265

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
    Subscriber odom_sub, range_sub;
    Twist cmd_vel_message;

    Position pos; //The robot's current absolute position
    Position startPos; //The robot's start position

    double yaw;

    double range;
    double angle;

    int id; //Id so we know which turtlebot this is
public:

    //Callback function that is called each time odometry is updated
    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg){
        //ROS_INFO("Seq: [%d]", msg->header.seq);
       // ROS_INFO("Odometry for turtlebot %d", id);
        //ROS_INFO("Position-> x: [%f], y: [%f], z: [%f]", msg->pose.pose.position.x,msg->pose.pose.position.y, msg->pose.pose.position.z);
        //ROS_INFO("Orientation-> x: [%f], y: [%f], z: [%f], w: [%f]", msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
        //ROS_INFO("Vel-> Linear: [%f], Angular: [%f]", msg->twist.twist.linear.x,msg->twist.twist.angular.z);
        
        //Update the robot position
        UpdatePos(msg->pose.pose.position.x, msg->pose.pose.position.y);

        UpdateRotation(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
    }

    //Callback function that is called each time range is updated
    void rangeCallback(const std_msgs::Float64MultiArray::ConstPtr& msg){
        //ROS_INFO("Seq: [%d]", msg->header.seq);

        //Each turtlebot has a specific id. Ex robot 0 has id 107
        if(id == 0 && msg->data[0] == 107){
            //ROS_INFO("ROBOT 0 Id: [%f], Range: [%f], Angle: [%f]", msg->data[0], msg->data[1], msg->data[2]);
            range = msg->data[1];
            angle = msg->data[2];
            CalculateWall(msg->data[1], msg->data[2]);
        }
        else if(id == 1 && msg->data[0] == 1751){
            //ROS_INFO("ROBOT 1 Id: [%f], Range: [%f], Angle: [%f]", msg->data[0], msg->data[1], msg->data[2]);
            CalculateWall(msg->data[1], msg->data[2]);
        }
        else if(id == 2 && msg->data[0] == 3395){
            //ROS_INFO("ROBOT 2 Id: [%f], Range: [%f], Angle: [%f]", msg->data[0], msg->data[1], msg->data[2]);
            CalculateWall(msg->data[1], msg->data[2]);
        }



        
    }

    //Publish a velocity
    void Publish(){
        if(yaw <= 90){
            cmd_vel_message.angular.z = 0.4;
        }
        else{
            cmd_vel_message.angular.z = 0;
        }

        if(range >= 0.5 && yaw >= 90){
            cmd_vel_message.linear.x = 0.3;
        }
        else{
            cmd_vel_message.linear.x = 0;
        }

        
        //cout << "Publishing!!!!!" << endl;
        
        //cmd_vel_message.linear.x = 1.0;
        cmd_vel_pub.publish(cmd_vel_message);
    }

    //Automatically create the topic name using the turtlebot id
    string Topic(string topic){
        stringstream sstm; //Stringstream allows to add variables to text
        sstm << "/turtlebot" << id << topic; //Creates a topic with the turtlebot id
        cout << "Topic: " << sstm.str() << endl;
        return sstm.str();
    }

    //Updates to robot pos by adding Odometry Position to startPos.
    void UpdatePos(double xOdom, double yOdom){
        pos.x = startPos.x + xOdom;
        pos.y = startPos.y + yOdom;
    }

    //Updates robot rotation using odometry values.
    void UpdateRotation(double x, double y, double z, double w){
        //https://answers.ros.org/question/50113/transform-quaternion/
        //https://gist.github.com/marcoarruda/f931232fe3490b7fa20dbb38da1195ac
       
        //Converts from quaternion to degrees
        tf::Quaternion q(x, y, z, w);
        tf::Matrix3x3 m(q);
        double roll, pitch;
        m.getRPY(roll, pitch, yaw);

        yaw = yaw * 180/PI;
        /*if(id == 0)
            std::cout << "Yaw: " << yaw << std::endl;*/
    }

    //Calculates a point where the nearest wall is located.
    void CalculateWall(double dist, double angle){
        //Calculate wall pos relative to the robot.
        cout << "Point measured for ROBOT " << id << endl;
        double xRelative = cos(angle * PI / 180.0) * dist;
        double yRelative = sin(angle * PI / 180.0) * dist;
        cout << "xRelative: " << xRelative << ", yRelative: " << yRelative << endl;
        //Calculate global wall pos using the robots rotation.
        //Calculate wall pos relative to the robot.
        double x = cos((angle + yaw) * PI / 180.0) * dist + pos.x;
        double y = sin((angle + yaw) * PI / 180.0) * dist + pos.y;
        cout << "xGlobal: " << x << ", yGlobal: " << y << endl;
    }

    //Constructor
    Turtlebot(int _id, Position _startPos){ //Sets up the turtlebot by storing variables and publishing/subscribing to relevant robot topics.
        ROS_INFO("Setting up turlebot %d", id); 

        //Saves variables to the turtlebot class
        id = _id;
        startPos.x = _startPos.x;
        startPos.y = _startPos.y;

        //Publish & Subscribe
        cmd_vel_pub = n.advertise<Twist>(Topic("/mobile_base/commands/velocity"), 1); //Publishes 
        odom_sub = n.subscribe(Topic("/odom"), 1000, &Turtlebot::odomCallback, this); //Subscribes to turtlebot odometry
        range_sub = n.subscribe("/sensor/range", 1000, &Turtlebot::rangeCallback, this); //Subscribes to sensor range
    }


};

//Variables and functions for initializing the turtlebot swarm
namespace TurtlebotManager{

    int numRobots = 3;

    vector<Position> robotStartPositions;

    vector<unique_ptr<Turtlebot>> turtlebots;

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