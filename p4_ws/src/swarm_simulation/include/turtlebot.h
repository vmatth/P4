#include <iostream>
#include <string>
#include <vector>
#include <utility>
#include <memory>
#include <math.h>  

#include <ros/ros.h>

#include <position.h>

#include <geometry_msgs/Twist.h>
#include "nav_msgs/Odometry.h"
#include <tf/transform_datatypes.h>
#include "std_msgs/Float64MultiArray.h"

using namespace std;
using namespace ros;
using namespace geometry_msgs;

#define PI 3.14159265

class Turtlebot{

private:

    //Ros variables
    NodeHandle n;
    Publisher cmd_vel_pub;
    Subscriber odom_sub, range_sub;
    Twist cmd_vel_message;

    Position pos; //The robot's current absolute position
    Position startPos; //The robot's start position
    Position prevPos;

    Position newPoint;

    double yaw;

    double range;
    double angle;

    int id; //Id so we know which turtlebot this is

public:

    //Callback function that is called each time odometry is updated
    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);

    //Callback function that is called each time range is updated
    void rangeCallback(const std_msgs::Float64MultiArray::ConstPtr& msg);

    //Publish a velocity
    void Publish();

    //Automatically create the topic name using the turtlebot id
    string Topic(string topic);

    //Updates to robot pos by adding Odometry Position to startPos.
    void UpdatePos(double xOdom, double yOdom);

    //Updates robot rotation using odometry values.
    void UpdateRotation(double x, double y, double z, double w);

    //Calculates a point where the nearest wall is located.
    void CalculateWall(double dist, double angle);

    //Print new point
    Position GetPoint();

    Position GetPosition();

    Position GetStartPos();

    Position GetPrevPosition();

    void calcPrevPos(Position pos);

    int GetId();

    void MoveToGoal(Position goalPos);

    //Constructor
    Turtlebot(int _id, Position _startPos); //Sets up the turtlebot by storing variables and publishing/subscribing to relevant robot topics.

};


Position Turtlebot::GetPoint(){
    return newPoint;
}

Position Turtlebot::GetPosition(){
    return pos;
}

Position Turtlebot::GetStartPos()
{
    return startPos;
}

Position Turtlebot::GetPrevPosition()
{
    return prevPos;
}

void Turtlebot::calcPrevPos(Position pos)
{
    prevPos = pos;
}

int Turtlebot::GetId()
{
    return id;
}

//Updates to robot pos by adding Odometry Position to startPos.
void Turtlebot::UpdatePos(double xOdom, double yOdom){
    pos.x = startPos.x + xOdom;
    pos.y = startPos.y + yOdom;
}

//Updates robot rotation using odometry values.
void Turtlebot::UpdateRotation(double x, double y, double z, double w){
    //https://answers.ros.org/question/50113/transform-quaternion/
    //https://gist.github.com/marcoarruda/f931232fe3490b7fa20dbb38da1195ac
    
    //Converts from quaternion to degrees
    tf::Quaternion q(x, y, z, w);
    tf::Matrix3x3 m(q);
    double roll, pitch;
    m.getRPY(roll, pitch, yaw);

    yaw = yaw * 180/PI;
}

//Calculates a point where the nearest wall is located.
void Turtlebot::CalculateWall(double dist, double angle){
    //Calculate wall pos relative to the robot.
    //cout << "Point measured for ROBOT " << id << endl;
    double xRelative = cos(angle * PI / 180.0) * dist;
    double yRelative = sin(angle * PI / 180.0) * dist;
    //cout << "xRelative: " << xRelative << ", yRelative: " << yRelative << endl;
    //Calculate global wall pos using the robots rotation.
    //Calculate wall pos relative to the robot.
    double x = cos((angle + yaw) * PI / 180.0) * dist + pos.x;
    double y = sin((angle + yaw) * PI / 180.0) * dist + pos.y;
   // cout << "xGlobal: " << x << ", yGlobal: " << y << endl;


   newPoint.x = x;
   newPoint.y = y;
}


void Turtlebot::odomCallback(const nav_msgs::Odometry::ConstPtr& msg){
    //ROS_INFO("Seq: [%d]", msg->header.seq);
    // ROS_INFO("Odometry for turtlebot %d", id);
    //ROS_INFO("Position-> x: [%f], y: [%f], z: [%f]", msg->pose.pose.position.x,msg->pose.pose.position.y, msg->pose.pose.position.z);
    //ROS_INFO("Orientation-> x: [%f], y: [%f], z: [%f], w: [%f]", msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
    //ROS_INFO("Vel-> Linear: [%f], Angular: [%f]", msg->twist.twist.linear.x,msg->twist.twist.angular.z);
    
    //Update the robot position
    UpdatePos(msg->pose.pose.position.x, msg->pose.pose.position.y);

    UpdateRotation(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
}

void Turtlebot::rangeCallback (const std_msgs::Float64MultiArray::ConstPtr& msg){   //Callback function that is called each time range is updated
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


//Automatically create the topic name using the turtlebot id
string Turtlebot::Topic(string topic){
    stringstream sstm; //Stringstream allows to add variables to text
    sstm << "/turtlebot" << id << topic; //Creates a topic with the turtlebot id
    cout << "Topic: " << sstm.str() << endl;
    return sstm.str();
}

void Turtlebot::Publish(){
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


//Constructor
Turtlebot::Turtlebot(int _id, Position _startPos){ //Sets up the turtlebot by storing variables and publishing/subscribing to relevant robot topics.
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

// move to goal
void Turtlebot::MoveToGoal(Position goalPos){
    double alpha, relativeGoalx, relativeGoaly, gamma, beta;

    double RobotX = pos.x;
    double RobotY = pos.y;

    double GoalX = goalPos.x;
    double GoalY = goalPos.y;

    
    relativeGoalx = GoalX-RobotX;
    relativeGoaly = GoalY-RobotY;

    alpha = atan2(relativeGoaly, relativeGoalx) * 57.2957795;

    //yaw = 10;
    gamma = alpha - yaw;

    cout << "MoveToGoal Pos: ("  << goalPos.x << ", " << goalPos.y << ")" << endl;

    cout << "RobotPos: ("  << pos.x << ", " << pos.y << ")" << endl;

    cout << "alpha: " << alpha << endl;

    cout << "Yaw: " << yaw << endl;

    cout << "Gamma: " << gamma << endl;


    if (gamma <0){
        if(yaw >= gamma){
        cmd_vel_message.angular.z = -0.4;
        }
        else{
            cmd_vel_message.angular.z = 0;
        }
    }else if(gamma >=0){
        if(yaw <= gamma){
        cmd_vel_message.angular.z = 0.4;
        }
        else{
            cmd_vel_message.angular.z = 0;
        }
    }
    

    //cout << "Publishing!!!!!" << endl;

    //cmd_vel_message.linear.x = 1.0;
    cmd_vel_pub.publish(cmd_vel_message);
}

