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

#include <string.h>

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
    Position goalPos; //Saves the point where the robot will move towards

    double yaw;

    double range;
    double angle;

    int id; //Id so we know which turtlebot this is

    bool moving = false; //Moving towards goalPos
    bool turning; //Turning towards 2 deg of the goalPos margin (unused)

    int firstRobotId;

    bool dontTurn = false;

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

    //Checks if a new point is in the robot's trajectory
    void AngleCalc(Position point, double angle, double dist); 

    //Print new point
    Position GetPoint();

    Position GetPosition();

    Position GetStartPos();

    Position GetPrevPosition();

    void calcPrevPos(Position pos);

    int GetId();

    void MoveToGoal(Position goalPos);

    void GoalReached(); //Called when the turtlebot reaches goalPos.

    void Move(); //Moves the robot directly to a "goalPos"

    void ChangeGoal(Position _goalPos); //Change the coordinates of goalPos.

    void PrintPoisition(Position pos);

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

void PrintPosition(Position pos, string text){
    cout << text << "(" << pos.x << ", " << pos.y << ")" << endl;
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

    if(dist != 100){ //The distance is set to 100 when the sensor doesnt hit an object

    // cout << "Calculate Wall!" << endl;
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


        AngleCalc(newPoint, angle, dist);
    }
}

//Checks if the newPoint is in the robot's trajectory
void Turtlebot::AngleCalc(Position point, double angle, double dist){
    if(dontTurn)
        return;
    if(moving == true) //Only check when robot is moving
    {
        if(dist <= 0.5){ //Only check when the wall is close
            cout << "Angle: " << angle << endl;
            if(angle <= 15 && angle >= 0 || angle >=345 && angle <=360) //If the angle is under 10 deg, then the point is near the robot's trajectory
            {   
                cout << "You about to hit something" << endl;

                //Quik Maffs
                //Rotates the robot by -90 degrees (right)
                Position newPos;
                newPos.x = cos((-90 + yaw + angle) * PI / 180.0);
                newPos.y = sin((-90 + yaw + angle) * PI / 180.0);

                PrintPosition(newPos, "New Pos: ");

                //Adds the robot's current pos to newPos
                newPos.x = newPos.x + pos.x;
                newPos.y = newPos.y + pos.y;

                goalPos.x = newPos.x;
                goalPos.y = newPos.y;

                dontTurn = true;

            }else 
            {
                cout << "The coast is clear" << endl;
            }
        }
    }
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
    //ROS_INFO("[%i], (%f, %f)", id, pos.x, pos.y);
}

void Turtlebot::rangeCallback (const std_msgs::Float64MultiArray::ConstPtr& msg){   //Callback function that is called each time range is updated
    
    //msg->data[0] is different if the world has changed.
    //This function automaically finds out which sensor id matches with which each turtlebot.
    //ROS_INFO("Seq: [%d]", msg->header.seq);

    /*if(firstRobotId == 0)  
        firstRobotId = msg->data[0];*/

    //ROS_INFO("ROBOT: [%f], Range: [%f], Angle: [%f]", msg->data[0], msg->data[1], msg->data[2]);

    //Each turtlebot has a specific id. Ex robot 0 has id 107
    if(id == 0 && msg->data[0] == 17){
        //ROS_INFO("ROBOT 0 Id: [%f], Range: [%f], Angle: [%f]", msg->data[0], msg->data[1], msg->data[2]);
        range = msg->data[1];
        angle = msg->data[2];
        CalculateWall(msg->data[1], msg->data[2]);
    }
    else if(id == 1 && msg->data[0] == 1661){
        //ROS_INFO("ROBOT 1 Id: [%f], Range: [%f], Angle: [%f]", msg->data[0], msg->data[1], msg->data[2]);
        CalculateWall(msg->data[1], msg->data[2]);
    }
    else if(id == 2 && msg->data[0] == 3305){
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


    //cout << "Publishing!!!!!" << endl;

    //cmd_vel_pub.publish(cmd_vel_message);
}


//Constructor
Turtlebot::Turtlebot(int _id, Position _startPos){ //Sets up the turtlebot by storing variables and publishing/subscribing to relevant robot topics.
    cout << "_id" << _id << endl;
    
    ROS_INFO("Setting up turlebot %d", _id); 

    //Saves variables to the turtlebot class
    id = _id;
    startPos.x = _startPos.x;
    startPos.y = _startPos.y;
    pos.x = startPos.x;
    pos.y = startPos.y;
    yaw = 0;

    ROS_INFO("Turtlebot spawned at: (%f, %f) | Yaw: [%f]", startPos.x, startPos.y, yaw);

    //Publish & Subscribe
    cmd_vel_pub = n.advertise<Twist>(Topic("/mobile_base/commands/velocity"), 1); //Publishes 
    odom_sub = n.subscribe(Topic("/odom"), 1000, &Turtlebot::odomCallback, this); //Subscribes to turtlebot odometry
    range_sub = n.subscribe("/sensor/range", 1000, &Turtlebot::rangeCallback, this); //Subscribes to sensor range
}

void Turtlebot::MoveToGoal(Position _goalPos){
    goalPos = _goalPos;
    cout << "Started moving turtlebot [" << id << "]" << " from " << "(" << pos.x << ", " << pos.y << ") " " to (" << goalPos.x << ", " << goalPos.y << ")" << endl;
    moving = true; //Allows the Move() function to start

}

void Turtlebot::GoalReached(){
    cout << "The goal has been reached by turtlebot [" << id << "]" << endl;
    moving = false;
}


// Moves directly to a new point (No obstacle pathfinding)
void Turtlebot::Move(){
    //Only move the turtlebot when moving == true
    if(moving){
        double alpha, relativeGoalx, relativeGoaly, gamma, beta;

        relativeGoalx = goalPos.x-pos.x;
        relativeGoaly = goalPos.y-pos.y;

        alpha = atan2(relativeGoaly, relativeGoalx) * 57.2957795;

        //Alpha is always the shortest angle to the goal. E.g: 270 deg = -90
        //This is a problem when the goal is 180 deg, as it switches from 180 deg to -180 deg

        //If the alpha is negative (E.g -185), find the positive alpha (E.g 175) and check which is closets to the robot's current yaw
        if(alpha < 0){
            //cout << "----------------" << endl;
            //cout << "Negative alpha: " << alpha << endl;
            double positiveAlpha = alpha + 360;
            //cout << "Positive alpha: " << positiveAlpha << endl;
            //cout << "Current Yaw:" << yaw << endl;
        
                double positiveDifference = abs(positiveAlpha - yaw);
                double negativeDifference = abs(alpha - yaw);
                //cout << "Positive Dif:" << positiveDifference << endl;
                //cout << "Negative Dif:" << negativeDifference << endl;
                if(positiveDifference < negativeDifference){
                    alpha = positiveAlpha; //Swap negative alpha with positive
                }
        }

        gamma = alpha - yaw;

        // cout << "---------------" << endl;

        // cout << "MoveToGoal Pos: ("  << goalPos.x << ", " << goalPos.y << ")" << endl;

        // cout << "RobotPos: ("  << pos.x << ", " << pos.y << ")" << endl;

        // cout << "Alpha (Angle to point): " << alpha << endl; //Vinklen fra (0,0) til punktet

        // cout << "Robot Yaw: " << yaw << endl; //Robottens vinkel

        // cout << "Gamma (Angles to turn): " << gamma << endl; //forskel i vinklerne

        // cout << "Gamma (Rads): " << (gamma/180 * PI) << endl; //forskel i vinklerne

        cmd_vel_message.angular.z = gamma/180 * PI;

        //If the robots angle is in the margin (Rotated correctly)
        if(gamma < 2 && gamma > -2){
            Position posDifference; 
            posDifference.x = goalPos.x - pos.x;
            posDifference.y = goalPos.y - pos.y;
            //cout << "PosDiffernce: (" << posDifference.x << ", " << posDifference.y << ")" << endl;
            //cout << "PosDiff Abosulte: (" << abs(posDifference.x) << ", " << abs(posDifference.y) << ")" << endl;
            //If the robot isn't near the goalPos: Keep moving
            if((abs(posDifference.x) > 0.1) || (abs(posDifference.y) > 0.1)){
                cmd_vel_message.linear.x = 0.3;
            }
            //If the robot has reached the goalPos
            else{
                cmd_vel_message.linear.x = 0;
                GoalReached();
            }
        }
        //The robot is not rotated correctly yet - Dont move
        else{
            cmd_vel_message.linear.x = 0;
        }

        //Publish linear and rotation 
        cmd_vel_pub.publish(cmd_vel_message);
    }
}

void Turtlebot::ChangeGoal(Position _goalPos)
{
    
}