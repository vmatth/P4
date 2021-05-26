
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
#include <states.h>

#include "visualization_msgs/Marker.h"

using namespace std;
using namespace ros;
using namespace geometry_msgs;

#define PI 3.14159265


//Contains information about how the robot will move. (E.g turn in place or move towards a goal Position)
struct Movement{
    MovementType movementType;
    Position goalPos;
    double goalRotation;
    TurnType turnType;
    bool fixRelative; //Used to add + yaw to goalRotation, so it is relative. Must only be added once, so this bool tracks it.
    int lineId; //rViz line id
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

    Position newPoint; //Stores a new wall point
    Position prevPos; //Saves the point where the robot moved last
    Position pathfindPos; //Saves the point that the robot is using A* to

    double yaw, goalYaw;

    double range, angle;

    int id; //Id so we know which turtlebot this is

    int firstRobotId;

    MovementState movementState;

    MovementState nextState; //State after finishing a turn

    list<Movement> movements; //A list of all the movements the turtlebot will perform. After finished one movement, the next will be performed.

    bool turningRobot = false;
    bool followWall = false;

    Position freeCell;  //The last free cell explored by the turtebot

    bool pathfinding = false; //If the robot is using A*. If yes, it will not ignore the "front right left" check everytime it reaches a new cell.
    //If false it will find the next cell after it reaches another cell.

    bool avoidingMaster = false; //If the robot is avoiding collision with another robot!
    bool avoidingSlave = false; //Slave: The robot stands still with temp walls around it

    double speed = 0.3;
    double rotationSpeed = 1;

    bool forcePathfind = false;

    bool pathfindingToStart = false;

    uint64_t LinePointId = 0;
    uint64_t LineLineId = 0;
    Publisher LineMarker_pub;

public:

    bool finished = false;

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

    Position GetfreeCell();

    Position GetGoalPos();

    list<Position> GetMovements();

    bool GetPathfinding();

    double GetRotation();

    void calcPrevPos(Position pos);

    int GetId();

    void MoveToGoal(Position goalPos);

    void GoalReached(Position); //Called when the turtlebot reaches goalPos.

    void GoalYawReached();

    void Move(); //Moves the robot directly to a "goalPos"

    void PrintPosition(Position pos, string);

    //Creates a new Movement struct with given variables. The movement is pushed into the "movements" list
    void NewMovement(MovementType, Position);

    void NewMovement(MovementType, double, TurnType);

    //Constructor
    Turtlebot(int _id, Position _startPos); //Sets up the turtlebot by storing variables and publishing/subscribing to relevant robot topics.

    void EmptyList();

    void EmptyfreeCell();

    void EmptyNewPoint();

    void SetPathfinding(bool);

    void SetPathfindingPoint(Position);

    int GetMovementsSize();

    Position GetPathfindingPoint();

    void SetAvoidingMaster(bool);

    bool GetAvoidingMaster();

    void SetAvoidingSlave(bool);

    bool GetAvoidingSlave();    

    void PauseMovement(); //Stops the movement by setting speed to 0.

    void ResumeMovement(); //Resumes by reverting the speed

    void SetForcePathfind(bool _value){
        forcePathfind = _value;
    }

    bool GetForcePathfind(){
        return forcePathfind;
    }

    void Line(Position, Position);

    void RemoveLine(int);

    void SetupLineMarker();

    void SetPathfindingStartPos(bool);

    bool GetPathfindingStartPos();

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

Position Turtlebot::GetfreeCell()
{
    return freeCell;
}

double Turtlebot::GetRotation()
{
    return yaw;
}

Position Turtlebot::GetGoalPos(){
    return movements.front().goalPos;
}

bool Turtlebot::GetPathfinding(){
    return pathfinding;
}

void Turtlebot::SetPathfindingPoint(Position pos){
    pathfindPos = pos;
}

void Turtlebot::SetPathfinding(bool _value){
    pathfinding = _value;
}

Position Turtlebot::GetPathfindingPoint(){
    return pathfindPos;
}

list<Position> Turtlebot::GetMovements(){
    list<Position> positions; //List to return later
    for(auto const& m : movements){
        positions.push_back(m.goalPos);
    }
    return positions;
}

void Turtlebot::calcPrevPos(Position pos)
{
    prevPos = pos;
}

int Turtlebot::GetId()
{
    return id;
}

int Turtlebot::GetMovementsSize(){
    return movements.size();
}

void Turtlebot::SetAvoidingMaster(bool _avoid){
    avoidingMaster = _avoid;
}

bool Turtlebot::GetAvoidingMaster(){
    return avoidingMaster;
}

void Turtlebot::SetAvoidingSlave(bool _avoid){
    avoidingSlave = _avoid;
}

bool Turtlebot::GetAvoidingSlave(){
    return avoidingSlave;
}

void Turtlebot::PrintPosition(Position pos, string text){
    cout << text << "(" << pos.x << ", " << pos.y << ")" << endl;
}

void Turtlebot::PauseMovement(){
    speed = 0;
    rotationSpeed = 0;
}

void Turtlebot::ResumeMovement(){
    speed = 0.3;
    rotationSpeed = 1;
}

//Updates to robot pos by adding Odometry Position to startPos.
void Turtlebot::UpdatePos(double xOdom, double yOdom){
    pos.x = startPos.x + xOdom;
    pos.y = startPos.y + yOdom;
}

void Turtlebot::SetPathfindingStartPos(bool _value){
    pathfindingToStart = _value;
}

bool Turtlebot::GetPathfindingStartPos(){
    return pathfindingToStart;
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
    if (yaw < 0)
    {
        yaw = 360 + yaw;
    }
}

//Calculates a point where the nearest wall is located.
void Turtlebot::CalculateWall(double dist, double angle){

    if(dist != 100){ //The distance is set to 100 when the sensor doesnt hit an object

        //Calculate wall pos relative to the robot.
        double xRelative = cos(angle * PI / 180.0) * dist;
        double yRelative = sin(angle * PI / 180.0) * dist;

        //Calculate global wall pos using the robots rotation.
        double x = cos((angle + yaw) * PI / 180.0) * dist + pos.x;
        double y = sin((angle + yaw) * PI / 180.0) * dist + pos.y;

        newPoint.x = x;
        newPoint.y = y;

    }
}



void Turtlebot::odomCallback(const nav_msgs::Odometry::ConstPtr& msg){
    //Update the robot position
    UpdatePos(msg->pose.pose.position.x, msg->pose.pose.position.y);

    UpdateRotation(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
    //ROS_INFO("[%i], (%f, %f)", id, pos.x, pos.y);
}

void Turtlebot::rangeCallback (const std_msgs::Float64MultiArray::ConstPtr& msg){   //Callback function that is called each time range is updated
    //ROS_INFO("ROBOT: [%f], Range: [%f], Angle: [%f]", msg->data[0], msg->data[1], msg->data[2]);

    //Each turtlebot has a specific id. Ex robot 0 has id 107.  bugTest = 17
    if(id == 0 && msg->data[0] == 102){ //box world: 128. office = 107. newoffice 102. testBox 38. TOffice = 77. smallBox = 56. robust = 38. curveOffice 77
        CalculateWall(msg->data[1]-0.1, msg->data[2]);
    }
    else if(id == 1 && msg->data[0] == 1746){ //box world 1772. office = 1751. newoffice 1746. smallBox = 1700 = 1682
        CalculateWall(msg->data[1]-0.1, msg->data[2]);
    }
    else if(id == 2 && msg->data[0] == 3390){ //box world 3416. office = 3395. newoffice 3390. smallBox = 3344 = 3326
        CalculateWall(msg->data[1]-0.1, msg->data[2]);
    }     
}


//Automatically create the topic name using the turtlebot id
string Turtlebot::Topic(string topic){
    stringstream sstm; //Stringstream allows to add variables to text
    sstm << "/turtlebot" << id << topic; //Creates a topic with the turtlebot id
    cout << "Topic: " << sstm.str() << endl;
    return sstm.str();
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
    SetupLineMarker();
}

//Creates a new Movement struct with given variables. The movement is pushed into the "movements" list
void Turtlebot::NewMovement(MovementType _movementType, Position _goalPos){
    //cout << "New Movement Command has been given to [" << id << "]" << " of type: Traverse " << endl;
    Movement m;
    m.movementType = _movementType;
    m.goalPos = _goalPos;

    //Draw Line to visualize the new movement path
    if(movements.empty()){
        Line(GetPosition(), m.goalPos);
    }
    else{
        Line(movements.back().goalPos, m.goalPos);
    }
    m.lineId = LinePointId + (id * 1000);

    movements.push_back(m);

}

//Creates a new Movement with type "turn". Relative 90 will move the turtlebot 90 deg relative to the robot (turns left). While false, turtlebot will move to 90 deg globally
void Turtlebot::NewMovement(MovementType _movementType, double _goalRotation, TurnType _turnType){
    //cout << "New Movement Command has been given to [" << id << "]" << " (" << _goalRotation;
    Movement m;
    m.movementType = _movementType;
    m.goalRotation = _goalRotation;
    m.turnType = _turnType;

    // if(_turnType == relative){
    //     //cout << ", relative)" << endl;
    // }
    // else
    //     //cout << ", absolute)" << endl;

    movements.push_back(m);

    //Draw MLine
}

void Turtlebot::GoalReached(Position _goal){
    //cout << "The goal has been reached by turtlebot [" << id << "]" << endl;
    freeCell = _goal;
    movementState = idle;
    prevPos = _goal;
    //Stop force pathfinding 
}

void Turtlebot::GoalYawReached(){
    cout << "The yaw has been reached by turtlebot [" << id << "]" << endl;
    movementState = nextState;
}


//Moves the turtlebot using the "movements" list. Will move towards the first index of the movements list.
void Turtlebot::Move(){


    //Check if "movements" has anything
    if(!movements.empty()){
        //If movementype is turn
        if(movements.front().movementType == turn){
            turningRobot = true;
            //Gamma is the difference from the current yaw and the goalyaw. The robot will use gamma to find out how much to move with.
            double gamma;

            if(movements.front().turnType == relative){
                //cout << "Turn Type: Relative" << endl;
                if(movements.front().fixRelative == false)
                    //Adds yaw to the goal rotation to make it relative.
                    movements.front().goalRotation = movements.front().goalRotation + yaw;
                    movements.front().fixRelative = true; //Only convert the goal rotation to relative once.
            }
            else {
                //cout << "Turn Type: Absolute" << endl;
            }
            gamma = movements.front().goalRotation - yaw; //Calculate gamma
            //cout << "Current Yaw: " << yaw << endl;
            //cout << "Goal Yaw:" << movements.front().goalRotation << endl;
            
            //Fix gamma
            if(gamma < 0){
                //cout << "Gamma negativez" << endl;
                double positiveGamma = 360 + gamma;
                //cout << "Positive Gamma:" << positiveGamma << endl;
                if(positiveGamma < abs(gamma)){
                    gamma = positiveGamma;
                }
            }
            //Fix gamma
            if(gamma > 0){
                //cout << "Gamma Positive" << endl;
                double negativeGamma = gamma - 360;
                //cout << "Negative Gamma:" << negativeGamma << endl;
                if(negativeGamma < abs(gamma)){
                    gamma = negativeGamma;
                }
            }

            //cout << "Gamma: " << gamma << endl;

            if(gamma < 1 && gamma > -1){
                //cout << "Goal has been reached!" << endl;
                movements.pop_front();
                turningRobot = false;
            }

            //If the robots angle is in the margin (Rotated correctly)
            cmd_vel_message.angular.z = gamma/180 * PI * rotationSpeed;

            cmd_vel_message.linear.x = 0;

            //Publish linear and rotation 
            cmd_vel_pub.publish(cmd_vel_message);
        }
        //If movementype is to traverse to a point
        else if(movements.front().movementType == traverse){
            //cout << "Traversing robot" << endl;
            //Alpha is the angle from the robot pos to the goal pos. (Absolute vale)
            double alpha, gamma, beta;

            Position relativeGoal; //The goal relative to the robot's current position (E.g currentPosition = (5, 9) & goalPosition = (6, 12) then goalPos = (1, 3))
            relativeGoal.x = movements.front().goalPos.x - pos.x;
            relativeGoal.y = movements.front().goalPos.y - pos.y;
            alpha = atan2(relativeGoal.y, relativeGoal.x) * 57.2957795;

            //Alpha can be negtive (E.g -90), so we fix that here so as degrees from 0-360 are desired
            if(alpha < 0)
                alpha = 360 + alpha;

            gamma = alpha - yaw;


            if(alpha >= yaw){
                //cout << "Alpha > yaw" << endl;
                double newGamma = (360 - gamma) * -1;
                //cout << "New Gamma: " << newGamma << endl;
                if(abs(newGamma) <= abs(gamma))
                    gamma = newGamma;
            }
            else if(alpha < yaw){
                //cout << "Alpha < yaw" << endl;
                double newGamma = 360 + gamma;
                //cout << "New Gamma: " << newGamma << endl;
                if(abs(newGamma) <= abs(gamma))
                    gamma = newGamma;
            }

            cmd_vel_message.angular.z = gamma/180 * PI * rotationSpeed;

            //If the robots angle is in the margin (Rotated correctly)
            if(gamma < 4 && gamma > -4){
                //cout << "PosDiffernce: (" << posDifference.x << ", " << posDifference.y << ")" << endl;
                //cout << "PosDiff Abosulte: (" << abs(posDifference.x) << ", " << abs(posDifference.y) << ")" << endl;
                
                //If the robot isn't near the goalPos: Keep moving
                if((abs(relativeGoal.x) > 0.08) || (abs(relativeGoal.y) > 0.08)){
                    cmd_vel_message.linear.x = speed;
                }
                //If the robot has reached the goalPos
                else{
                    cmd_vel_message.linear.x = 0;
                    //cout << "Goal has been reached!" << endl;
                    GoalReached(movements.front().goalPos);
                    RemoveLine(movements.front().lineId);
                    movements.pop_front();
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
}

void Turtlebot::EmptyList(){
    for(auto const& m : movements){
        RemoveLine(m.lineId);
    }
    movements.clear();
    turningRobot = false;
}

void Turtlebot::EmptyfreeCell(){
    freeCell.x = 0;
    freeCell.y = 0;
}

//Reset the last detection point
void Turtlebot::EmptyNewPoint(){
    newPoint.x = 0;
    newPoint.y = 0;
}


//Creates the topic visualization_marker/robotMarker
void Turtlebot::SetupLineMarker(){
    NodeHandle node_handle_Mline;

    LineMarker_pub = node_handle_Mline.advertise<visualization_msgs::Marker>("/visualization_marker/mline", 1);
}

void Turtlebot::Line(Position startPos, Position goalPos){

    LinePointId++;
    LineLineId++;

    visualization_msgs::Marker mLineMarkerPoints, mLineMarkerLine;
    mLineMarkerPoints.header.frame_id = "map";
    mLineMarkerPoints.header.stamp = ros::Time();
    mLineMarkerPoints.ns = "point";
    mLineMarkerPoints.id = LinePointId + (id * 1000);
    mLineMarkerPoints.type = visualization_msgs::Marker::POINTS;
    mLineMarkerPoints.action = visualization_msgs::Marker::ADD;
    mLineMarkerPoints.pose.position.x = 0;
    mLineMarkerPoints.pose.position.y = 0;
    mLineMarkerPoints.pose.position.z = 0;
    mLineMarkerPoints.pose.orientation.x = 0.0;
    mLineMarkerPoints.pose.orientation.y = 0.0;
    mLineMarkerPoints.pose.orientation.z = 0.0;
    mLineMarkerPoints.pose.orientation.w = 1.0;
    mLineMarkerPoints.scale.x = 0.15;
    mLineMarkerPoints.scale.y = 0.15;
    mLineMarkerPoints.scale.z = 0.15;
    mLineMarkerPoints.color.a = 0; // Don't forget to set the alpha!

    mLineMarkerPoints.lifetime = ros::Duration();

    /************** LINE MARKER SETUP*****************/
    mLineMarkerLine.header.frame_id = "map";
    mLineMarkerLine.header.stamp = ros::Time();
    mLineMarkerLine.ns = "line";
    mLineMarkerLine.id = LinePointId + (id * 1000);
    mLineMarkerLine.type = visualization_msgs::Marker::LINE_STRIP;
    mLineMarkerLine.action = visualization_msgs::Marker::ADD;
    mLineMarkerLine.pose.position.x = 0;
    mLineMarkerLine.pose.position.y = 0;
    mLineMarkerLine.pose.position.z = 0;
//    mLineMarkerLine.pose.orientation.x = 0.0;
//    mLineMarkerLine.pose.orientation.y = 0.0;
//    mLineMarkerLine.pose.orientation.z = 0.0;
    mLineMarkerLine.pose.orientation.w = 1.0;
    mLineMarkerLine.scale.x = 0.1;
//    mLineMarkerLine.scale.y = 1.0;
//    mLineMarkerLine.scale.z = 1.0;
    mLineMarkerLine.color.a = 0.8; // Don't forget to set the alpha!

    //If statement her
    if (id == 0)
    {
        mLineMarkerLine.color.r = 1.0;
        mLineMarkerLine.color.g = 0.0;
        mLineMarkerLine.color.b = 0.0;

    } else if (id == 1)
    {
        mLineMarkerLine.color.r = 0.0;
        mLineMarkerLine.color.g = 1.0;
        mLineMarkerLine.color.b = 0.0;
        
    } else if (id == 2)
    {
        mLineMarkerLine.color.r = 0.0;
        mLineMarkerLine.color.g = 0.0;
        mLineMarkerLine.color.b = 1.0;
    }

    mLineMarkerLine.lifetime = ros::Duration();

    for (uint32_t i = 0; i < 2; ++i)
    { 
        geometry_msgs::Point pStart, pGoal;
        pStart.x = startPos.x;
        pStart.y = startPos.y;
        pStart.z = 0.1;

        pGoal.x = goalPos.x;
        pGoal.y = goalPos.y;
        pGoal.z = 0.1;
  
        mLineMarkerPoints.points.push_back(pStart);
        mLineMarkerPoints.points.push_back(pGoal);
  
        mLineMarkerLine.points.push_back(pStart);
        mLineMarkerLine.points.push_back(pGoal);
    }
    usleep(100);
    LineMarker_pub.publish(mLineMarkerLine);

}


void Turtlebot::RemoveLine(int lineId){
    visualization_msgs::Marker mLineMarkerLine;

    mLineMarkerLine.header.frame_id = "map";
    mLineMarkerLine.header.stamp = ros::Time();
    mLineMarkerLine.ns = "line";
    mLineMarkerLine.id = lineId;
    mLineMarkerLine.type = visualization_msgs::Marker::LINE_STRIP;
    mLineMarkerLine.action = visualization_msgs::Marker::DELETE;
    mLineMarkerLine.pose.position.x = 0;
    mLineMarkerLine.pose.position.y = 0;
    mLineMarkerLine.pose.position.z = 0;
    usleep(100);
    LineMarker_pub.publish(mLineMarkerLine);
}