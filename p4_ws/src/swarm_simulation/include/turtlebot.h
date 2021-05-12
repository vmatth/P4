
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

    bool avoiding = false; //If the robot is avoiding collision with another robot!

    double speed = 0.3;
    double rotationSpeed = 1;

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

    void SetAvoiding(bool);

    bool GetAvoiding();

    void PauseMovement(); //Stops the movement by setting speed to 0.

    void ResumeMovement(); //Resumes by reverting the speed

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

void Turtlebot::SetAvoiding(bool _avoid){
    avoiding = _avoid;
}

bool Turtlebot::GetAvoiding(){
    return avoiding;
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


        //AngleCalc(newPoint, angle, dist);
    }
}

//Checks if the newPoint is in the robot's trajectory
void Turtlebot::AngleCalc(Position point, double angle, double dist){
    if(turningRobot)
        return;
    //Only check if a wall is in these angles
    if((angle >= 0 && angle <= 90) || angle >= 270 && angle <= 360){
        //FIx angle to go from 0-90 and not 270-360
        if(angle >= 270)
            angle = (360 - angle) * -1;

        //cout << "Angle: " << angle << endl;
        //The distance for the angle is calculated using the formula:
        double calculatedDistance = 0.0000493827 * pow(abs(angle), 2) - 0.0077777 * abs(angle) + 0.5;

    // cout << "Calculated distance for angle: " << calculatedDistance << endl;
    // cout << "Actual distance: " << dist << endl;
        
        //Check if the distance is in the turtlebot's trajectory
        if(dist <= calculatedDistance){
            //cout << "You about to hit something" << endl;

            EmptyList();

            NewMovement(turn, -90 + angle, relative);

            //Find a new point that is in front of the turtlebot so it moves along the wall
            Position pointInFront;
            pointInFront.x = pos.x + cos((-90 + angle + yaw) * PI/180.0);
            pointInFront.y = pos.y + sin((-90 + angle + yaw) * PI/180.0);

            NewMovement(traverse, pointInFront);
            followWall = true;

        }
    }
    //If the robot is following a wall
    if(followWall){
        if(angle >= 80 && angle <= 100){
            cout << "Following wall with angle: " << angle << endl;
            //Find a new point that is in front of the turtlebot so it moves along the wall
            Position pointInFront;
            pointInFront.x = pos.x + cos((-90 + angle + yaw) * PI/180.0);
            pointInFront.y = pos.y + sin((-90 + angle + yaw) * PI/180.0);
            //Only move along wall when no other movements are active
            if(movements.empty())
                NewMovement(traverse, pointInFront);
        }
        else if(angle >= 110 && angle <= 130){
            followWall = false;
            cout << "Wall is too far away" << endl;
            EmptyList();

            NewMovement(turn, 90, relative);

            //Find a new point that is in front of the turtlebot so it moves along the wall
            Position pointInFront;
            pointInFront.x = pos.x + cos((-90 + angle + yaw) * PI/180.0);
            pointInFront.y = pos.y + sin((-90 + angle + yaw) * PI/180.0);
            NewMovement(traverse, pointInFront);

            /*Position goalPos;
            goalPos.x = 13;
            goalPos.y = 5;
            NewMovement(traverse, goalPos);*/
        }
    }
    /*
    if(angle <= 95 && angle >= 85) //If the angle is close to 90 deg, then the point is near left to the robot
    { 
        cout << "Angle: " << angle << endl;
        if(dist <= 0.5) //Only check when the wall is close
        {  
            cout << "Wall to the left of the turtlebot!" << endl;
            //Find a new point that is in front of the turtlebot so it moves along the wall
            Position pointInFront;
            pointInFront.x = pos.x + cos(yaw * PI/180.0);
            pointInFront.y = pos.y + sin(yaw * PI/180.0);

            //Update goalPos to the point in front of the turtlebot
            goalPos.x = pointInFront.x;
            goalPos.y = pointInFront.y;
        }
    }
    if(angle > 130 || angle < 85){  
        cout << "CURRENT YAW: " << yaw << endl; 
        goalYaw = 90 + yaw;
        cout << "Goal Yaw: " << goalYaw;
        cout << "sdfgfgftghgrshhhhfdhgfghfhghfhghfhhfhghfghfghfhf" << endl;
        movementState = turning;
        nextState = direct;
        goalPos.x = 6;
        goalPos.y = 0; //lmao
    }*/

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

    //Each turtlebot has a specific id. Ex robot 0 has id 107.  bugTest = 17
    if(id == 0 && msg->data[0] == 104){ //box world: 104. office = 107
        //ROS_INFO("ROBOT 0 Id: [%f], Range: [%f], Angle: [%f]", msg->data[0], msg->data[1], msg->data[2]);
        range = msg->data[1];
        angle = msg->data[2];
        CalculateWall(msg->data[1], msg->data[2]);
    }
    else if(id == 1 && msg->data[0] == 1748){ //box world 1748. office = 1751
        //ROS_INFO("ROBOT 1 Id: [%f], Range: [%f], Angle: [%f]", msg->data[0], msg->data[1], msg->data[2]);
        CalculateWall(msg->data[1], msg->data[2]);
    }
    else if(id == 2 && msg->data[0] == 3392){ //box world 3392. office = 3395
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

//Creates a new Movement struct with given variables. The movement is pushed into the "movements" list
void Turtlebot::NewMovement(MovementType _movementType, Position _goalPos){
    //cout << "New Movement Command has been given to [" << id << "]" << " of type: Traverse " << endl;
    Movement m;
    m.movementType = _movementType;
    m.goalPos = _goalPos;

    movements.push_back(m);
}

//Creates a new Movement with type "turn". Relative 90 will move the turtlebot 90 deg relative to the robot (turns left). While false, turtlebot will move to 90 deg globally
void Turtlebot::NewMovement(MovementType _movementType, double _goalRotation, TurnType _turnType){
    cout << "New Movement Command has been given to [" << id << "]" << " (" << _goalRotation;
    Movement m;
    m.movementType = _movementType;
    m.goalRotation = _goalRotation;
    m.turnType = _turnType;

    if(_turnType == relative){
        cout << ", relative)" << endl;
    }
    else
        cout << ", absolute)" << endl;

    movements.push_back(m);
}

void Turtlebot::GoalReached(Position _goal){
    //cout << "The goal has been reached by turtlebot [" << id << "]" << endl;
    freeCell = _goal;
    movementState = idle;
    PrintPosition(_goal, "Goal Reached: ");
    prevPos = _goal;
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
                cout << "Goal has been reached!" << endl;
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

            //cout << "Alpha (Absolute angles to goalPos): " << alpha << endl;

            //Alpha is always the shortest angle to the goal. E.g: 270 deg = -90
            //This is a problem when the goal is 180 deg, as it switches from 180 deg to -180 deg

           //If the alpha is negative (E.g -185), find the positive alpha (E.g 175) and check which is closets to the robot's current yaw
            /*if(alpha < 0){
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

            }*/

    

            gamma = alpha - yaw;
            //cout << "Robot Yaw: " << yaw << endl; //Robottens vinkel

            //cout << "gamma: " << gamma << endl; //forskel i vinklerne

            //Always find the shortest angle (gamma) to the destination (alpha)

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

            //cout << "Final gamma: " << gamma << endl; //forskel i vinklerne
            // cout << "---------------" << endl;

            //cout << "MoveToGoal Pos: ("  << goalPos.x << ", " << goalPos.y << ")" << endl;

            // cout << "RobotPos: ("  << pos.x << ", " << pos.y << ")" << endl;

            // cout << "Alpha (Angle to point): " << alpha << endl; //Vinklen fra (0,0) til punktet


            // cout << "Gamma (Rads): " << (gamma/180 * PI) << endl; //forskel i vinklerne

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
                    cout << "Goal has been reached!" << endl;
                    GoalReached(movements.front().goalPos);
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
            //cout << "Moving with movement type: "
    }
}

void Turtlebot::EmptyList(){
    movements.clear();
    cout << "List emptied!" << endl;
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
