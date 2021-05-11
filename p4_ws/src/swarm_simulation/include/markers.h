#pragma once

#include <ros/ros.h>

#include "visualization_msgs/Marker.h"

#include <position.h>
#include <index.h>

#include <unistd.h>

#include <states.h>

using namespace ros;

class Markers{
private:
    uint64_t markerId = 0;
    uint64_t robotMarkerId = 0;
    uint64_t cellMarkerId = 0;
    uint64_t mLinePointId = 0;
    uint64_t mLineLineId = 0;

    Publisher wallmarker_pub;
    Publisher robotMarker_pub;
    Publisher cellMarker_pub;
    Publisher MlineMarker_pub;

public:
    void NewMarker(Position pos, int robotId);
    void SetupMarker();
    void RobotMarker(Position robotPos, int robotId);
    void UpdateRobotMarker(Position robotPos, int rob);
    void SetupRobotMarker();
    bool IsEven(int x);
    void CellMarker(Position cellPos, State state, Index subareaIndex, int size);
    void CellMarkerUpdate(int ID, State state, Position cellPos, Index subAreaIndex, int size);
    void SetupCellMarker();
    void MLine(Position startPos, Position goalPos, int robotId);
    void SetupMlineMarker();
    Markers();
};

//Creates the topic visualization_marker
void Markers::SetupMarker(){
    NodeHandle node_handle;

    wallmarker_pub = node_handle.advertise<visualization_msgs::Marker>("/visualization_marker/wall", 1);
}

//Creates the topic visualization_marker/robotMarker
void Markers::SetupRobotMarker(){
    NodeHandle node_handle_robot;

    robotMarker_pub = node_handle_robot.advertise<visualization_msgs::Marker>("/visualization_marker/robot", 1);
}

//Creates the topic visualization_marker/robotMarker
void Markers::SetupCellMarker(){
    NodeHandle node_handle_cell;

    cellMarker_pub = node_handle_cell.advertise<visualization_msgs::Marker>("/visualization_marker/cell", 1);
}

//Creates the topic visualization_marker/robotMarker
void Markers::SetupMlineMarker(){
    NodeHandle node_handle_Mline;

    MlineMarker_pub = node_handle_Mline.advertise<visualization_msgs::Marker>("/visualization_marker/mline", 1);
}

//Constructor
Markers::Markers(){

}
bool Markers::IsEven(int x){
    if(x % 2 == 0){
        return true;
    } else{
        return false;
    }
}


void Markers::NewMarker(Position pos, int robotId)
{
    markerId++;

    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time();
    marker.ns = "marker";
    marker.id = markerId;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = pos.x;
    marker.pose.position.y = pos.y;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.2;
    marker.scale.y = 0.2;
    marker.scale.z = 0.2;
    marker.color.a = 0.7; // Don't forget to set the alpha!
//    marker.color.r = 1.0;
//    marker.color.g = 0.0;
//    marker.color.b = 1.0;
    //only if using a MESH_RESOURCE marker type:
    //marker.mesh_resource = "package://pr2_description/meshes/base_v0/base.dae";
    marker.lifetime = ros::Duration(2);

    if (robotId == 0)
    {

        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;

    } else if (robotId == 1)
    {

        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
        
    } else if (robotId == 2)
    {

        marker.color.r = 0.0;
        marker.color.g = 0.0;
        marker.color.b = 1.0;
        
    } else 
    {
        cout << "Robot does not exist" << endl;
    }

    wallmarker_pub.publish(marker);
}

void Markers::RobotMarker(Position robotPos, int robotId)
{
    robotMarkerId++;

    visualization_msgs::Marker robotMarker;
    robotMarker.header.frame_id = "map";
    robotMarker.header.stamp = ros::Time();
    robotMarker.ns = "robot";
    robotMarker.id = robotMarkerId;
    robotMarker.type = visualization_msgs::Marker::SPHERE;
    robotMarker.action = visualization_msgs::Marker::ADD;
    robotMarker.pose.position.x = robotPos.x;
    robotMarker.pose.position.y = robotPos.y;
    robotMarker.pose.position.z = 0;
    robotMarker.pose.orientation.x = 0.0;
    robotMarker.pose.orientation.y = 0.0;
    robotMarker.pose.orientation.z = 0.0;
    robotMarker.pose.orientation.w = 1.0;
    robotMarker.scale.x = 0.2;
    robotMarker.scale.y = 0.2;
    robotMarker.scale.z = 0.2;
    robotMarker.color.a = 0.2; // Don't forget to set the alpha!

    if (robotId == 0)
    {

        robotMarker.color.r = 1.0;
        robotMarker.color.g = 0.5;
        robotMarker.color.b = 0.5;

    } else if (robotId == 1)
    {

        robotMarker.color.r = 0.5;
        robotMarker.color.g = 1.0;
        robotMarker.color.b = 0.5;
        
    } else if (robotId == 2)
    {

        robotMarker.color.r = 0.5;
        robotMarker.color.g = 0.5;
        robotMarker.color.b = 1.0;
        
    } else 
    {
        cout << "Robot does not exist" << endl;
    }

    robotMarker.lifetime = ros::Duration(10);

    robotMarker_pub.publish(robotMarker);
}

void UpdateRobotMarker(){
    ;
}


void Markers::CellMarker(Position cellPos, State state, Index subareaIndex, int size)
{

    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time();
    marker.ns = "cell";
    marker.id = cellMarkerId;
    marker.type = visualization_msgs::Marker::POINTS;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = 0;
    marker.pose.position.y = 0;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.1;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;
    marker.color.a = 0.7; // Don't forget to set the alpha!

    for(int x = 0; x < size; x++){ //i am good boi
        for(int y = 0; y < size; y++){
            if(subareaIndex.x == x && subareaIndex.y == y){
                //cout << "Dette er inde i if-statement" << endl;
                if(IsEven(y) == true){
                    if(IsEven(x) == true){
                        if(state == Unexplored){
                        // cout << "This cell is unexplored" << endl;
                            marker.color.r = 0.8;
                            marker.color.g = 0.1;
                            marker.color.b = 0.8;

                        }   else if (state == Wall){

                            marker.color.r = 0;
                            marker.color.g = 1;
                            marker.color.b = 1;

                        }   else if (state == Free){
                            //cout << "This cell is Free" << endl;
                            marker.color.r = 1;
                            marker.color.g = 1;
                            marker.color.b = 0;
                        }
                    } else{
                        if(state == Unexplored){
                        // cout << "This cell is unexplored" << endl;
                            marker.color.r = 0.5;
                            marker.color.g = 0.8;
                            marker.color.b = 1;

                        }   else if (state == Wall){

                            marker.color.r = 0;
                            marker.color.g = 1;
                            marker.color.b = 1;

                        }   else if (state == Free){
                            //cout << "This cell is Free" << endl;
                            marker.color.r = 0.14;
                            marker.color.g = 0.6;
                            marker.color.b = 0.1;
                        }
                    }
                } else if(IsEven(y) == false){
                    if(IsEven(x) == false){
                        if(state == Unexplored){
                        // cout << "This cell is unexplored" << endl;
                            marker.color.r = 0.8;
                            marker.color.g = 0.1;
                            marker.color.b = 0.8;

                        }   else if (state == Wall){

                            marker.color.r = 0;
                            marker.color.g = 1;
                            marker.color.b = 1;

                        }   else if (state == Free){
                            //cout << "This cell is Free" << endl;
                            marker.color.r = 1;
                            marker.color.g = 1;
                            marker.color.b = 0;
                        }
                    } else{
                        if(state == Unexplored){
                        // cout << "This cell is unexplored" << endl;
                            marker.color.r = 0.5;
                            marker.color.g = 0.8;
                            marker.color.b = 1;

                        }   else if (state == Wall){

                            marker.color.r = 0;
                            marker.color.g = 1;
                            marker.color.b = 1;

                        }   else if (state == Free){
                            //cout << "This cell is Free" << endl;
                            marker.color.r = 0.14;
                            marker.color.g = 0.6;
                            marker.color.b = 0.1;
                        }
                    }
                }
            }  
        }
    }

    marker.lifetime = ros::Duration();

        for (uint32_t i = 0; i < 1; ++i)
    { 
        geometry_msgs::Point pCell;
        pCell.x = cellPos.x;
        pCell.y = cellPos.y;
        pCell.z = 0;
  
        marker.points.push_back(pCell);
    }
    cellMarker_pub.publish(marker);
    cellMarkerId++;
}

void Markers::CellMarkerUpdate(int ID, State state, Position cellPos, Index subareaIndex, int size)
{
    visualization_msgs::Marker cellmarker;
    cellmarker.header.frame_id = "map";
    cellmarker.header.stamp = ros::Time();
    cellmarker.ns = "cell";
    cellmarker.id = ID;
    cellmarker.type = visualization_msgs::Marker::POINTS;
    cellmarker.action = visualization_msgs::Marker::ADD;
    cellmarker.scale.x = 0.1;
    cellmarker.scale.y = 0.1;
    cellmarker.scale.z = 0.1;
    cellmarker.color.a = 0.7; // Don't forget to set the alpha!
    cellmarker.lifetime = ros::Duration();

    for(int x = 0; x < size; x++){ //i am good boi
        for(int y = 0; y < size; y++){
            if(subareaIndex.x == x && subareaIndex.y == y){
                //cout << "Dette er inde i if-statement" << endl;
                if(IsEven(y) == true){
                    if(IsEven(x) == true){
                        if(state == Unexplored){
                        // cout << "This cell is unexplored" << endl;
                            cellmarker.color.r = 0.8;
                            cellmarker.color.g = 0.1;
                            cellmarker.color.b = 0.8;

                        }   else if (state == Wall){

                            cellmarker.color.r = 1;
                            cellmarker.color.g = 1;
                            cellmarker.color.b = 1;

                        }   else if (state == Free){
                            //cout << "This cell is Free" << endl;
                            cellmarker.color.r = 1;
                            cellmarker.color.g = 1;
                            cellmarker.color.b = 0;
                        }
                    } else{
                        if(state == Unexplored){
                        // cout << "This cell is unexplored" << endl;
                            cellmarker.color.r = 0.5;
                            cellmarker.color.g = 0.8;
                            cellmarker.color.b = 1;

                        }   else if (state == Wall){

                            cellmarker.color.r = 1;
                            cellmarker.color.g = 1;
                            cellmarker.color.b = 1;

                        }   else if (state == Free){
                            //cout << "This cell is Free" << endl;
                            cellmarker.color.r = 0.14;
                            cellmarker.color.g = 0.6;
                            cellmarker.color.b = 0.1;
                        }
                    }
                } else if(IsEven(y) == false){
                    if(IsEven(x) == false){
                        if(state == Unexplored){
                        // cout << "This cell is unexplored" << endl;
                            cellmarker.color.r = 0.8;
                            cellmarker.color.g = 0.1;
                            cellmarker.color.b = 0.8;

                        }   else if (state == Wall){

                            cellmarker.color.r = 1;
                            cellmarker.color.g = 1;
                            cellmarker.color.b = 1;

                        }   else if (state == Free){
                            //cout << "This cell is Free" << endl;
                            cellmarker.color.r = 1;
                            cellmarker.color.g = 1;
                            cellmarker.color.b = 0;
                        }
                    } else{
                        if(state == Unexplored){
                        // cout << "This cell is unexplored" << endl;
                            cellmarker.color.r = 0.5;
                            cellmarker.color.g = 0.8;
                            cellmarker.color.b = 1;

                        }   else if (state == Wall){

                            cellmarker.color.r = 1;
                            cellmarker.color.g = 1;
                            cellmarker.color.b = 1;

                        }   else if (state == Free){
                            //cout << "This cell is Free" << endl;
                            cellmarker.color.r = 0.14;
                            cellmarker.color.g = 0.6;
                            cellmarker.color.b = 0.1;
                        }
                    }
                }
            }  
        }
    }

    // if(state == Unexplored){

    //     cellmarker.color.r = 1;
    //     cellmarker.color.g = 0;
    //     cellmarker.color.b = 1;

    // }   else if (state == Wall){

    //     cellmarker.color.r = 1;
    //     cellmarker.color.g = 1;
    //     cellmarker.color.b = 1;

    // }   else if (state == Free){

    //     cellmarker.color.r = 1;
    //     cellmarker.color.g = 1;
    //     cellmarker.color.b = 0;
    // }

        for (uint32_t i = 0; i < 1; ++i)
    { 
        geometry_msgs::Point pCell;
        pCell.x = cellPos.x;
        pCell.y = cellPos.y;
        pCell.z = 0;
  
        cellmarker.points.push_back(pCell);
    }

    cellMarker_pub.publish(cellmarker);
}

void Markers::MLine(Position startPos, Position goalPos, int robotId)
{
    mLinePointId++;
    mLineLineId++;

    visualization_msgs::Marker mLineMarkerPoints, mLineMarkerLine;
    mLineMarkerPoints.header.frame_id = "map";
    mLineMarkerPoints.header.stamp = ros::Time();
    mLineMarkerPoints.ns = "point";
    mLineMarkerPoints.id = mLinePointId;
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
    mLineMarkerPoints.color.a = 1.0; // Don't forget to set the alpha!

    mLineMarkerPoints.lifetime = ros::Duration();

    /************** LINE MARKER SETUP*****************/
    mLineMarkerLine.header.frame_id = "map";
    mLineMarkerLine.header.stamp = ros::Time();
    mLineMarkerLine.ns = "line";
    mLineMarkerLine.id = mLinePointId;
    mLineMarkerLine.type = visualization_msgs::Marker::LINE_STRIP;
    mLineMarkerLine.action = visualization_msgs::Marker::ADD;
    mLineMarkerLine.pose.position.x = 0;
    mLineMarkerLine.pose.position.y = 0;
    mLineMarkerLine.pose.position.z = 0;
//    mLineMarkerLine.pose.orientation.x = 0.0;
//    mLineMarkerLine.pose.orientation.y = 0.0;
//    mLineMarkerLine.pose.orientation.z = 0.0;
    mLineMarkerLine.pose.orientation.w = 1.0;
    mLineMarkerLine.scale.x = 0.2;
//    mLineMarkerLine.scale.y = 1.0;
//    mLineMarkerLine.scale.z = 1.0;
    mLineMarkerLine.color.a = 1.0; // Don't forget to set the alpha!

    //If statement her
    if (robotId == 0)
    {

        mLineMarkerPoints.color.r = 1.0;
        mLineMarkerPoints.color.g = 0.0;
        mLineMarkerPoints.color.b = 0.0;

        mLineMarkerLine.color.r = 1.0;
        mLineMarkerLine.color.g = 0.0;
        mLineMarkerLine.color.b = 0.0;

    } else if (robotId == 1)
    {

        mLineMarkerPoints.color.r = 0.0;
        mLineMarkerPoints.color.g = 1.0;
        mLineMarkerPoints.color.b = 0.0;

        mLineMarkerLine.color.r = 0.0;
        mLineMarkerLine.color.g = 1.0;
        mLineMarkerLine.color.b = 0.0;
        
    } else if (robotId == 2)
    {
        mLineMarkerPoints.color.r = 0.0;
        mLineMarkerPoints.color.g = 0.0;
        mLineMarkerPoints.color.b = 1.0;

        mLineMarkerLine.color.r = 1.0;
        mLineMarkerLine.color.g = 0.0;
        mLineMarkerLine.color.b = 1.0;
    }
/*
    mLineMarkerPoints.color.r = 0.0;
    mLineMarkerPoints.color.g = 1.0;
    mLineMarkerPoints.color.b = 0.0;

    mLineMarkerLine.color.r = 1.0;
    mLineMarkerLine.color.g = 0.0;
    mLineMarkerLine.color.b = 1.0;
*/
    mLineMarkerLine.lifetime = ros::Duration();

    for (uint32_t i = 0; i < 2; ++i)
    { 
        geometry_msgs::Point pStart, pGoal;
        pStart.x = startPos.x;
        pStart.y = startPos.y;
        pStart.z = 0;

        pGoal.x = goalPos.x;
        pGoal.y = goalPos.y;
        pGoal.z = 0;
  
        mLineMarkerPoints.points.push_back(pStart);
        mLineMarkerPoints.points.push_back(pGoal);
  
        mLineMarkerLine.points.push_back(pStart);
        mLineMarkerLine.points.push_back(pGoal);
    }

    usleep(1000);
    MlineMarker_pub.publish(mLineMarkerPoints);
    usleep(1000);
    MlineMarker_pub.publish(mLineMarkerLine);

}