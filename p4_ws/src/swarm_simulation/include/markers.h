#pragma once

#include <ros/ros.h>

#include "visualization_msgs/Marker.h"

#include <position.h>

using namespace ros;

class Markers{
private:
    uint64_t markerId = 0;
    uint64_t robotMarkerId = 0;

    Publisher vis_pub;
    Publisher robotMarker_pub;

public:
    void NewMarker(Position pos, int robotId);
    void SetupMarker();
    void robotMarker(Position robotPos, int robotId);
    void SetupRobotMarker();
    Markers();
};

//Creates the topic visualization_marker
void Markers::SetupMarker(){
    NodeHandle node_handle;

    vis_pub = node_handle.advertise<visualization_msgs::Marker>("/visualization_marker/wall", 1);
}

//Creates the topic visualization_marker/robotMarker
void Markers::SetupRobotMarker(){
    NodeHandle node_handle_robot;

    robotMarker_pub = node_handle_robot.advertise<visualization_msgs::Marker>("/visualization_marker/robot", 1);
}

//Constructor
Markers::Markers(){

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
    marker.pose.position.z = 0.1;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.5;
    marker.scale.y = 0.5;
    marker.scale.z = 0.5;
    marker.color.a = 1.0; // Don't forget to set the alpha!
//    marker.color.r = 1.0;
//    marker.color.g = 0.0;
//    marker.color.b = 1.0;
    //only if using a MESH_RESOURCE marker type:
    //marker.mesh_resource = "package://pr2_description/meshes/base_v0/base.dae";
    marker.lifetime = ros::Duration();

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

    /*if (vis_pub.getNumSubscribers() < 1)
    {
        //ROS_WARN_ONCE("Please create a subscriber to the marker");
//        sleep(1);
        ROS_INFO("afadfadfd");
    }*/

    vis_pub.publish(marker);
}

void Markers::robotMarker(Position robotPos, int robotId)
{
    robotMarkerId++;

    visualization_msgs::Marker robotMarker;
    robotMarker.header.frame_id = "map";
    robotMarker.header.stamp = ros::Time();
    robotMarker.ns = "marker";
    robotMarker.id = robotMarkerId;
    robotMarker.type = visualization_msgs::Marker::SPHERE;
    robotMarker.action = visualization_msgs::Marker::ADD;
    robotMarker.pose.position.x = robotPos.x;
    robotMarker.pose.position.y = robotPos.y;
    robotMarker.pose.position.z = 0.1;
    robotMarker.pose.orientation.x = 0.0;
    robotMarker.pose.orientation.y = 0.0;
    robotMarker.pose.orientation.z = 0.0;
    robotMarker.pose.orientation.w = 1.0;
    robotMarker.scale.x = 0.2;
    robotMarker.scale.y = 0.2;
    robotMarker.scale.z = 0.2;
    robotMarker.color.a = 0.1; // Don't forget to set the alpha!

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

    robotMarker.lifetime = ros::Duration();

    robotMarker_pub.publish(robotMarker);


/* TESTING SITE
   // robotMarkerId = markerId;
    robotMarkerId++;
//    Position startPos;
//    Position prevPos;

    visualization_msgs::Marker robotpoints, robotlines;
    robotpoints.header.frame_id = robotlines.header.frame_id = "map";

    robotpoints.header.stamp = robotlines.header.stamp = ros::Time();
    robotpoints.ns = robotlines.ns = "robot_marker";
    robotpoints.id = robotlines.id = robotMarkerId;
    robotpoints.type = visualization_msgs::Marker::POINTS;
    robotlines.type = visualization_msgs::Marker::LINE_STRIP;
    robotpoints.action = visualization_msgs::Marker::ADD;
    robotlines.action = visualization_msgs::Marker::ADD;
    robotpoints.pose.position.x = robotPos.x;
    robotpoints.pose.position.y = robotPos.y;
    robotpoints.pose.position.z = 0;
    robotlines.pose.position.x = robotpoints.pose.position.x;
    robotlines.pose.position.y = robotpoints.pose.position.y;
    robotlines.pose.position.z = robotpoints.pose.position.z;
//    robotpoints.pose.orientation.x = robotlines.pose.orientation.x = 0.0;
 //   robotpoints.pose.orientation.y = robotlines.pose.orientation.y = 0.0;
  //  robotpoints.pose.orientation.z = robotlines.pose.orientation.z = 0.0;
//    robotpoints.pose.orientation.w = robotlines.pose.orientation.w = 1.0;
    robotpoints.scale.x = 0.1;
    robotpoints.scale.y = 0.1;
    robotlines.scale.x = 0.2; 
//    robotmarker.scale.z = 0.5;
    robotpoints.color.a = 1.0; // Don't forget to set the alpha!
    robotlines.color.a = 1.0; // Don't forget to set the alpha!

    if (robotId == 0)
    {
        robotpoints.color.r = 1.0;
        robotpoints.color.g = 0.0;
        robotpoints.color.b = 0.0;

        robotlines.color.r = 1.0;
        robotlines.color.g = 0.0;
        robotlines.color.b = 1.0;
    } else if (robotId == 1)
    {
        robotpoints.color.r = 0.0;
        robotpoints.color.g = 1.0;
        robotpoints.color.b = 0.0;
        
        robotlines.color.r = 0.0;
        robotlines.color.g = 1.0;
        robotlines.color.b = 0.0;
    } else if (robotId == 2)
    {
        robotpoints.color.r = 0.0;
        robotpoints.color.g = 0.0;
        robotpoints.color.b = 1.0;
        
        robotlines.color.r = 0.0;
        robotlines.color.g = 0.0;
        robotlines.color.b = 1.0;
    } else
    {
        cout << "Robot does not exist" << endl;
    }
    
    for (uint64_t i = 0; i < 1000; ++i)
    {
//        float y = 5 * sin(f + i / 100.0f * 2 * M_PI);
//        float z = 5 * cos(f + i / 100.0f * 2 * M_PI);
   
        geometry_msgs::Point p;
        p.x = robotPos.x;
        p.y = robotPos.y;
        p.z = 0;

        robotpoints.points.push_back(p);
        robotlines.points.push_back(p);
  
        // The line list needs two points for each line
//        line_list.points.push_back(p);
//        p.z += 1.0;
//        line_list.points.push_back(p);
    }

    robotpoints.lifetime = ros::Duration();
    robotlines.lifetime = ros::Duration();

    robotMarker_pub.publish(robotlines);
    robotMarker_pub.publish(robotpoints);
//    robotMarker_pub.publish(robotlines);
//    prevPos = robotPos; 
                                */
}