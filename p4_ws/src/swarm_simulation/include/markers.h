#pragma once

#include <ros/ros.h>

#include "visualization_msgs/Marker.h"

#include <position.h>

using namespace ros;

class Markers{
private:
    uint64_t markerId = 0;

    Publisher vis_pub;

public:
    void NewMarker(Position pos);
    void SetupMarker();
    Markers();
};

//Creates the topic visualization_marker
void Markers::SetupMarker(){
    NodeHandle node_handle;

    vis_pub = node_handle.advertise<visualization_msgs::Marker>("/visualization_marker", 1);
}

//Constructor
Markers::Markers(){

}

void Markers::NewMarker(Position pos)
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
    marker.pose.position.z = 1;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.5;
    marker.scale.y = 0.5;
    marker.scale.z = 0.5;
    marker.color.a = 1.0; // Don't forget to set the alpha!
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    //only if using a MESH_RESOURCE marker type:
    //marker.mesh_resource = "package://pr2_description/meshes/base_v0/base.dae";
    marker.lifetime = ros::Duration();

    /*if (vis_pub.getNumSubscribers() < 1)
    {
        //ROS_WARN_ONCE("Please create a subscriber to the marker");
//        sleep(1);
        ROS_INFO("afadfadfd");
    }*/

    vis_pub.publish(marker);
}