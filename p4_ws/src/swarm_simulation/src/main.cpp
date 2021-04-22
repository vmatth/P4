#include <ros/ros.h>

#include <turtlebot.h>

#include "visualization_msgs/Marker.h"


using namespace std;
using namespace ros;

uint64_t markerId;

 Publisher vis_pub;


void markersCallback(const visualization_msgs::Marker::ConstPtr& msg)
{
    cout << "Here is SPHERE" << endl;
}

void SetupMarkers(){
    NodeHandle node_handle;
//    Publisher vis_pub;

    vis_pub = node_handle.advertise<visualization_msgs::Marker>("/visualization_marker", 1);

   // sleep(1);

   // Subscriber marker_sub;
   // marker_sub = node_handle.subscribe("/visualization_marker", 1000, &markersCallback); //Subscribes to marker
   
}

void NewMarker(Position pos)
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

    if (vis_pub.getNumSubscribers() < 1)
    {
        //ROS_WARN_ONCE("Please create a subscriber to the marker");
//        sleep(1);
        ROS_INFO("afadfadfd");
    }

    vis_pub.publish( marker );
}

//Variables and functions for initializing the turtlebot swarm
namespace TurtlebotManager{

    int numRobots = 3;

    vector<Position> robotStartPositions;

    vector<std::unique_ptr<Turtlebot>> turtlebots;

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
    void GetPoints(){
        for(int i = 0; i < numRobots; i++){
            NewMarker(turtlebots[i]->GetPoint());
        }
    }

}




int main(int argc, char *argv[])
{

    init(argc, argv, "Multi_Robot_Test");

    ROS_INFO("Hello World");

    TurtlebotManager::InitializeTurtlebots();

    SetupMarkers();

    //Main loop
    Rate loop_rate(10);
//    markers_sub();
    while (ok())
    {

    

        TurtlebotManager::MoveTurtlebots();
        TurtlebotManager::GetPoints();
    
        ros::spinOnce(); //Spin for callback functions 
        loop_rate.sleep();
    }


    return 0;
}