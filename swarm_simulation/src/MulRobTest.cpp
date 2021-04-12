#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

using namespace std;
using namespace ros;

int main(int argc, char *argv[])
{
    init(argc, argv, "Multi_Robot_Test");

    NodeHandle n;

    Publisher cmd_vel_pub = n.advertise<geometry_msgs::Twist>("/turtlebot1/mobile_base/commands/velocity", 1);
    geometry_msgs::Twist cmd_vel_message;
    cmd_vel_message.angular.z = 1.0;
    cmd_vel_message.linear.x = 1.0;


    NodeHandle n2;

    Publisher cmd_vel_pub2 = n.advertise<geometry_msgs::Twist>("/turtlebot2/mobile_base/commands/velocity", 1);
    geometry_msgs::Twist cmd_vel_message2;
    cmd_vel_message2.angular.z = 1.0;
    cmd_vel_message2.linear.x = 1.0;

    Rate loop_rate(10);
    while (ok())
    {
        cmd_vel_pub.publish(cmd_vel_message);
        cmd_vel_pub2.publish(cmd_vel_message2);
        loop_rate.sleep();
    }

    return(0);
}