#include <iostream>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

using namespace std;
using namespace ros;

class Turtlebot{
private:
    NodeHandle n;
    Publisher cmd_vel_pub;
    geometry_msgs::Twist cmd_vel_message;

public:

     

};

int main(int argc, char *argv[])
{

    init(argc, argv, "Multi_Robot_Test");

    Turtlebot t;

    std::cout << "Hello World" << std::endl;
    return 0;
}