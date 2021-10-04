#include <ros/ros.h>


int main(int argc, char** argv)
{
    ros::init(argc, argv, "helloworld");
    ros::NodeHandle nh;
    ros::Duration(1).sleep();

    ROS_INFO("Hello World!");

    return 0;
}
