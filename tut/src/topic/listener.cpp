#include <ros/ros.h>
#include "tut/Number.h"


void chatCallback(const tut::Number::ConstPtr& msg)
{
    ROS_INFO("I heard %d", msg->num);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "listener");
    ros::NodeHandle nh;

    // parameters of nh.subscribe: topic name, queue size, callback function
    ros::Subscriber subscriber = nh.subscribe("/chat", 100, chatCallback);
    ros::Duration(1).sleep();

    ROS_INFO("Listening Started.");
    ros::spin();

    return 0;
}
