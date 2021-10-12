#include <ros/ros.h>
#include "tut/Adder.h"
#include <iostream>


int main(int argc, char **argv)
{
    ros::init(argc, argv, "adder_client");
    ros::NodeHandle nh;
    ros::ServiceClient client = nh.serviceClient<tut::Adder>("/adder");
    tut::Adder adder_msg;

    while (ros::ok())
    {
        client.waitForExistence();
        ros::Duration(1).sleep();
        ROS_INFO("Please input two numbers:");
        std::cin >> adder_msg.request.a.num >> adder_msg.request.b.num;
        client.call(adder_msg);
        
        ROS_INFO("Sum = %d\n", adder_msg.response.sum.num);
    }

    return 0;
}
