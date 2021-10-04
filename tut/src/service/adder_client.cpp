#include <ros/ros.h>
#include "tut/Adder.h"
#include <iostream>


class Client
{
private:
    ros::NodeHandle nh;
    ros::ServiceClient client;
    tut::Adder adder_msg;

public:
    Client()
    : client(nh.serviceClient<tut::Adder>("/adder"))
    {
        client.waitForExistence();
    }

    int callAdder(int a, int b)
    {
        adder_msg.request.a.num = a;
        adder_msg.request.b.num = b;
        client.call(adder_msg);
        return adder_msg.response.sum.num;
    }
};


int main(int argc, char **argv)
{
    ros::init(argc, argv, "adder_client");
    Client client;
    ros::Duration(1).sleep();

    int a, b, sum;
    while (ros::ok())
    {
        ROS_INFO("Please input two numbers:");
        std::cin >> a >> b;
        sum = client.callAdder(a, b);
        
        ROS_INFO("Sum = %d\n", sum);
    }

    return 0;
}
