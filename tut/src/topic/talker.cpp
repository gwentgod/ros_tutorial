#include <ros/ros.h>
#include "tut/Number.h"


int main(int argc, char **argv)
{
    ros::init(argc, argv, "talker");
    ros::NodeHandle nh;

    ros::Publisher publisher = nh.advertise<tut::Number>("/chat", 100);

    ros::Rate rate(2);

    // ros::Duration(1).sleep();

    int count = 1;
    tut::Number msg;
    while (ros::ok())
    {
        msg.num = count;
        publisher.publish(msg);
        ++count;

        rate.sleep();
    }

    return 0;
}
