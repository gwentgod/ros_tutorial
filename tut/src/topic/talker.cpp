#include <ros/ros.h>
#include "tut/Number.h"


class Talker
{
private:
    ros::NodeHandle nh;
    ros::Publisher publisher;
    tut::Number msg;

public:
    Talker()
    : publisher(nh.advertise<tut::Number>("/chat", 100)) {}

    void talk(int number)
    {
        msg.num = number;
        publisher.publish(msg);
    }
};


int main(int argc, char **argv)
{
    ros::init(argc, argv, "talker");
    Talker talker;
    ros::Rate rate(2);
    ros::Duration(1).sleep();

    int count = 1;
    while (ros::ok())
    {
        talker.talk(count);
        ++count;

        rate.sleep();
    }

    return 0;
}
