#include <ros/ros.h>
#include "tut/Number.h"

class Listener
{
private:
    ros::NodeHandle nh;
    ros::Subscriber subscriber;

    static void chatCallback(const tut::Number::ConstPtr& msg)
    {
        ROS_INFO("I heard %d", msg->num);
    }

public:
    Listener()
    //                        name, queue size, callback function
    : subscriber(nh.subscribe("/chat", 100, chatCallback))
    {
        ROS_INFO("Listening Started.");
    }
};


int main(int argc, char **argv)
{
    ros::init(argc, argv, "listener");
    Listener listener;
    ros::Duration(1).sleep();

    ros::spin();

    return 0;
}
