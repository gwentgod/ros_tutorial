#include <ros/ros.h>
#include "tut/Adder.h"


class Adder
{
private:
    ros::NodeHandle nh;
    ros::ServiceServer server;

    static bool add(tut::AdderRequest &request, tut::AdderResponse &responce)
    {
        responce.sum.num = request.a.num + request.b.num;
        return true;
    }

public:
    Adder()
    : server(nh.advertiseService("/adder", add))
    {
        ROS_INFO("Adder server ready!");
    }
};


int main(int argc, char **argv)
{
    ros::init(argc, argv, "adder_srv");
    Adder adder;
    ros::spin();

    return 0;
}
