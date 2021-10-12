#include <ros/ros.h>
#include "tut/Adder.h"


bool add(tut::AdderRequest &request, tut::AdderResponse &responce)
{
    responce.sum.num = request.a.num + request.b.num;
    return true;
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "adder_srv");
    ros::NodeHandle nh;
    ros::ServiceServer server = nh.advertiseService("/adder", add);
    ros::Duration(1).sleep();

    ROS_INFO("Adder server ready!");
    
    ros::spin();

    return 0;
}
