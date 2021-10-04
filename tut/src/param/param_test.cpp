#include <ros/ros.h>
#include <string>


int main(int argc, char** argv)
{
    ros::init(argc, argv, "param_test");
    ros::NodeHandle nh;
    ros::Duration(1).sleep();

    nh.setParam("/foo", "Hello");
    nh.setParam("/bar", 1);

    std::string foo = "";
    int bar = 0;

    if (nh.getParam("/foo", foo) && nh.getParam("/bar", bar)) {
        ROS_INFO("%s %d", foo.c_str(), bar);
    }
    else{
        ROS_ERROR("Failed to get the parameters");
        return 1;
    }

    return 0;
}
