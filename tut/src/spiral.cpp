#include <ros/ros.h>
#include "turtlesim/TeleportAbsolute.h"
#include <cmath>


class Spiral
{
private:
    ros::NodeHandle nh;
    ros::ServiceClient client;
    turtlesim::TeleportAbsolute teleport;

public:
    Spiral()
    : client(nh.serviceClient<turtlesim::TeleportAbsolute>("/turtle1/teleport_absolute"))
    {
        client.waitForExistence();
    }

    void cmd(float x, float y)
    {
        teleport.request.x = x;
        teleport.request.y = y;
        client.call(teleport);
    }
};


int main(int argc, char **argv)
{
    ros::init(argc, argv, "spiral");
    Spiral spiral;
    ros::Rate rate(30);
    ros::Duration(1).sleep();

    const double center = 5.544445;
    double x, y, r, s, t=0.2;
    while (ros::ok())
    {
        t += 0.005;
        s = 1.6 - 1/t;
        r = std::exp(s);
        x = r * std::cos(10 * s) + center;
        y = r * std::sin(10 * s) + center;

        spiral.cmd(x, y);
        rate.sleep();
    }

    return 0;
}
