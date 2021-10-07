#!/usr/bin/env python3

import rospy as ros
from tut.srv import WalkDistance, WalkDistanceResponse
from std_srvs.srv import Empty, EmptyRequest
from geometry_msgs.msg import Twist
from turtlesim.srv import TeleportAbsolute, TeleportAbsoluteRequest


class Reset:
    # client of `/clear` and `/turtle1/teleport_absolute`
    # reset turtle position and clear the board
    def __init__(self):
        self.teleport = ros.ServiceProxy("/turtle1/teleport_absolute", TeleportAbsolute)
        self.clear = ros.ServiceProxy("/clear", Empty)
        ros.wait_for_service("/turtle1/teleport_absolute")


    def reset(self):
        origin = TeleportAbsoluteRequest(0, 5.45, 0)
        self.teleport(origin)
        self.clear(EmptyRequest())


class Commander:
    # publisher of `turtle1/cmd_vel`
    # responsible for publish linear velocit to `/turtle1/cmd_vel`
    def __init__(self):
        self.commander = ros.Publisher("/turtle1/cmd_vel", Twist, queue_size=100)

    def sendVelocit(self, velocit):
        cmd = Twist()
        cmd.linear.x = velocit
        self.commander.publish(cmd)


class WalkDistanceServer:
    def __init__(self):
        self.reset = Reset()
        self.commander = Commander()
        self.walk_server = ros.Service("/turtle1/walk_distance", WalkDistance, self.walkCallback)

        self.rate = ros.Rate(60)

        self.reset.reset()


    def walkCallback(self, request):
        self.reset.reset()

        # set linear velocity to 1 & publish for `distance` seconds
        timeout = ros.Duration.from_sec(request.distance)
        start_time = ros.Time.now()
        while (ros.Time.now() - start_time < timeout):
            self.commander.sendVelocit(1)
            self.rate.sleep()

        # set linear velocity to 0 & publish once
        self.commander.sendVelocit(0)

        # return response
        return WalkDistanceResponse(True)


def main():
    ros.init_node("turtle_walk")
    walk_server = WalkDistanceServer()
    ros.sleep(1)

    ros.loginfo("Walk server started")
    ros.spin()


if (__name__ == "__main__"):
    main()
