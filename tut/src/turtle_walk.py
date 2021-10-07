#!/usr/bin/env python3

import rospy as ros
from tut.srv import WalkDistance, WalkDistanceResponse
from std_srvs.srv import Empty, EmptyRequest
from geometry_msgs.msg import Twist
from turtlesim.srv import TeleportAbsolute, TeleportAbsoluteRequest


def reset():
    # client of `/clear` and `/turtle1/teleport_absolute`
    # reset turtle position and clear the board
    teleport = ros.ServiceProxy("/turtle1/teleport_absolute", TeleportAbsolute)
    clear = ros.ServiceProxy("/clear", Empty)

    ros.wait_for_service("/turtle1/teleport_absolute")
    ros.wait_for_service("/clear")

    teleport(TeleportAbsoluteRequest(0, 5.45, 0))
    clear(EmptyRequest())


def walkCallback(request):
    # callback of srv "/turtle1/walk"
    # responsible for publish linear velocit to `/turtle1/cmd_vel`
    commander = ros.Publisher("/turtle1/cmd_vel", Twist, queue_size=100)
    rate = ros.Rate(60)

    reset()
    
    cmd = Twist()
    cmd.linear.x = 1

    # publish
    timeout = ros.Duration.from_sec(request.distance)
    start_time = ros.Time.now()
    while (ros.Time.now() - start_time < timeout):
        commander.publish(cmd)
        rate.sleep()
    
    cmd.linear.x = 0
    commander.publish(cmd)

    return WalkDistanceResponse(True)


def main():
    ros.init_node("turtle_walk")
    walk_service = ros.Service("/turtle1/walk_distancd", WalkDistance, walkCallback)
    reset()
    ros.sleep(1)

    ros.loginfo("Walk server started")
    ros.spin()


if (__name__ == "__main__"):
    main()
