#!/usr/bin/env python3

import rospy as ros
from geometry_msgs.msg import Twist
import random


def main():
    ros.init_node('commander')
    publisher = ros.Publisher('/turtle1/cmd_vel', Twist, queue_size=100)
    rate = ros.Rate(2)
    ros.sleep(1)

    cmd = Twist()
    cmd.linear.x = 1

    while (not ros.is_shutdown()):
        cmd.angular.z = (random.random() - 0.3) * 5
        publisher.publish(cmd)
        rate.sleep()


if (__name__ == "__main__"):
    main()
