#!/usr/bin/env python3

import rospy as ros
from turtlesim.msg import Pose


def poseCallback(data):
    ros.loginfo(f"x: {data.x:0.2f}, y: {data.y:0.2f}")


def main():
    ros.init_node("observer")
    subscriber = ros.Subscriber("/turtle1/pose", Pose, poseCallback)
    ros.sleep(1)

    ros.spin()


if (__name__ == "__main__"):
    main()
