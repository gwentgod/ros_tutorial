#!/usr/bin/python3

import rospy as ros


def main():
    ros.init_node('hello_py')
    ros.sleep(1)

    ros.loginfo("Hello world!")


if (__name__ == "__main__"):
    main()
