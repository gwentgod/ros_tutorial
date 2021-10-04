#!/usr/bin/python3

import rospy as ros


def main():
    ros.init_node('hello_py')
    ros.sleep(1)

    ros.set_param("/foo", "Hello")
    ros.set_param("/bar", 1)

    if (ros.has_param("/foo") and ros.has_param("/bar")):
        foo = ros.get_param("/foo")
        bar = ros.get_param("/bar")
        ros.loginfo(f"{foo} {bar}")
    else:
        ros.logerr("Failed to get the parameters")
        exit(1)


if (__name__ == "__main__"):
    main()
