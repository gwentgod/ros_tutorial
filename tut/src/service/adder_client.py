#!/usr/bin/env python3

import rospy as ros
from tut.srv import *


def main():
    ros.init_node("adder_clientpy")
    ros.sleep(1)

    client = ros.ServiceProxy('/adder', Adder)
    ros.wait_for_service('/adder')

    while (not ros.is_shutdown()):
        request = AdderRequest()
    
        ros.loginfo("Please input two numbers: ")
        try:
            request.a.num, request.b.num = map(int, input().split())
        except (ValueError):
            continue;
        
        response = client(request)
        ros.loginfo(f"Sum = {response.sum.num}\n")


if (__name__ == "__main__"):
    main()
