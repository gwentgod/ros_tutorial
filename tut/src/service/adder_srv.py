#!/usr/bin/env python3

import rospy as ros
from tut.srv import *


def adderCallback(request):
    response = AdderResponse()
    response.sum.num = request.a.num + request.b.num
    return response;


def main():
    ros.init_node("adder_srvpy")
    ros.Service("/adder", Adder, adderCallback)
    ros.loginfo("Adder ready!")
    ros.spin()


if (__name__ == "__main__"):
    main()
