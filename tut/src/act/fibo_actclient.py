#! /usr/bin/env python3

import rospy as ros
import actionlib
from tut.msg import *


def feedbackCallback(feedback):
    ros.loginfo(f"Index: {feedback.current_index}; Value: {feedback.current_value}")


def activeCallback():
    ros.loginfo("Action started")


def doneCallback(status, result):
    ros.loginfo(f"Target value = {result.target_value}\n")


def main():
    ros.init_node("fibo_actclient")

    # Init action goal message
    goal = FibonacciGoal()

    # Init action client
    actClient = actionlib.SimpleActionClient('/fibonacci', FibonacciAction)

    # Wait for action server
    ros.loginfo("Waiting for action client")
    actClient.wait_for_server()
    ros.loginfo("Action server connected")

    while (not ros.is_shutdown()):
        ros.loginfo("Please input target index and wait time:")
        try:
            goal.target_index, wait_time = map(int, input().split())
        except (ValueError):
            continue;

        actClient.send_goal(goal, feedback_cb=feedbackCallback, active_cb=activeCallback, done_cb=doneCallback)

        # wait_for_resul(duration) blocks until this goal finishes or timeout
        # returns finished ? true : false
        finished_before_timeout = actClient.wait_for_result(ros.Duration(wait_time))

        if (not finished_before_timeout):
            actClient.cancel_goal()
            ros.logwarn("Action timeout")

        ros.sleep(0.5);

if (__name__ == "__main__"):
    main()
