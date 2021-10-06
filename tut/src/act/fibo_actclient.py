#! /usr/bin/env python3

import rospy as ros
import actionlib
from tut.msg import *


class FiboClient:
    def __init__(self):
        # init action client
        self.actClient = actionlib.SimpleActionClient('/fibonacci', FibonacciAction)

        # init messages
        self.preempt_key = 0
        self.goal = FibonacciGoal()

        # Wait for action server
        ros.loginfo("Waiting for action client")
        self.actClient.wait_for_server()
        ros.loginfo("Action server connected")


    def feedbackCallback(self, feedback):
        ros.loginfo(f"Index: {feedback.current_index}; Value: {feedback.current_value}")

        if (feedback.current_value == self.preempt_key):
            ros.logwarn("Reached preempt key, cancling action")
            self.actClient.cancel_goal()


    def activeCallback(self):
        ros.loginfo("Action started")


    def doneCallback(self, status, result):
        ros.loginfo(f"Target value = {result.target_value}\n")


    def sendGoal(self, target_index, preempt_key):
        self.goal.target_index = target_index
        self.preempt_key = preempt_key
        self.actClient.send_goal(self.goal, feedback_cb=self.feedbackCallback, active_cb=self.activeCallback, done_cb=self.doneCallback)


def main():
    ros.init_node("fibo_actclient")
    ros.sleep(1)

    fiboClient = FiboClient()

    while (not ros.is_shutdown()):
        ros.loginfo("Please input target index and preempt key:")
        try:
            fiboClient.sendGoal(*map(int, input().split()))
        except (ValueError):
            continue


if (__name__ == "__main__"):
    main()
