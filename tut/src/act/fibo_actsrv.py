#!/usr/bin/env python3

import rospy as ros
import actionlib
from tut.msg import *


def executeCallback(goal):
    ros.loginfo(f"Received goal {goal.target_index}")

    # Init messages
    feedback = FibonacciFeedback()
    result = FibonacciResult()
    sequence = [1, 1]

    # Execution
    for i in range(2, goal.target_index+1):
        # Preempted
        if (actSrv.is_preempt_requested()):
            ros.logwarn("I got preempted")
            result.target_value = -1
            actSrv.set_preempted(result)
            return;

        # Executing
        sequence.append(sequence[i-1] + sequence[i-2])
        feedback.current_index = i
        feedback.current_value = sequence[i]
        actSrv.publish_feedback(feedback)

        rate.sleep()

    # Succeeded
    result.target_value = sequence[goal.target_index]
    actSrv.set_succeeded(result)
    ros.loginfo("Succeeded")



if (__name__ == "__main__"):
    ros.init_node('fibonacci')
    rate = ros.Rate(2)
    actSrv = actionlib.SimpleActionServer("/fibonacci", FibonacciAction, execute_cb=executeCallback, auto_start = False)
    actSrv.start()

    ros.loginfo("Fibonacci action server started")
    ros.spin()