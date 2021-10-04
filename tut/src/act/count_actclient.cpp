#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include "tut/CountAction.h"
#include <iostream>


class countActionClient
{
private:
    ros::NodeHandle nh;
    actionlib::SimpleActionClient<tut::CountAction> actClient;
    tut::CountGoal goal_msg;

    static void doneCallback(const actionlib::SimpleClientGoalState& state,
                             const tut::CountResultConstPtr& result)
    {
        ROS_INFO("Action status: %s\n", state.toString().c_str());
    }


    static void activeCallback()
    {
        ROS_INFO("Action started!");
    }


    static void feedbackCallback(const tut::CountFeedbackConstPtr& feedback)
    {
        ROS_INFO("Got feedback: %d", feedback->current_num);
    }


public:
    countActionClient()
      // true causes the client to spin its own thread
    : actClient("count", true)
    {
        ROS_INFO("Waiting for action server");
        actClient.waitForServer();
        ros::Duration(0.5).sleep();
        ROS_INFO("Action server connected");
    }

    void callActServer(const int& goal, const int& waiting_time)
    {
        goal_msg.goal_num = goal;
        actClient.sendGoal(goal_msg, &doneCallback, &activeCallback, &feedbackCallback);

        bool finished_before_timeout;
        finished_before_timeout = actClient.waitForResult(ros::Duration(waiting_time));

        if (!finished_before_timeout) {
            actClient.cancelGoal();
            ROS_WARN("Action timeout");
        }
    }
};


int main(int argc, char** argv)
{
    ros::init(argc, argv, "count_actclient");
    countActionClient counter_client;

    while (ros::ok()) {
        ros::Duration(0.5).sleep();
        ROS_INFO("Please input goal and waiting time:");
        int goal, waiting_time;
        std::cin >> goal >> waiting_time;
        counter_client.callActServer(goal, waiting_time);
    }

    return 0;
}