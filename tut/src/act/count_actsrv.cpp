#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <tut/CountAction.h>


class countActionServer
{
private:
    ros::NodeHandle nh;
    ros::Rate rate;
    actionlib::SimpleActionServer<tut::CountAction> actServer;

    tut::CountFeedback feedback_msg;
    tut::CountResult result_msg;

    // execute counting
    void executeCallback(const tut::CountGoalConstPtr& goal_msg)
    {
        int goal = goal_msg->goal_num;
        ROS_INFO("Received goal %d", goal);

        int progress = 0;
        while (ros::ok()) {
            // preempted
            if (actServer.isPreemptRequested()) {
                ROS_WARN("I got preempt!");
                result_msg.succeeded = 0;
                actServer.setPreempted(result_msg);
                return;
            }

            // succeeded
            if (progress == goal) {
                result_msg.succeeded = 1;
                actServer.setSucceeded(result_msg);
                ROS_INFO("Successfully reached goal %d", goal);
                return;
            }

            // executing
            else {
                rate.sleep();
                ++progress;
                feedback_msg.current_num = progress;
                actServer.publishFeedback(feedback_msg);
            }
        }
    }

public:
    countActionServer()
    : rate(2),
    actServer(nh, "/count", boost::bind(&countActionServer::executeCallback, this, _1), false)
    {
        actServer.start();
    }
};


int main(int argc, char** argv)
{
    ros::init(argc, argv, "count_actsrv");
    countActionServer counter;
    ros::Duration(1).sleep();
    
    ROS_INFO("Conut action server started");

    ros::spin();

    return 0;
}
