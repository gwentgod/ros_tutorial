#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <tut/CountAction.h>

class countActionServer
{
private:
    // Init
    ros::NodeHandle nh;
    ros::Rate rate;
    actionlib::SimpleActionServer<tut::CountAction> actServer;

    // Init messages
    tut::CountFeedback feedback_msg;
    tut::CountResult result_msg;
    int goal;

    // Execution
    void executeCallback(const tut::CountGoalConstPtr& goal_msg)
    {
        goal = goal_msg->goal_num;
        ROS_INFO("Received goal %d", goal);
        
        for (int progress = 0; progress <= goal; ++progress) {
            rate.sleep();

            // Action died or preempted
            if (!actServer.isActive() || actServer.isPreemptRequested()) {return;}
            
            // Action succeeded
            if (progress == goal) {
                result_msg.succeeded = 1;
                actServer.setSucceeded(result_msg);
                ROS_INFO("Reached goal %d", goal);
            }

            // Executing
            else {
                feedback_msg.current_num = progress;
                actServer.publishFeedback(feedback_msg);
            }
        }
    }


    void preemptCallback()
    {
        ROS_WARN("I got preempt");
        result_msg.succeeded = 0;
        actServer.setPreempted(result_msg);
    }


public:
    countActionServer()
    : rate(2),
    actServer(nh, "/count", boost::bind(&countActionServer::executeCallback, this, _1), false)
      
    {
        actServer.registerPreemptCallback(boost::bind(&countActionServer::preemptCallback, this));
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
