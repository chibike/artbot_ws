#ifndef __STATE_CHANGE_ACTION_SERVER_H
#define __STATE_CHANGE_ACTION_SERVER_H

#include <actionlib/server/simple_action_server.h>
#include <image_processing_pkg/StateChangeRequestAction.h>

class StateChangeRequestAction
{
public:
    StateChangeRequestAction(ros::NodeHandle nh_, std::string name) : as_(nh_, name, boost::bind(&StateChangeRequestAction::executeCB, this, _1), false), action_name_(name)
    {
        as_.start();
    }

    void executeCB(const image_processing_pkg::StateChangeRequestGoalConstPtr &goal);
    
protected:
    actionlib::SimpleActionServer<image_processing_pkg::StateChangeRequestAction> as_; /* Node handle must be created before this line */
    std::string action_name_;


    image_processing_pkg::StateChangeRequestFeedback feedback_;
    image_processing_pkg::StateChangeRequestResult   result_;
};

#endif