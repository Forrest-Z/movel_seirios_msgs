#ifndef WAIT_FOR_GO_HANDLER_H
#define WAIT_FOR_GO_HANDLER_H

#include <ros/ros.h>
#include <task_supervisor/plugins/task_handler.h>
#include <task_supervisor/common.h>
#include <movel_seirios_msgs/Task.h>
#include <std_msgs/Bool.h>

namespace wait_for_go_handler
{
class WaitForGoHandler: public task_supervisor::TaskHandler
{
protected:
    bool setupHandler();
    void statusCb(std_msgs::Bool msg);

    bool go_;
    
public:
    WaitForGoHandler();

    virtual task_supervisor::ReturnCode runTask(movel_seirios_msgs::Task& task, std::string& error_message);
    ros::Subscriber status_sub;
};
}

#endif