#include <wait_for_go_handler/wait_for_go.h>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(wait_for_go_handler::WaitForGoHandler, task_supervisor::TaskHandler);

namespace wait_for_go_handler
{

WaitForGoHandler::WaitForGoHandler() : go_(false)
{}

bool WaitForGoHandler::setupHandler()
{
    status_sub = nh_handler_.subscribe("/task_supervisor/wait_for_go/go", 1, &WaitForGoHandler::statusCb, this);
    return true;
}

task_supervisor::ReturnCode WaitForGoHandler::runTask(movel_seirios_msgs::Task& task, std::string& error_message)
{
    task_active_ = true;
    task_parsed_ = false;
    start_ = ros::Time::now();
    go_ = false;

    ROS_INFO("[%s] Waiting mode is ON!", name_.c_str());
    ROS_INFO("[%s] Waiting for -go- signal from user to run a new task", name_.c_str());
    
    while(ros::ok())
    {
        if(!isTaskActive())
        {
            ROS_INFO("[%s] Task cancelled", name_.c_str());
            error_message = "[" + name_ + "] Task cancelled";
            setTaskResult(false);
            return code_;
        }
        else if(go_)
        {
            ROS_INFO("[%s] -Go- signal has been received", name_.c_str());
            ROS_INFO("[%s] Waiting mode is OFF!", name_.c_str());
            ROS_INFO("[%s] Running a new task!", name_.c_str());
            setTaskResult(true);
            return code_;
        }
        ros::spinOnce();
        ros::Duration(0.1).sleep();
    }
    return code_;
}

void WaitForGoHandler::statusCb(std_msgs::Bool msg)
{
    if (msg.data == true)
        go_ = true;
}

}