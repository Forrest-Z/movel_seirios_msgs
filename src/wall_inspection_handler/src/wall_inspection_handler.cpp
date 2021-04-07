#include <wall_inspection_handler/wall_inspection_handler.h>
//#include <boost/algorithm/string/predicate.hpp>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(task_supervisor::WallInspectionHandler, task_supervisor::TaskHandler);

namespace task_supervisor
{
WallInspectionHandler::WallInspectionHandler()
{}

bool WallInspectionHandler::setupHandler()
{
    if(!loadParams())
    {
        ROS_FATAL("[%s] Error during parameter loading. Shutting down.", name_.c_str());
        return false;
    }

    return true;
}

bool WallInspectionHandler::loadParams()
{
    ros_utils::ParamLoader param_loader(nh_handler_);

    param_loader.get_optional("loop_rate", p_loop_rate_, 5.0);
    param_loader.get_required("wall_inspection_launch_package", p_wall_inspection_launch_package_);
    param_loader.get_required("wall_inspection_launch_file", p_wall_inspection_launch_file_);

    return param_loader.params_valid();
}

bool WallInspectionHandler::onStatus(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res)
{
    res.success = launchStatus(wall_inspection_launch_id_);
    return true;
}

bool WallInspectionHandler::runInspection()
{
    bool running = true;
    ros::Rate r(p_loop_rate_);
    while (ros::ok())
    {
        // Check if cancellation has been called
        if (!isTaskActive())
        {
            ROS_INFO("[%s] Task cancelled, running required cleanup tasks", name_.c_str());
            endInspection();
            ros::Time cancel_start = ros::Time::now();
            while (!stopped_)
            {
                ROS_INFO("[%s] Waiting for wall inspection to end", name_.c_str());
                if (ros::Time::now().toSec() - cancel_start.toSec() > 5.0)
                    break;
                r.sleep();
            }

            stopLaunch(wall_inspection_launch_id_);
            while (launchExists(wall_inspection_launch_id_))
                ;
            //actionlib_msgs::GoalID move_base_cancel;
            //cancel_pub_.publish(move_base_cancel);

            //error_message = "[" + name_ + "] Task cancelled";
            wall_inspection_launch_id_ = 0;
            return false;
        }

        // Check if pause request is received
        if (isTaskPaused() && running)
        {
            pauseInspection(true);
            running = false;
        }
        // Check if resume request is received
        else if (!isTaskPaused() && !running)
        {
            pauseInspection(false);
            running = true;
        }

        // Wall inspection completed
        if (stopped_)
        {
            ROS_INFO("[%s] Wall inspection complete", name_.c_str());
            stopLaunch(wall_inspection_launch_id_);
            while (launchExists(wall_inspection_launch_id_))
                ;
            wall_inspection_launch_id_ = 0;
            return true;
        }
        r.sleep();
    }

    //error_message = "[" + name_ + "] Task failed, ROS was shutdown";
    return false;
}

ReturnCode WallInspectionHandler::runTask(movel_seirios_msgs::Task &task, std::string &error_message)
{
    task_active_ = true;
    task_parsed_ = false;
    start_ = ros::Time::now();
    stopped_ = false;

    ros::ServiceServer status_srv_ = nh_handler_.advertiseService("status", &WallInspectionHandler::onStatus, this);

    // Launch wall inspection node
    ROS_INFO("[%s] Starting wall inspection package: %s, launch file: %s", name_.c_str(), p_wall_inspection_launch_package_.c_str(), p_wall_inspection_launch_file_.c_str());
    wall_inspection_launch_id_ = startLaunch(p_wall_inspection_launch_package_, p_wall_inspection_launch_file_, "");
    if (!wall_inspection_launch_id_)
    {
        ROS_ERROR("[%s] Failed to launch wall inspection launch file", name_.c_str());
        setTaskResult(false);
        return code_;
    }

    ros::Subscriber stopped_sub = nh_handler_.subscribe("/file_transfer/transferred", 1, &WallInspectionHandler::stoppedCB, this);

    pause_client_ = nh_handler_.serviceClient<std_srvs::SetBool>("/wall_inspection/pause");
    //ros::ServiceClient skip_client_ = nh_handler_.serviceClient<std_srvs::Trigger>("/wall_inspection/skip");
    end_client_ = nh_handler_.serviceClient<std_srvs::Trigger>("/wall_inspection/terminate");

    bool inspection_done = runInspection();
    setTaskResult(inspection_done);
    return code_;
}

void WallInspectionHandler::stoppedCB(const std_msgs::EmptyConstPtr& msg)
{
    stopped_ = true;
}

bool WallInspectionHandler::pauseInspection(bool pause)
{
    ROS_INFO("[%s] Pausing wall inspection", name_.c_str());
    std_srvs::SetBool pause_inspection;
    pause_inspection.request.data = pause;
    if(!pause_client_.call(pause_inspection))
    {
        ROS_INFO("Failed to call wall inspection pause/resume service");
        return false;
    }
    else
        return pause_inspection.response.success;
}

bool WallInspectionHandler::endInspection()
{
    ROS_INFO("[%s] Ending wall inspection", name_.c_str());
    std_srvs::Trigger end_inspection;
    if(!end_client_.call(end_inspection))
    {
        ROS_INFO("Failed to call wall inspection terminate service");
        return false;
    }
    else
        return end_inspection.response.success;
}
}