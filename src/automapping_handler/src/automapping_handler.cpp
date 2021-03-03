#include <automapping_handler/automapping_handler.h>
#include <boost/algorithm/string/predicate.hpp>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(task_supervisor::AutomappingHandler, task_supervisor::TaskHandler);

namespace task_supervisor
{
AutomappingHandler::AutomappingHandler()
{}

bool AutomappingHandler::setupHandler()
{
    if(!loadParams())
    {
        ROS_FATAL("[%s] Error during parameter loading. Shutting down.", name_.c_str());
        return false;
    }

    return true;
}

bool AutomappingHandler::loadParams()
{
    ros_utils::ParamLoader param_loader(nh_handler_);

    param_loader.get_optional("loop_rate", p_loop_rate_, 5.0);
    param_loader.get_optional("save_timeout", p_save_timeout_, 5.0);
    param_loader.get_optional("map_topic", p_map_topic_, std::string("/map"));

    param_loader.get_required("automapping_launch_package", p_automapping_launch_package_);
    param_loader.get_required("automapping_launch_file", p_automapping_launch_file_);

    return param_loader.params_valid();
}

bool AutomappingHandler::onStatus(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res)
{
    res.success = launchStatus(automapping_launch_id_);
    return true;
}

bool AutomappingHandler::onSaveServiceCall(movel_seirios_msgs::StringTrigger::Request& req,
                                           movel_seirios_msgs::StringTrigger::Response& res)
{
    res.success = saveMap(req.input);
    saved_ = true;
    return true;
}

bool AutomappingHandler::onAsyncSave(movel_seirios_msgs::StringTrigger::Request& req,
                                     movel_seirios_msgs::StringTrigger::Response& res)
{
    res.success = saveMap(req.input);
    return true;
}

bool AutomappingHandler::saveMap(std::string map_name)
{
    // Set path to save file
    std::string launch_args = " map_topic:=" + p_map_topic_;
    if (!map_name.empty())
        launch_args = launch_args + " file_path:=" + map_name;

    // Call map saving through launch manager service
    ROS_INFO("[%s] Saving map %s", name_.c_str(), map_name.size() != 0 ? ("to" + map_name).c_str() : "");
    unsigned int map_saver_id = startLaunch("task_supervisor", "map_saver.launch", launch_args);

    // Check if startLaunch succeeded
    if (!map_saver_id)
    {
        ROS_ERROR("[%s] Failed to start map saver", name_.c_str());
        return false;
    }

    // While loop until timeout
    ros::Time start_time = ros::Time::now();
    ros::Rate r(p_loop_rate_);
    while (ros::Time::now().toSec() - start_time.toSec() < p_save_timeout_)
    {
        // TODO map_saver might die before saving
        if (!launchExists(map_saver_id))
        {
            ROS_INFO("[%s] Save complete", name_.c_str());
            return true;
        }

        r.sleep();
    }

    ROS_WARN("[%s] Timeout occurred, save failed", name_.c_str());
    stopLaunch(map_saver_id);
    return false;
}

bool AutomappingHandler::runAutomapping()
{
    ROS_INFO("[%s] Starting automapping package: %s, launch file: %s", name_.c_str(), p_automapping_launch_package_.c_str(),
           p_automapping_launch_file_.c_str());

    // Run mapping asynchronously
    automapping_launch_id_ = startLaunch(p_automapping_launch_package_, p_automapping_launch_file_, "");
    if (!automapping_launch_id_)
    {
        ROS_ERROR("[%s] Failed to launch automapping launch file", name_.c_str());
        return false;
    }

    // Loop until save callback is called
    ros::Rate r(p_loop_rate_);
    while (!saved_ && ros::ok())
    {
        // Check if cancellation has been called
        if (!isTaskActive())
        {
            ROS_INFO("[%s] Waiting for automapping thread to exit", name_.c_str());
            stopLaunch(automapping_launch_id_);
            while (launchExists(automapping_launch_id_))
                ;
            automapping_launch_id_ = 0;
            return false;
        }

        // Check if pause request is received
        if (isTaskPaused())
        {
            while (isTaskPaused())
            {
                vel_pub_.publish(geometry_msgs::Twist());
                r.sleep();
            }
        }

        /*if (ended_)
        {
            saveMap("automap");
            ROS_INFO("[%s] Automapping complete, waiting for automapping thread to exit", name_.c_str());
            stopLaunch(automapping_launch_id_);
            while (launchExists(automapping_launch_id_))
                ;
            return true;
        }*/

        r.sleep();
    }

    ROS_INFO("[%s] Stopping automapping", name_.c_str());
    stopLaunch(automapping_launch_id_);
    while (launchExists(automapping_launch_id_))
        ;
    automapping_launch_id_ = 0;
    ROS_INFO("[%s] Automapping stopped", name_.c_str());

    //saved_ = false;
    return true;
}

ReturnCode AutomappingHandler::runTask(movel_seirios_msgs::Task &task, std::string &error_message)
{
    task_active_ = true;
    task_parsed_ = false;
    start_ = ros::Time::now();
    //ended_ = false;
    saved_ = false;

    vel_pub_ = nh_handler_.advertise<geometry_msgs::Twist>("/cmd_vel_mux/safety", 1);
    stopped_pub_ = nh_handler_.advertise<std_msgs::Empty>("stopped", 1);
    ros::Subscriber status_sub_ = nh_handler_.subscribe("/rosout",1, &AutomappingHandler::logCB, this);
    ros::ServiceServer serv_status_ = nh_handler_.advertiseService("status", &AutomappingHandler::onStatus, this);
    ros::ServiceServer serv_save_ = nh_handler_.advertiseService("save_map", &AutomappingHandler::onSaveServiceCall, this);
    ros::ServiceServer serv_save_async_ =
      nh_handler_.advertiseService("save_map_async", &AutomappingHandler::onAsyncSave, this);

    bool mapping_done = runAutomapping();
    setTaskResult(mapping_done);
    return code_;
}

void AutomappingHandler::logCB(const rosgraph_msgs::LogConstPtr& msg)
{
    // Retrieve log message
    if (msg->name == "/explore" && msg->level == 2)
    {
        std::size_t found;
        found = msg->msg.find("Exploration stopped.");
        if (found != std::string::npos)
        {
            //ended_ = true;
            ROS_INFO("[%s] Automapping complete.", name_.c_str());
            stopped_pub_.publish(std_msgs::Empty());
        }
    }
}
}
