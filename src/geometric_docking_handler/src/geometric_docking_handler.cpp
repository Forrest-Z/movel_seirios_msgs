#include <geometric_docking_handler/geometric_docking_handler.h>
#include <boost/algorithm/string/predicate.hpp>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(geometric_docking_handler::GeometricDockingHandler, task_supervisor::TaskHandler);
namespace geometric_docking_handler
{
GeometricDockingHandler::GeometricDockingHandler() : docking_(false){};

bool GeometricDockingHandler::setupHandler()
{
    if(!loadParams())
    {
        ROS_FATAL("[%s] Error during parameter loading. Shutting down.", name_.c_str());
        return false;
    }

    start_dock = nh_handler_.serviceClient<std_srvs::Trigger>("/geometric_docking_node/startDocking");
    stop_dock = nh_handler_.serviceClient<std_srvs::Trigger>("/geometric_docking_node/stopDocking");
    resume_dock = nh_handler_.serviceClient<std_srvs::Trigger>("/geometric_docking_node/resumeDocking"); 
    pause_dock = nh_handler_.serviceClient<std_srvs::Trigger>("/geometric_docking_node/pauseDocking"); 

    dock_status = nh_handler_.subscribe("/geometric_docking_node/status",1, &GeometricDockingHandler::dockStatusCb, this);

    health_report_sub_ = nh_handler_.subscribe("/task_supervisor/health_report", 1, &GeometricDockingHandler::healthReportCb, this);
    health_report_pub_ = nh_handler_.advertise<movel_seirios_msgs::Reports>("/task_supervisor/health_report", 1);

    return true;
}

bool GeometricDockingHandler::loadParams()
{
    ros_utils::ParamLoader param_loader(nh_handler_);

    param_loader.get_optional("loop_rate", p_loop_rate_, 5.0);
    param_loader.get_required("geometric_launch_package", p_geometric_launch_package_);
    param_loader.get_required("geometric_launch_file", p_geometric_launch_file_);
    param_loader.get_required("geometric_launch_nodes", p_geometric_launch_node_);

    return param_loader.params_valid();
}

void GeometricDockingHandler::startDocking()
{  
    ROS_INFO("[%s] Starting geometric docking package: %s, launch file: %s", name_.c_str(), p_geometric_launch_package_.c_str(), p_geometric_launch_file_.c_str());
    
    // Launch geometric_docking launch file
    geometric_launch_id = startLaunch(p_geometric_launch_package_, p_geometric_launch_file_, "");
    if(!geometric_launch_id)
    {
        ROS_ERROR("[%s] Failed to launch docking launch file", name_.c_str());
    }
    
    // Calling the start service
    ros::service::waitForService("/geometric_docking_node/startDocking");
    std_srvs::Trigger service_call;
    if (start_dock.call(service_call))
    {
        ROS_INFO("[%s] Docking task started", name_.c_str());
    }
    dock_status_ = 1;       // Docking is active
    docking_ = true;

}

void GeometricDockingHandler::stopAllLaunch()
{
    // Terminate geometric_docking node
	// stopLaunch(geometric_launch_id, p_geometric_launch_node_);
    stopLaunch(geometric_launch_id);
	while(launchExists(geometric_launch_id));
	geometric_launch_id = 0;
}

task_supervisor::ReturnCode GeometricDockingHandler::runTask(movel_seirios_msgs::Task& task, std::string& error_message)
{
    task_active_ = true;
    task_parsed_ = false;
    start_ = ros::Time::now();
    dock_status_ = 0;       // docking is not active
    docking_ = false;

    ROS_INFO("[%s] Task payload %s",name_.c_str(),task.payload.c_str());

    // Checking the payload with non-case sensitive boost library
    bool result;
    if (boost::iequals(task.payload, "start"))
        startDocking();
    else{
        result = false;
        error_message = "[" + name_ + "] Payload command format invalid, input must be 'start'";
        setTaskResult(result);
        return code_;
    }

    ros::Rate r(p_loop_rate_);
    while (ros::ok())
    {

        if(!isTaskActive())         // docking cancelled
        {
            ROS_INFO("[%s] Task cancelled, running required cleanup tasks", name_.c_str());
            cancelDocking();
            // while(dock_status_ != 3)
            //     ;
            dock_status_ = 0;

            stopAllLaunch();
            error_message = "[" + name_ + "] Task cancelled";
            setTaskResult(false);
            dock_status_ = 0;
            return code_;
        }
        
        if (docking_)
        {
            if (dock_status_ == 2)      // docking completed
            {
                ROS_INFO("[%s] Docking completed", name_.c_str());
                stopAllLaunch();
                setTaskResult(true);
                return code_;
            }

            if (dock_status_ == 3)      // docking failed
            {
                ROS_INFO("[%s] Task failed, the robot fails to dock", name_.c_str());
                stopAllLaunch();
                error_message = "[" + name_ + "] Task stopped";
                setTaskResult(false);
                return code_;
            }
        }
        
        if(isTaskPaused() && docking_)      // Pause is triggered
        {
            docking_ = false;
            pauseDocking();
        }
        else if (!isTaskPaused() && !docking_)         // Resume is triggered
        {
            docking_ = true;
            resumeDocking();
        }
        r.sleep();
    }
    error_message = "[" + name_ + "] Task failed, ROS was shutdown";
    setTaskResult(false);
    return code_;
}

void GeometricDockingHandler::dockStatusCb(const std_msgs::UInt8ConstPtr& msg)
{
    // Retrieve status message
    dock_status_ = msg->data;
}

bool GeometricDockingHandler::cancelDocking()
{
    docking_ = false;
    std_srvs::Trigger stop_call;
    if(!stop_dock.call(stop_call))
    {
        ROS_INFO("[%s] Failed to call geometric docking service", name_.c_str());
        return false;
    }
    else
        return stop_call.response.success;
}

bool GeometricDockingHandler::pauseDocking()
{
    ROS_INFO("[%s] Pausing geometric docking", name_.c_str());
    std_srvs::Trigger pause_call;
    if(!pause_dock.call(pause_call))
    {
        ROS_INFO("[%s] Failed to call geometric docking pause service", name_.c_str());
        return false;
    }
    else
        return pause_call.response.success;
}

bool GeometricDockingHandler::resumeDocking()
{
    ROS_INFO("[%s] Resuming geometric docking", name_.c_str());
    std_srvs::Trigger resume_call;
    if(!resume_dock.call(resume_call))
    {
        ROS_INFO("[%s] Failed to resume geometric docking", name_.c_str());
        return false;
    }
    else
        return resume_call.response.success;

}

void GeometricDockingHandler::healthReportCb(const movel_seirios_msgs::ReportsConstPtr &msg)
{
    if (msg->handler == "localization_handler")
    {
        loc_is_healthy_ = msg->healthy;
    }
}

bool GeometricDockingHandler::healthCheck()
{
    static int fail_count = 0;
    if (docking_)
    {
        bool healthy = launchStatus(geometric_launch_id);

        if (!healthy || !loc_is_healthy_)
        {
            fail_count += 1;
            if (fail_count >= 2*p_watchdog_rate_)
            {
                // prep report
                ROS_INFO("[%s] some nodes are dead", name_.c_str());
                movel_seirios_msgs::Reports health_report;
                health_report.header.stamp = ros::Time::now();
                health_report.handler = "geometric_docking_handler";
                health_report.task_type = task_type_;
                health_report.healthy = false;
                health_report.message = "one or more nodes in geometric_docking task have failed";
                health_report_pub_.publish(health_report);

                // stop task
                docking_ = false;
                fail_count = 0;
                // cancelTask();
                // stopLaunch(geometric_launch_id);
                
                setTaskResult(false);
                return false;
            }
        }
        else
            fail_count = 0;
    }
    return true;
}
}