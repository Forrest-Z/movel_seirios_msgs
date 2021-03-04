#include <docking_handler/docking_handler.h>
#include <docking_handler/json.hpp>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(docking_handler::DockingHandler, task_supervisor::TaskHandler);

using json = nlohmann::json;

namespace docking_handler
{

DockingHandler::DockingHandler() : dock_status_(false), docking_(false)
{}

bool DockingHandler::setupHandler()
{
    if(!loadParams())
    {
        ROS_FATAL("[%s] Error during parameter loading. Shutting down.", name_.c_str());
        return false;
    }

    start_dock = nh_handler_.serviceClient<camera_lidar_docking_pallet::StartAutoDock>("/autodock_pallet/StartAutoDocking");
    resume_dock = nh_handler_.advertise<std_msgs::Bool>("/autodock_pallet/resumeDocking", 1); 
    pause_dock = nh_handler_.advertise<std_msgs::Bool>("/autodock_pallet/pauseDocking", 1); 
    cancel_dock = nh_handler_.advertise<std_msgs::Bool>("/autodock_pallet/cancelDocking", 1);
    status_sub = nh_handler_.subscribe("/autodock_pallet/status",1, &DockingHandler::dockStatusCb, this);
    return true;
}

bool DockingHandler::loadParams()
{
    ros_utils::ParamLoader param_loader(nh_handler_);

	param_loader.get_required("docking_launch_package", p_docking_launch_package_);
	param_loader.get_required("docking_launch_file", p_docking_launch_file_);
    param_loader.get_required("docking_launch_nodes", p_docking_launch_node_);

	return param_loader.params_valid();
}

task_supervisor::ReturnCode DockingHandler::runTask(movel_seirios_msgs::Task& task, std::string& error_message)
{
    task_active_ = true;
    task_parsed_ = false;
    start_ = ros::Time::now();

    bool launch_docking = runDocking();
    dock_status_ = 0;
    ROS_INFO("[%s] Task payload %s",name_.c_str(),task.payload.c_str());

    json recv_payload =  json::parse(task.payload);
    
    if(recv_payload.find("x") != recv_payload.end())
    {
        camera_lidar_docking_pallet::StartAutoDock service_call;
        
        service_call.request.x          = recv_payload["x"].get<float>();
        service_call.request.y          = recv_payload["y"].get<float>();
        service_call.request.theta      = recv_payload["theta"].get<float>();
        service_call.request.operation  = recv_payload["operation"].get<std::string>();
        service_call.request.id_pallet  = recv_payload["id_pallet"].get<int>();
        
        ros::service::waitForService("/autodock_pallet/StartAutoDocking");
        
        if(!start_dock.call(service_call))
        {
            ROS_INFO("[%s] Failed to call docking package.", name_.c_str());
        }

        docking_ = true;
        
        while (ros::ok())
        {   
            if(!isTaskActive())
            {
                ROS_INFO("[%s] Task cancelled", name_.c_str());
                cancelDocking();
                while (dock_status_ != 3)
                ;

                stopDocking();
                error_message = "[" + name_ + "] Task cancelled";
                setTaskResult(false);
                return code_;
            }
            else if(isTaskPaused() && docking_)      // Pause is triggered
            {
                docking_ = false;
                pauseDocking();
            }
            else if (!isTaskPaused() && !docking_)         // Resume is triggered
            {
                docking_ = true;
                resumeDocking();
            }
            else if (dock_status_ == 2)         // docking completed
            {
                bool dockng_done = stopDocking();
                setTaskResult(dockng_done);
                dock_status_ = false;
                return code_;
            }
            ros::spinOnce();
            ros::Duration(0.1).sleep();
        }
    }
    else
    {
      setMessage("Malformed payload Example: {\"x\": 0.0, \"y\": 0.0, \"theta\": 0.0, \"operation\": 'pickup' ,\"id_pallet\": 0}"  );
      error_message = message_;
      setTaskResult(false);
    }
    return code_;
}

bool DockingHandler::runDocking()
{
    ROS_INFO("[%s] Starting Docking package: %s, launch file: %s", name_.c_str(), p_docking_launch_package_.c_str(), p_docking_launch_file_.c_str());

    docking_launch_id = startLaunch(p_docking_launch_package_, p_docking_launch_file_, "");
    if(!docking_launch_id)
    {
        ROS_ERROR("[%s] Failed to launch docking launch file", name_.c_str());
        return false;
    }
    return true;
}

bool DockingHandler::stopDocking()
{
    ROS_INFO("[%s] Stopping docking", name_.c_str());
	stopLaunch(docking_launch_id, p_docking_launch_node_);
	while(launchExists(docking_launch_id));
	docking_launch_id = 0;
	ROS_INFO("[%s] Docking stopped", name_.c_str());

	return true;
}

bool DockingHandler::pauseDocking()
{
    ROS_INFO("[%s] Pausing docking", name_.c_str());
    std_msgs::Bool pause;
    pause.data = true;
    pause_dock.publish(pause);
    return true;
}

bool DockingHandler::resumeDocking()
{
    ROS_INFO("[%s] Resuming docking", name_.c_str());
    std_msgs::Bool resume;
    resume.data = true;
    resume_dock.publish(resume);
    return true;
}

void DockingHandler::dockStatusCb(const std_msgs::UInt8ConstPtr& msg)
{
    dock_status_ = msg->data;
}

bool DockingHandler::cancelDocking()
{
    std_msgs::Bool cancel;
    cancel.data = true;
    cancel_dock.publish(cancel);
    return true;
}

}