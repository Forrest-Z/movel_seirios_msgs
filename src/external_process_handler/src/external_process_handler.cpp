#include <external_process_handler/external_process_handler.h>
#include <pluginlib/class_list_macros.h>
#include <external_process_handler/json.hpp>

PLUGINLIB_EXPORT_CLASS(external_process_handler::ExternalProcessHandler, task_supervisor::TaskHandler);

using json = nlohmann::json;

namespace external_process_handler
{

bool ExternalProcessHandler::setupHandler()
{
    if(!loadParams())
    {
        ROS_FATAL("[%s] Error during parameter loading. Shutting down.", name_.c_str());
        return false;
    }
    
    return true;
}

bool ExternalProcessHandler::loadParams()
{
    ros_utils::ParamLoader param_loader(nh_handler_);
    
    param_loader.get_required("service_req", p_service_req_);
    
    if(p_service_req_)
    {
        
        param_loader.get_required("service_start", p_service_start_);
        param_loader.get_required("service_stop", p_service_stop_);
        param_loader.get_optional("service_start_msg", p_service_start_msg, std::string(""));
        param_loader.get_optional("service_stop_msg", p_service_stop_msg, std::string(""));
    }
    param_loader.get_required("launch_req", p_launch_req_);
    
    if(p_launch_req_)
    {
        param_loader.get_required("launch_package", p_launch_package_);
        param_loader.get_required("launch_file", p_launch_file_);
        param_loader.get_required("launch_nodes", p_launch_node_);
    }
    param_loader.get_required("topic_cancel_req", p_topic_cancel_req);
    param_loader.get_optional("loop_rate", p_loop_rate_, 5.0);
    param_loader.get_optional("topic_process_cancel", p_topic_process_cancel, std::string("/client/node/cancel"));
    param_loader.get_optional("client_status", p_service_status, std::string("/client/node/status"));
	return param_loader.params_valid();
}

task_supervisor::ReturnCode ExternalProcessHandler::runTask(movel_seirios_msgs::Task& task, std::string& error_message)
{

    task_active_ = true;
    task_parsed_ = false;
    start_ = ros::Time::now();
    isProcessHealthy_ = true;
    process_status_ = 0;
    status_sub = nh_handler_.subscribe(p_service_status,1, &ExternalProcessHandler::clientStatusCb, this);
    ROS_INFO("[%s] Task payload %s", name_.c_str(), task.payload.c_str());

    if(!task.payload.empty())
    {
       json payload = json::parse(task.payload);

       if (payload.find("service_start_msg") != payload.end())
            srv_start_msg = payload["service_start_msg"];
       else
            srv_start_msg = p_service_start_msg;

       if (payload.find("service_stop_msg") != payload.end())
            srv_stop_msg = payload["service_stop_msg"];
       else
            srv_stop_msg = p_service_stop_msg;

    }
    else 
    {
        srv_start_msg = p_service_start_msg;
        srv_stop_msg = p_service_stop_msg;
    }

    if(p_topic_cancel_req)
        cancel_process_ = nh_handler_.advertise<std_msgs::Bool>(p_topic_process_cancel, 1);    
    
    if(p_launch_req_)
    {
        bool launch_process = runLaunch();
        
        if(!launch_process)
        {
            ROS_INFO("[%s] Launch Failed", name_.c_str());
            error_message = "[" + name_ + "] Launch Failed !!!";
            setTaskResult(false);
            return code_;
        } 

        if(!p_service_req_) 
        {
            while (ros::ok())
            {   
                
                if(!isTaskActive())
                {
                    ROS_INFO("[%s] Task cancelled", name_.c_str());
                    cancelProcess();
                    ros::Time cancel_start = ros::Time::now();
                    while (process_status_ != 3)
                    {
                        if (ros::Time::now().toSec() - cancel_start.toSec() > 5.0)
                            break;
                    }

                    stopProcess();
                    error_message = "[" + name_ + "] Task cancelled";
                    setTaskResult(false);
                    return code_;
                }
                else if (process_status_ == 2)         // process completed
                {
                    bool process_done = stopProcess();
                    setTaskResult(process_done);
                    return code_;
                }

                else if (process_status_ == 3)         // process Failed
                {
                    ROS_INFO("[%s] Some nodes are disconnected. Stopping %s", name_.c_str(),name_.c_str());
                    error_message = "[" + name_ + "] Failed !! Cancel the task and try again";
                    setTaskResult(false);
                    cancelProcess();
                    stopProcess();
                    return code_;
                }            

                if (!isProcessHealthy_)
                {
                    ROS_INFO("[%s] Some nodes are disconnected. Stopping %s", name_.c_str(),name_.c_str());
                    cancelProcess();
                    stopProcess();
                    error_message = "[" + name_ + "] Some nodes are disconnected";
                    setTaskResult(false);
                    return code_;
                }
                ros::spinOnce();
                ros::Duration(0.1).sleep();
            }
        }
    }         


    if(p_service_req_)
    { 
        ins_hel_stop_ = false;
        ins_serv_stop_ = false;
        start_process_srv_ = nh_handler_.serviceClient<movel_seirios_msgs::StringTrigger>(p_service_start_);
        stop_process_srv_ = nh_handler_.serviceClient<movel_seirios_msgs::StringTrigger>(p_service_stop_);
        
        // call Process service
        if (!start_process_srv_.waitForExistence(ros::Duration(15.0)))
        {
        ROS_INFO("[%s] Process start service failed to manifest", name_.c_str());
        error_message = "[" + name_ + "] start service failed to manifest !!" ;
        setTaskResult(false);
        stopProcess();
        return code_;
        }
        std::thread t(&ExternalProcessHandler::check_service_health_,this);
        t.detach();
        movel_seirios_msgs::StringTrigger start_process_trigger_;
        start_process_trigger_.request.input=srv_start_msg;
        start_process_srv_.call(start_process_trigger_);
        if(!start_process_trigger_.response.success)
        {
            cancelProcess();
            ROS_INFO("[%s] Error %s", name_.c_str(),start_process_trigger_.response.message.c_str());
            error_message = "[" + name_ + "] Error !!" + start_process_trigger_.response.message.c_str();
            if(!ins_serv_stop_)
            stopProcess();
            setTaskResult(false);
            return code_;
        }
        else 
        {
            bool process_done = stopProcess();
            setTaskResult(process_done);
            return code_;
        }

    }
}

void ExternalProcessHandler::check_service_health_()
{
    while (ros::ok())
    {
        if(!isTaskActive())
        {
            if(!stop_process_srv_.waitForExistence(ros::Duration(5.0)))
            {
                ROS_INFO("[%s] Service Health Detach", name_.c_str());
                cancelProcess();
                break;
            } 
            else
            {
                if(!ins_serv_stop_){
                    ins_hel_stop_= true;
                    ROS_INFO("[%s] Task cancelled in this function", name_.c_str());
                    ros::Time cancel_start = ros::Time::now();
                    cancelProcess();
                    movel_seirios_msgs::StringTrigger stop_process_trigger_;
                    stop_process_trigger_.request.input=srv_stop_msg;
                    stop_process_srv_.call(stop_process_trigger_);
                }
            }
            break;
        }
        ros::spinOnce();
        ros::Duration(1).sleep();
    }
}



bool ExternalProcessHandler::runLaunch()
{
    ROS_INFO("[%s] Starting package: %s, launch file: %s", name_.c_str(), p_launch_package_.c_str(), p_launch_file_.c_str());

    process_launch_id = startLaunch(p_launch_package_, p_launch_file_, "");
    if(!process_launch_id)
    {
        ROS_ERROR("[%s] Failed to launch %s launch file", name_.c_str(),p_launch_file_.c_str() );
        return false;
    }
    return true;
}

bool ExternalProcessHandler::stopProcess()
{
    if(p_service_req_)
    {
        
        if(!ins_hel_stop_){
            ins_serv_stop_ = true;
            movel_seirios_msgs::StringTrigger stop_process_trigger_;
            stop_process_trigger_.request.input=srv_stop_msg;
            if(stop_process_srv_.waitForExistence(ros::Duration(3.0)))
                stop_process_srv_.call(stop_process_trigger_);
            ROS_INFO("[%s] Stopping Process Service", name_.c_str());
            ROS_INFO("[%s] Service Response : %s", name_.c_str(),stop_process_trigger_.response.message.c_str());
        }
    }
   
    if(p_launch_req_) 
    {    
        ROS_INFO("[%s] Stopping %s", name_.c_str(),name_.c_str());
        stopLaunch(process_launch_id, p_launch_node_);
        process_launch_id = 0;
        ROS_INFO("[%s] %s stopped", name_.c_str(),name_.c_str());
    }

	return true;
}

void ExternalProcessHandler::clientStatusCb(const std_msgs::UInt8ConstPtr& msg)
{
    process_status_ = msg->data;
}

bool ExternalProcessHandler::cancelProcess()
{
    if(p_topic_cancel_req)
    {
        std_msgs::Bool cancel;
        cancel.data = true;
        cancel_process_.publish(cancel);

    }
    cancelTask();
    return true;
}


bool ExternalProcessHandler::healthCheck()
{

    if(p_launch_req_) 
    {
        static int failcount = 0;
        if (process_launch_id == 0)
        {
            failcount = 0;
            return true;
        }
        bool isHealthy = launchStatus(process_launch_id);
        if (!isHealthy && process_launch_id)
        {
            failcount += 1;
            if (failcount >= 60*p_watchdog_rate_)
            {
                isProcessHealthy_ = false;
                if(p_service_req_)
                {
                    ROS_INFO("[%s] Some nodes are disconnected. Stopping %s", name_.c_str(),name_.c_str());
                    cancelProcess();
                    stopProcess();
                    setTaskResult(false);
                    return false;            
                }
                failcount = 0;
                process_launch_id = 0;
            }
        }
        else
            failcount = 0;
        return isHealthy;
    }
    return true;
}
}