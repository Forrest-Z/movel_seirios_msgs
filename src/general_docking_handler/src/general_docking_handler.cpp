#include <general_docking_handler/general_docking_handler.h>
#include <pluginlib/class_list_macros.h>
#include <movel_common_libs/json.hpp>

using json = nlohmann::json;

PLUGINLIB_EXPORT_CLASS(general_docking_handler::GeneralDockingHandler, task_supervisor::TaskHandler);

namespace general_docking_handler
{
  GeneralDockingHandler::GeneralDockingHandler() : odom_received_(false), external_process_running_(false), rack_args_("")
  {}

  bool GeneralDockingHandler::setupHandler()
  {
    if (!loadParams())
      return false;

    if(dock_)
    {
      internal_feedback_sub_ = nh_handler_.subscribe(internal_topic_, 1, &GeneralDockingHandler::internalCb, this);
      pause_docking_client_ = nh_handler_.serviceClient<std_srvs::SetBool>(pause_service_);
      if(use_external_feedback_)
        external_feedback_sub_ = nh_handler_.subscribe(external_topic_, 1, &GeneralDockingHandler::externalCb, this);
    }

    if(use_external_service_)
    {
      external_process_client_ = nh_handler_.serviceClient<std_srvs::Trigger>(external_service_);
      external_cancel_pub_ = nh_handler_.advertise<std_msgs::Empty>(external_cancel_topic_, 1);
    }

    odom_sub_ = nh_handler_.subscribe(odom_topic_, 1, &GeneralDockingHandler::odomCb, this);
    vel_pub_ = nh_handler_.advertise<geometry_msgs::Twist>("/cmd_vel_mux/autonomous", 1);
    health_check_pub_ = nh_handler_.advertise<movel_seirios_msgs::Reports>("/task_supervisor/health_report", 1);
    
    enable_smoother_client_ = nh_handler_.serviceClient<std_srvs::SetBool>("/velocity_smoother/velocity_smoother_on");
    smoother_status_client_ = nh_handler_.serviceClient<std_srvs::Trigger>("/velocity_smoother/get_status");

    return true;
  }

  bool GeneralDockingHandler::loadParams()
  {
    ros_utils::ParamLoader param_loader(nh_handler_);

    param_loader.get_required("dock", dock_);
    param_loader.get_required("loop_rate", loop_rate_);
    param_loader.get_required("odom_topic", odom_topic_);
    param_loader.get_required("use_external_service", use_external_service_);
    param_loader.get_required("disable_smoother", disable_smoother_);

    if(use_external_service_)
    {
      param_loader.get_required("external_service", external_service_);
      param_loader.get_required("external_cancel_topic", external_cancel_topic_);
    }

    if(dock_)
    {
      param_loader.get_required("use_external_feedback", use_external_feedback_);
      if(use_external_feedback_)
      {
        param_loader.get_required("external_topic", external_topic_);
        param_loader.get_required("feedback_timeout", feedback_timeout_);
      }
      param_loader.get_required("internal_topic", internal_topic_);
      param_loader.get_required("pause_service", pause_service_);
      param_loader.get_required("docking_launch_package", launch_pkg_);
      param_loader.get_required("docking_launch_file", launch_file_);
      param_loader.get_required("camera_name", camera_name_);
      param_loader.get_required("enable_retry", enable_retry_);
    }

    param_loader.get_required("undocking_distance", undocking_distance_);
    param_loader.get_required("undocking_speed", undocking_speed_);

    return param_loader.params_valid();
  }

  std::string GeneralDockingHandler::startTask()
  {
    std::string error_message;
    // Call external service
    if (!dock_ && use_external_service_)
    {
      error_message = "[" + name_ + "] " + externalProcess();
      if(!healthy_ || task_cancelled_)
      {
        geometry_msgs::Twist stop;
        vel_pub_.publish(stop);
        return error_message;
      }
    }

    // Wait for odom data
    ros::Time start_time = ros::Time::now();
    while(!odom_received_)
    {
      if(ros::Time::now().toSec() - start_time.toSec() > 1.0)
        break;
    }

    // Undock
    if(odom_received_ && !dock_)
    {
      if(!startUndock() || task_cancelled_)
      {
        geometry_msgs::Twist stop;
        vel_pub_.publish(stop);
        return error_message;
      }
    }
    else if(!odom_received_ && !dock_)
    {
      healthy_ = false;
      error_message = "[" + name_ + "] No odometry data received";
      return error_message;
    }

    // Call external service
    if(dock_)
    {
      error_message = "[" + name_ + "] " + startDock();
      if(!healthy_ || task_cancelled_)
      {
        geometry_msgs::Twist stop;
        vel_pub_.publish(stop);
        return error_message;
      }

      if (use_external_service_)
      {
        error_message = "[" + name_ + "] " + externalProcess();
        if(!healthy_ || task_cancelled_)
        {
          geometry_msgs::Twist stop;
          vel_pub_.publish(stop);
          return error_message;
        }
      }
    }
    return error_message;
  }

  task_supervisor::ReturnCode GeneralDockingHandler::runTask(movel_seirios_msgs::Task& task,
                                                           std::string& error_message)
  {
    task_active_ = true;
    task_paused_ = false;
    task_parsed_ = false;
    healthy_ = true;
    task_cancelled_ = false;
    goal_received_ = false;

    if(!task.payload.empty())
    {
      if(launch_file_.find("pose") != std::string::npos)
      {
        ROS_INFO("[%s] Mode: Pose Docking.", name_.c_str());

        json payload = json::parse(task.payload);
        if (payload.find("position") != payload.end())
        {
          goal_pose_.header.frame_id = "map";
          goal_pose_.pose.position.x = payload["position"]["x"].get<float>();
          goal_pose_.pose.position.y = payload["position"]["y"].get<float>();
          goal_pose_.pose.position.z = payload["position"]["z"].get<float>();
          goal_pose_.pose.orientation.x = payload["orientation"]["x"].get<float>();
          goal_pose_.pose.orientation.y = payload["orientation"]["y"].get<float>();
          goal_pose_.pose.orientation.z = payload["orientation"]["z"].get<float>();
          goal_pose_.pose.orientation.w = payload["orientation"]["w"].get<float>();
          goal_received_ = true;
        }
        else
        {
          error_message = "[" + name_ + "] Payload command format invalid, use the following format: {\"position\":{\"x\":-4,\"y\":0.58,\"z\":0}, "
                          "\"orientation\":{\"x\":0,\"y\":0,\"z\":0.71,\"w\":0.69}}";
          setTaskResult(false);
          return code_;
        }
      }
      else
      {
        ROS_INFO("[%s] Mode: Rack / Charge Docking.", name_.c_str());
        bool is_valid_payload = {false};

        json payload = json::parse(task.payload);
        rack_args_ = "";

        if (payload.find("final_xy_tolerance") != payload.end())
        {
          is_valid_payload = true;

          // add payload
          rack_args_ = " final_xy_tolerance:=" + std::to_string(payload["final_xy_tolerance"].get<float>());
        }
        if (payload.find("final_yaw_tolerance") != payload.end())
        {
          is_valid_payload = true;
          // add payload
          rack_args_ = rack_args_ + " final_yaw_tolerance:=" + std::to_string(payload["final_yaw_tolerance"].get<float>());
        }
        if (payload.find("max_linear_vel") != payload.end())
        {
          is_valid_payload = true;
          // add payload
          rack_args_ = rack_args_ + " max_linear_vel:=" + std::to_string(payload["max_linear_vel"].get<float>());
        }
        if (payload.find("max_turn_vel") != payload.end())
        {
          is_valid_payload = true;
          // add payload
          rack_args_ = rack_args_ + " max_turn_vel:=" + std::to_string(payload["max_turn_vel"].get<float>());
        }
        if (payload.find("tag_bundles") != payload.end())
        {
          is_valid_payload = true;
          // add payload
          rack_args_ = rack_args_ + " tag_bundles:=" + payload["tag_bundles"].get<std::string>(); //not sure, need to check this
        }

        if (!is_valid_payload)
        {
          error_message = "[" + name_ + "] Rack Docking Payload command format invalid, input 'dock', 'undock' or the following format: {\"final_xy_tolerance\":0.5, \"final_yaw_tolerance\":0.5, \"max_linear_vel\":0.5, \"max_turn_vel\":0.5, \"tag_bundles\": \"[{name: 'tag_0',layout:[{id: 0, size: 0.10, x: -0.1},{id: 1, size: 0.10, x: 0.1}]},]\"}";
          setTaskResult(false);
          return code_;
        }
      }
    }

    // Check if velocity_smoother is on
    if(disable_smoother_)
    {
      std_srvs::Trigger smoother_status;
      if(!smoother_status_client_.call(smoother_status))
      {
        ROS_WARN("[%s] Failed to call /velocity_smoother/check_status", name_.c_str());
        smoother_on_ = false;
      }
      else
        smoother_on_ = smoother_status.response.success;
    }
    else
    {
      smoother_on_ = false;
    }

    // Disable velocity_smoother
    std_srvs::SetBool enable_smoother;
    if(smoother_on_ && disable_smoother_)
    {
      enable_smoother.request.data = false;
      if(!enable_smoother_client_.call(enable_smoother))
      {
        ROS_WARN("[%s] Failed to call /velocity_smoother/velocity_smoother_on", name_.c_str());
      }
    }

    // Do task
    error_message = startTask();

    // Re-enable velocity_smoother
    if(smoother_on_ && disable_smoother_)
    {
      enable_smoother.request.data = true;
      if(!enable_smoother_client_.call(enable_smoother))
      {
        ROS_WARN("[%s] Failed to call /velocity_smoother/velocity_smoother_on", name_.c_str());
      }
    }

    if(healthy_)
      error_message.clear();
    setTaskResult(healthy_);
    return code_;
  }

  std::string GeneralDockingHandler::startDock()
  {
    ROS_INFO("[%s] Start docking", name_.c_str());
    std::string error_message;
    // launch nodes
    std::string arg;
    if(!goal_received_) //rack / charge docking
    {
      arg = " camera:=" + camera_name_ + rack_args_;
      rack_args_ = ""; //reset
    }
    else
      arg = " x:=" + std::to_string(goal_pose_.pose.position.x) + " y:=" + std::to_string(goal_pose_.pose.position.y) +
                        " z:=" + std::to_string(goal_pose_.pose.orientation.z) + " w:=" + std::to_string(goal_pose_.pose.orientation.w);
    docking_launch_id_ = startLaunch(launch_pkg_, launch_file_, arg);
    if (!docking_launch_id_)
    {
      error_message = "Failed to launch docking nodes";
      healthy_ = false;
      return error_message;
    }

    ros::Duration(1.0).sleep();

    docking_success_ = false;
    task_done_ = false;
    docking_success_internal_ = false;
    docking_success_external_ = false;
    task_done_internal_ = false;
    task_done_external_ = false;
    waiting_ = false;
    bool paused = false;
    std_srvs::SetBool pause_srv;

    // wait for completion loop
    ros::Rate r(loop_rate_);
    while(ros::ok())
    {
      // active check
      if (task_cancelled_)
      {
        if (docking_launch_id_)
        {
          stopLaunch(docking_launch_id_);
	        docking_launch_id_ = 0;
        }
        geometry_msgs::Twist stop;
        vel_pub_.publish(stop);
        break;
      }

      // pause check
      if (isTaskPaused() && !paused) // pause rising edge
      {
        pause_srv.request.data = true;
        pause_docking_client_.call(pause_srv);
        paused = true;
      }
      // resume check
      else if (!isTaskPaused() && paused) // pause falling edge
      {
        pause_srv.request.data = false;
        pause_docking_client_.call(pause_srv);
        paused = false;
      }

      // success check
      if(!use_external_feedback_)
      {
        task_done_ = task_done_internal_;
        docking_success_ = docking_success_internal_;
      }
      else
      {
        if(task_done_external_)
        {
          task_done_ = task_done_external_;
          docking_success_ = docking_success_external_;
        }
        else
        {
          // If docking motion is complete, wait for external feedback
          if(task_done_internal_)
          {
            start_wait_time_ = ros::Time::now();
            waiting_ = true;
            task_done_internal_ = false;
            ROS_INFO("[%s] Wait for external feedback", name_.c_str());
            //task_done_ = task_done_internal_;
            //docking_success_ = false;
          }
          else
          {
            if(waiting_ && ros::Time::now().toSec() - start_wait_time_.toSec() > feedback_timeout_)
            {
              task_done_ = true;
              docking_success_ = false;
              ROS_WARN("[%s] Timed out waiting for external feedback", name_.c_str());
            }
          }
        }
      }

      // Task end
      if (task_done_)
      {
        if (docking_launch_id_)
        {
          stopLaunch(docking_launch_id_);
	        docking_launch_id_ = 0;
        }
        if (!docking_success_)
        {
          if(!enable_retry_)
          {
            healthy_ = false;
            error_message = "Failed to dock";
          }
          else
          {
            if(odom_received_)
            {
              if(!startUndock() || task_cancelled_)
              {
                geometry_msgs::Twist stop;
                vel_pub_.publish(stop);
                return error_message;
              }
            }
            else
            {
              healthy_ = false;
              error_message = "No odometry data received";
              return error_message;
            }
            error_message = startDock();
          }
        }
        break;
      }
      r.sleep();
    }
    return error_message;
  }

  bool GeneralDockingHandler::startUndock()
  {
    ROS_INFO("[%s] Start undocking", name_.c_str());
    nav_msgs::Odometry initial_pose = odom_pose_;

    // Velocity command for undock
    geometry_msgs::Twist stop;
    ros::Rate r(loop_rate_);
    while(calcDistance(initial_pose, odom_pose_) < undocking_distance_)
    {
      // Cancel check
      if(task_cancelled_)
      {
        vel_pub_.publish(stop);
        ROS_INFO("[%s] Task cancelled, running required cleanup tasks", name_.c_str());
        return false;
      }
      
      // Pause check
      if(isTaskPaused())
      {
        vel_pub_.publish(stop);
        task_paused_ = true;
        while(isTaskPaused())
          continue;
        task_paused_ = false;
      }

      // Undock
      geometry_msgs::Twist vel;
      vel.linear.x = undocking_speed_;
      vel_pub_.publish(vel);
      r.sleep();
    }

    // Task end
    vel_pub_.publish(stop);
    return true;
  }
  
  std::string GeneralDockingHandler::externalProcess()
  {
    ROS_INFO("[%s] Calling external service", name_.c_str());
    std::string error_message;
    std_srvs::Trigger external_srv;
    external_process_running_ = true;
    if(external_process_client_.call(external_srv))
    {
      healthy_ = external_srv.response.success;
      if(!healthy_)
      {
        error_message = "External process error";
      }
    }
    else
    {
      error_message = "Failed to call external service";
      healthy_ = false;
    }
    external_process_running_ = false;
    if(!healthy_)
      healthCheck(error_message);

    return error_message;
  }
  
  void GeneralDockingHandler::healthCheck(std::string error_message)
  {
    movel_seirios_msgs::Reports unhealthy_msg;
    unhealthy_msg.header.stamp = ros::Time::now();
    unhealthy_msg.handler = name_;
    unhealthy_msg.healthy = false;
    unhealthy_msg.message = error_message;
    unhealthy_msg.task_type = task_type_;

    health_check_pub_.publish(unhealthy_msg);
  }

  void GeneralDockingHandler::internalCb(std_msgs::Bool success)
  {
    docking_success_internal_ = success.data;
    task_done_internal_ = true;
    ROS_INFO("[%s] Docking ended (internal)", name_.c_str());
  }

  void GeneralDockingHandler::externalCb(std_msgs::Bool success)
  {
    docking_success_external_ = success.data;
    task_done_external_ = true;
    ROS_INFO("[%s] Docking ended (external)", name_.c_str());
  }

  void GeneralDockingHandler::odomCb(nav_msgs::Odometry odom)
  {
    odom_pose_ = odom;
    odom_received_ = true;
  }

  void GeneralDockingHandler::cancelTask()
  {
    ROS_INFO("[%s] Received cancel command", name_.c_str());
    if(external_process_running_)
    {
      std_msgs::Empty cancel;
      external_cancel_pub_.publish(cancel);
    }
    task_cancelled_ = true;
    task_parsed_ = true;
    task_active_ = false;
    task_paused_ = false;
  }

  double GeneralDockingHandler::calcDistance(nav_msgs::Odometry pose1, nav_msgs::Odometry pose2)
  {
    double dx = pose1.pose.pose.position.x - pose2.pose.pose.position.x;
    double dy = pose1.pose.pose.position.y - pose2.pose.pose.position.y;
    return sqrt(dx * dx + dy * dy);
  }
}
