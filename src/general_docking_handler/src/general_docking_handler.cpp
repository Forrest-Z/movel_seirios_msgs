#include <general_docking_handler/general_docking_handler.h>
#include <pluginlib/class_list_macros.h>
#include <task_supervisor/json.hpp>

using json = nlohmann::json;

PLUGINLIB_EXPORT_CLASS(general_docking_handler::GeneralDockingHandler, task_supervisor::TaskHandler);

namespace general_docking_handler
{
  GeneralDockingHandler::GeneralDockingHandler() : odom_received_(false), external_process_running_(false) //charging_(false), battery_status_received_(false), odom_received_(false)
  {}

  bool GeneralDockingHandler::setupHandler()
  {
    if (!loadParams())
      return false;

    //success_sub_ = nh_handler_.subscribe("/movel_dalu_docking/success", 1, &GeneralDockingHandler::successCb, this);
    internal_feedback_sub_ = nh_handler_.subscribe(internal_topic_, 1, &GeneralDockingHandler::internalCb, this);
    //battery_status_sub_ = nh_handler_.subscribe("/xnergy_charger_rcu/battery_state", 1, &GeneralDockingHandler::batteryStatusCb, this);
    if(use_external_feedback_)
      external_feedback_sub_ = nh_handler_.subscribe(external_topic_, 1, &GeneralDockingHandler::externalCb, this);

    //start_charging_client_ = nh_handler_.serviceClient<std_srvs::Trigger>("/xnergy_charger_rcu/start_charging");
    //stop_charging_client_ = nh_handler_.serviceClient<std_srvs::Trigger>("/xnergy_charger_rcu/stop_charging");
    if(use_external_service_)
    {
      external_process_client_ = nh_handler_.serviceClient<std_srvs::Trigger>(external_service_);
      external_cancel_pub_ = nh_handler_.advertise<std_msgs::Empty>(external_cancel_topic_, 1);
    }

    odom_sub_ = nh_handler_.subscribe(odom_topic_, 1, &GeneralDockingHandler::odomCb, this);
    vel_pub_ = nh_handler_.advertise<geometry_msgs::Twist>("/cmd_vel_mux/autonomous", 1);
    health_check_pub_ = nh_handler_.advertise<movel_seirios_msgs::Reports>("/task_supervisor/health_report", 1);
    
    //run_client_ = nh_handler_.serviceClient<std_srvs::SetBool>("/movel_dalu_docking/run");
    //velocity_smoother_sub_ = nh_handler_.subscribe(, 1, &GeneralDockingHandler::smootherCb, this);
    enable_smoother_client_ = nh_handler_.serviceClient<std_srvs::SetBool>("/velocity_smoother/velocity_smoother_on");
    //smoother_status_client_

    return true;
  }

  bool GeneralDockingHandler::loadParams()
  {
    ros_utils::ParamLoader param_loader(nh_handler_);

    param_loader.get_required("dock", dock_);
    //param_loader.get_required("battery_status_timeout", battery_status_timeout_);
    param_loader.get_required("loop_rate", loop_rate_);
    param_loader.get_required("odom_topic", odom_topic_);
    param_loader.get_required("use_external_service", use_external_service_);

    if(use_external_service_)
    {
      param_loader.get_required("external_service", external_service_);
      param_loader.get_required("external_cancel_topic", external_cancel_topic_);
    }

    if(dock_)
    {
      param_loader.get_required("use_external_feedback", use_external_feedback_);
      param_loader.get_required("external_topic", external_topic_);
      param_loader.get_required("internal_topic", internal_topic_);
      param_loader.get_required("docking_launch_package", launch_pkg_);
      param_loader.get_required("docking_launch_file", launch_file_);
      param_loader.get_required("camera_name", camera_name_);
      param_loader.get_required("enable_retry", enable_retry_);
      param_loader.get_required("retry_undocking_distance", retry_undocking_distance_);
    }
    else
    {
      param_loader.get_required("undocking_distance", undocking_distance_);
      param_loader.get_required("undocking_speed", undocking_speed_);
    }

    //param_loader.get_optional("use_apriltag", use_apriltag_, true);
    
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
        return error_message;
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
      if(!startUndock(false) || task_cancelled_)
        return error_message;
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
        return error_message;

      if (use_external_service_)
      {
        error_message = "[" + name_ + "] " + externalProcess();
        if(!healthy_ || task_cancelled_)
          return error_message;
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
    smoother_on_ = false;
    //charging_current_ = 0;

    //if(!use_apriltag_)
    if(!task.payload.empty())
    {
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
        error_message = "[" + name_ + "] Payload command format invalid, input 'dock', 'undock' or the following format: {\"position\":{\"x\":-4,\"y\":0.58,\"z\":0}, "
                        "\"orientation\":{\"x\":0,\"y\":0,\"z\":0.71,\"w\":0.69}}";
        setTaskResult(false);
        return code_;
      }
    }

    std_srvs::SetBool enable_smoother;
    //if(smoother_on_)
    {
      enable_smoother.request.data = false;
      if(!enable_smoother_client_.call(enable_smoother))
      {
        ROS_WARN("[%s] Failed to call /velocity_smoother/velocity_smoother_on", name_.c_str());
      }
    }

    error_message = startTask();

    //if(smoother_on)
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
    if(!goal_received_)
      arg = " camera:=" + camera_name_;
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

    // call docking service
    /*if (!run_client_.waitForExistence(ros::Duration(10.0)))
    {
      error_message = "Run docking service failed to manifest";
      healthy_ = false;
      return error_message;
    }
    docking_success_ = false;
    paused_ = false;
    task_done_ = false;
    std_srvs::SetBool dock_srv;
    if (!goal_received_)
    {
      dock_srv.request.data = true;
      run_client_.call(dock_srv);
    }
    else
    {
      ros::Duration(1.0).sleep();
      goal_pub_.publish(goal_pose_);
    }*/

    docking_success_ = false;
    task_done_ = false;
    docking_success_internal_ = false;
    docking_success_external_ = false;
    task_done_internal_ = false;
    task_done_external_ = false;

    // wait for completion loop
    ros::Rate r(loop_rate_);
    while(ros::ok())
    {
      // active check
      if (task_cancelled_)
      {
        /*std_srvs::SetBool dock_srv;
        dock_srv.request.data = false;
        run_client_.call(dock_srv);*/
        
        if (docking_launch_id_)
        {
          stopLaunch(docking_launch_id_);
	        docking_launch_id_ = 0;
        }
        break;
      }

      // pause check
      /*if (isTaskPaused() && !paused_) // pause rising edge
      {
        dock_srv.request.data = false;
        run_client_.call(dock_srv);
        paused_ = true;
      }
      // resume check
      else if (!isTaskPaused() && paused_) // pause falling edge
      {
        if (!goal_received_)
        {
          dock_srv.request.data = true;
          run_client_.call(dock_srv);
          paused_ = false;
        }
        else
        {
          goal_pub_.publish(goal_pose_);
	        paused_ = false;
	      }
      }*/

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
          if(task_done_internal_)
          {
            task_done_ = task_done_internal_;
            docking_success_ = false;
          }
        }
      }

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
              if(!startUndock(false) || task_cancelled_)
                return error_message;
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

  bool GeneralDockingHandler::startUndock(bool retry)
  {
    ROS_INFO("[%s] Start undocking", name_.c_str());
    nav_msgs::Odometry initial_pose = odom_pose_;

    // Velocity command for undock
    geometry_msgs::Twist stop;
    ros::Rate r(loop_rate_);
    double undock_distance = (retry) ? retry_undocking_distance_ : undocking_distance_;
    while(calcDistance(initial_pose, odom_pose_) < undock_distance)
    {
      if(task_cancelled_)
      {
        vel_pub_.publish(stop);
        ROS_INFO("[%s] Task cancelled, running required cleanup tasks", name_.c_str());
        return false;
      }
      
      if(isTaskPaused())
      {
        vel_pub_.publish(stop);
        task_paused_ = true;
        while(isTaskPaused())
          continue;
        task_paused_ = false;
      }

      geometry_msgs::Twist vel;
      vel.linear.x = undocking_speed_;
      vel_pub_.publish(vel);
      r.sleep();
    }
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
      /*else
      {
        if(battery_status_received_)
        {
          ros::Time start_time = ros::Time::now();
          while(charging_)
          {
            if (ros::Time::now().toSec() - start_time.toSec() > battery_status_timeout_)
            {
              error_message = "Timed out waiting for battery to stop charging";
              healthy_ = false;
              break;
            }
            continue;
          }
        }
        else
        {
          healthy_ = false;
          error_message = "No battery status received";
        }
      }*/
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
  
  /*std::string GeneralDockingHandler::startCharging()
  {
    std::string error_message;
    std_srvs::Trigger start_charging_srv;
    if(start_charging_client_.call(start_charging_srv))
    {
      healthy_ = start_charging_srv.response.success;
      if(!healthy_)
      {
        error_message = "Start charging service response returned false";
      }
      else
      {
        if(battery_status_received_)
        {
          ros::Time start_time = ros::Time::now();
          while(!charging_)
          {
            if (ros::Time::now().toSec() - start_time.toSec() > battery_status_timeout_)
            {
              error_message = "Timed out waiting for battery to start charging";
              healthy_ = false;
              break;
            }
            continue;
          }
          start_time = ros::Time::now();
          while(charging_current_ < (float)19.0)
          {
            if (ros::Time::now().toSec() - start_time.toSec() > 60)
            {
              error_message = "Timed out waiting for charging current to reach 20A";
              healthy_ = false;
              break;
            }
            continue;
          }
        }
        else
        {
          healthy_ = false;
          error_message = "No battery status received";
        }
      }
    }
    else
    {
      error_message = "Failed to call start charging service";
      healthy_ = false;
    }

    if(!healthy_)
      healthCheck(error_message);

    return error_message;
  }*/
  
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
