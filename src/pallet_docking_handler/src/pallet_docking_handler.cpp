#include <pallet_docking_handler/pallet_docking_handler.h>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(pallet_docking_handler::PalletDockingHandler, task_supervisor::TaskHandler);

namespace pallet_docking_handler
{
  PalletDockingHandler::PalletDockingHandler()
  {}

  bool PalletDockingHandler::setupHandler()
  {
    if (!loadParams())
      return false;

    retry_sub_ = nh_handler_.subscribe("retry", 1, &PalletDockingHandler::retryCb, this);
    pose_sub_ = nh_handler_.subscribe("/pose", 1, &PalletDockingHandler::poseCb, this);
    vel_pub_ = nh_handler_.advertise<geometry_msgs::Twist>("/cmd_vel_mux/autonomous", 1);
    cancel_pub_ = nh_handler_.advertise<actionlib_msgs::GoalID>("/move_base/cancel", 1);
    health_check_pub_ = nh_handler_.advertise<movel_seirios_msgs::Reports>("/task_supervisor/health_report", 1);
    success_sub_ = nh_handler_.subscribe("/goal/status", 1, &PalletDockingHandler::successCb, this);
    success_sub2_ = nh_handler_.subscribe("/pallet_docking/success", 1, &PalletDockingHandler::successCb2, this);
    pallet_sub_ = nh_handler_.subscribe("/pallet_detection/pallets", 1, &PalletDockingHandler::palletCb, this);
    plan_inspector_client_ = nh_handler_.serviceClient<std_srvs::SetBool>("/enable_plan_inspector");
    teb_client_ = nh_handler_.serviceClient<dynamic_reconfigure::Reconfigure>("/move_base/TebLocalPlannerROS/set_parameters");
    return true;
  }

  bool PalletDockingHandler::loadParams()
  {
    ros_utils::ParamLoader param_loader(nh_handler_);

    param_loader.get_required("dock", dock_);
    param_loader.get_required("loop_rate", loop_rate_);
    param_loader.get_required("pallet_docking_launch_package", launch_pkg_);
    param_loader.get_required("pallet_docking_launch_file", launch_file_);
    
    param_loader.get_optional("use_move_base", use_move_base_, false);
    param_loader.get_optional("spot_turn_vel", spot_turn_vel_, 0.1);
    param_loader.get_optional("detection_timeout", detection_timeout_, 10.0);
    param_loader.get_optional("xy_tolerance", xy_tolerance_, 0.04);
    param_loader.get_optional("yaw_tolerance", yaw_tolerance_, 0.0349);
    
    return param_loader.params_valid();
  }

  std::string PalletDockingHandler::startTask()
  {
    std::string error_message;

    // Undock
    if(!dock_)
    {
      error_message = "[" + name_ + "] " + startUndock(false);
      if(!healthy_)
        return error_message;
    }

    // Dock
    if(dock_)
    {
      error_message = "[" + name_ + "] " + startDock();
      if(!healthy_)
        return error_message;
    }
    return error_message;
  }

  task_supervisor::ReturnCode PalletDockingHandler::runTask(movel_seirios_msgs::Task& task,
                                                           std::string& error_message)
  {
    // Reset global variables
    task_active_ = true;
    task_paused_ = false;
    task_parsed_ = false;
    healthy_ = true;
    task_cancelled_ = false;
    pallet_detected_ = false;
    pose_received_ = false;
    status_received_ = false;

    error_message = startTask();
    setTaskResult(healthy_);
    return code_;
  }

  std::string PalletDockingHandler::startDock()
  {
    bool reset_plan_inspector = false;
    std::string error_message;

    // Launch nodes
    if(!use_move_base_)
      docking_launch_id_ = startLaunch(launch_pkg_, launch_file_, "");
    else
    {
      // Disable plan_inspector
      std_srvs::SetBool disable;
      disable.request.data = false;
      if(plan_inspector_client_.call(disable))
      {
        if (disable.response.message.find("already") == std::string::npos)
          reset_plan_inspector = true;
      }
      else
        ROS_WARN("[%s] Failed to call /enable_plan_inspector service", name_.c_str());

      setConfigs(true);
      docking_launch_id_ = startLaunch(launch_pkg_, launch_file_, "");
    }
    if(!docking_launch_id_)
    {
      error_message = "Failed to launch docking nodes";
      healthy_ = false;
      return error_message;
    }

    planner_fail_ = false;
    docking_success_ = false;
    paused_ = false;
    task_done_ = false;
    task2_done_ = false;
    retry_ = false;
    ros::Time start_time = ros::Time::now();
    actionlib_msgs::GoalID cancel;

    // wait for completion loop
    ros::Rate r(loop_rate_);
    while(ros::ok())
    {
      // Check of pallet is detected
      if(ros::Time::now().toSec() - start_time.toSec() > detection_timeout_ && !pallet_detected_)
      {
        if(!use_move_base_)
          spotTurn();
        if(!pallet_detected_)
        {
          if (docking_launch_id_)
          {
            stopLaunch(docking_launch_id_);
	          docking_launch_id_ = 0;
            if(use_move_base_)
            {
              cancel_pub_.publish(cancel);
              setConfigs(false);
            }
            if(reset_plan_inspector)
            {
              std_srvs::SetBool enable;
              enable.request.data = true;
              plan_inspector_client_.call(enable);
            }
            error_message = "Failed to detect pallet";
            healthy_ = false;
            return error_message;
          }
        }
      }
      // active check
      if (task_cancelled_)
      {
        if (docking_launch_id_)
        {
          stopLaunch(docking_launch_id_);
	        docking_launch_id_ = 0;
          if(use_move_base_)
          {
            cancel_pub_.publish(cancel);
            setConfigs(false);
          }
          if(reset_plan_inspector)
          {
            std_srvs::SetBool enable;
            enable.request.data = true;
            plan_inspector_client_.call(enable);
          }
          geometry_msgs::Twist stop;
          vel_pub_.publish(stop);
        }
        break;
      }

      // pause check
      /*if (isTaskPaused() && !paused_) // pause rising edge
      {
        if (docking_launch_id_)
        {
          stopLaunch(docking_launch_id_);
	        docking_launch_id_ = 0;
          if(use_move_base_)
          {
            cancel_pub_.publish(cancel);
          }
          geometry_msgs::Twist stop;
          vel_pub_.publish(stop);

        }
        paused_ = true;
      }
      // resume check
      else if (!isTaskPaused() && paused_) // pause falling edge
      {
        if(!use_move_base_)
          docking_launch_id_ = startLaunch(launch_pkg_, launch_file_, "");
        else
          docking_launch_id_ = startLaunch(launch_pkg_, launch_file_, "");
        if (!docking_launch_id_)
        {
          error_message = "Failed to launch docking nodes";
          healthy_ = false;
          return error_message;
        }
        paused_ = false;
      }*/

      // success check
      //if (task_done_)
      //{
        /*if (!docking_success_)
        {
          healthy_ = false;
          error_message = "Failed to dock";
        }*/
        //break;
      //}

      // Check if task is complete
      if(task2_done_)
      {
        if (docking_launch_id_)
        {
          stopLaunch(docking_launch_id_);
	        docking_launch_id_ = 0;
          if(use_move_base_)
          {
            cancel_pub_.publish(cancel);
            setConfigs(false);
          }
          if(reset_plan_inspector)
          {
            std_srvs::SetBool enable;
            enable.request.data = true;
            plan_inspector_client_.call(enable);
          }
        }
        if(!docking_success_)
          retry_ = true;
        break;
      }

      // Stop task if path planning error from planner_adjuster/move_base detected
      if(planner_fail_)
      {
        if (docking_launch_id_)
        {
          stopLaunch(docking_launch_id_);
	        docking_launch_id_ = 0;
          if(use_move_base_)
          {
            cancel_pub_.publish(cancel);
            setConfigs(false);
          }
          if(reset_plan_inspector)
          {
            std_srvs::SetBool enable;
            enable.request.data = true;
            plan_inspector_client_.call(enable);
          }
        }
        error_message = "planner error";
        healthy_ = false;
        break;
      }
      r.sleep();
    }

    /*while(ros::ok())
    {
      // active check
      if (task_cancelled_)
      {
        if (docking_launch_id_)
        {
          stopLaunch(docking_launch_id_);
	        docking_launch_id_ = 0;
          if(use_move_base_)
          {
            cancel_pub_.publish(cancel);
            setConfigs(false);
          }
          if(reset_plan_inspector)
          {
            std_srvs::SetBool enable;
            enable.request.data = true;
            plan_inspector_client_.call(enable);
          }
          geometry_msgs::Twist stop;
          vel_pub_.publish(stop);
        }
        break;
      }

      if(task2_done_)
      {
        if (docking_launch_id_)
        {
          stopLaunch(docking_launch_id_);
	        docking_launch_id_ = 0;
          if(use_move_base_)
          {
            cancel_pub_.publish(cancel);
            setConfigs(false);
          }
          if(reset_plan_inspector)
          {
            std_srvs::SetBool enable;
            enable.request.data = true;
            plan_inspector_client_.call(enable);
          }
        }
        if(!docking_success_)
          retry_ = true;
        break;
      }
      r.sleep();
    }*/

    // Retry docking (undock and dock again)
    if(retry_)
    {
      startUndock(true);
      error_message = startDock();
    } 
    return error_message;
  }

  // Rotate on the spot to look for pallets
  void PalletDockingHandler::spotTurn()
  {
    geometry_msgs::Twist stop;
    ros::Time start_time = ros::Time::now();
    geometry_msgs::Pose start_pose = pose_;
    while(!pallet_detected_ && calcAngleDiff(pose_, start_pose) < 0.7854)
    {
      geometry_msgs::Twist rotate;
      rotate.angular.z = -spot_turn_vel_;
      vel_pub_.publish(rotate);
      ros::Duration(0.1).sleep();
      if(task_cancelled_)
      {
        vel_pub_.publish(stop);
        return;
      }
    }
    vel_pub_.publish(stop);
    ros::Duration(1.0).sleep();
    
    while(!pallet_detected_ && !(calcAngleDiff(pose_, start_pose) < 0.1))
    {
      geometry_msgs::Twist rotate;
      rotate.angular.z = spot_turn_vel_;
      vel_pub_.publish(rotate);
      ros::Duration(0.1).sleep();
      if(task_cancelled_)
      {
        vel_pub_.publish(stop);
        return;
      }
    }
    while(!pallet_detected_ && calcAngleDiff(pose_, start_pose) < 0.7854)
    {
      geometry_msgs::Twist rotate;
      rotate.angular.z = spot_turn_vel_;
      vel_pub_.publish(rotate);
      ros::Duration(0.1).sleep();
      if(task_cancelled_)
      {
        vel_pub_.publish(stop);
        return;
      }
    }
    vel_pub_.publish(stop);
  }

  std::string PalletDockingHandler::startUndock(bool retry)
  {
    task_cancelled_ = false;
    paused_ = false;
    task2_done_ = false;

    std::string error_message;
    if(retry)
      docking_launch_id_ = startLaunch(launch_pkg_, "pallet_undocking.launch", " retry:=true");
    else
      docking_launch_id_ = startLaunch(launch_pkg_, "pallet_undocking.launch", "");
    if (!docking_launch_id_)
    {
      error_message = "Failed to launch docking nodes";
      healthy_ = false;
      return error_message;
    }

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
        break;
      }

      // pause check
      /*if (isTaskPaused() && !paused_) // pause rising edge
      {
        if (docking_launch_id_)
        {
          stopLaunch(docking_launch_id_);
	        docking_launch_id_ = 0;
        }
        paused_ = true;
      }
      // resume check
      else if (!isTaskPaused() && paused_) // pause falling edge
      {
        if(retry)
          docking_launch_id_ = startLaunch(launch_pkg_, "pallet_undocking.launch", " retry:=true");
        else
          docking_launch_id_ = startLaunch(launch_pkg_, "pallet_undocking.launch", "");
        if (!docking_launch_id_)
        {
          error_message = "Failed to launch docking nodes";
          healthy_ = false;
          return error_message;
        }
        paused_ = false;
      }*/

      // success check
      if (task2_done_)
      {
        /*if (!docking_success_)
        {
          healthy_ = false;
          error_message = "Failed to dock";
        }*/
        if (docking_launch_id_)
        {
          stopLaunch(docking_launch_id_);
	        docking_launch_id_ = 0;
        }
        break;
      }
      r.sleep();
    }
    return error_message;
  }
  
  void PalletDockingHandler::healthCheck(std::string error_message)
  {
    movel_seirios_msgs::Reports unhealthy_msg;
    unhealthy_msg.header.stamp = ros::Time::now();
    unhealthy_msg.handler = name_;
    unhealthy_msg.healthy = false;
    unhealthy_msg.message = error_message;
    unhealthy_msg.task_type = task_type_;

    health_check_pub_.publish(unhealthy_msg);
  }

  // Reached goal with planner_adjuster/move_base
  void PalletDockingHandler::successCb(std_msgs::Bool success)
  {
    if(!success.data)
    {
      planner_fail_ = true;
    }
    if(status_received_)
    {
      task_done_ = true;
      return;
    }
    status_received_ = true;
  }

  // Reached goal with pallet_docking
  void PalletDockingHandler::successCb2(std_msgs::Bool success)
  {
    docking_success_ = success.data;
    task2_done_ = true;
  }

  // Trigger docking retry
  void PalletDockingHandler::retryCb(std_msgs::Empty msg)
  {
    docking_success_ = false;
    task2_done_ = true;
  }

  // Check if pallet is detected
  void PalletDockingHandler::palletCb(visualization_msgs::Marker pallets)
  {
    pallet_detected_ = true;
  }

  // Get robot pose
  void PalletDockingHandler::poseCb(geometry_msgs::Pose pose)
  {
    pose_ = pose;
    pose_received_ = true;
  }

  void PalletDockingHandler::cancelTask()
  {
    task_cancelled_ = true;
    task_parsed_ = true;
    task_active_ = false;
    task_paused_ = false;
  }

  double PalletDockingHandler::calcDistance(geometry_msgs::Pose pose1, geometry_msgs::Pose pose2)
  {
    double dx = pose1.position.x - pose2.position.x;
    double dy = pose1.position.y - pose2.position.y;
    return sqrt(dx * dx + dy * dy);
  }

  double PalletDockingHandler::calcAngleDiff(geometry_msgs::Pose init_pose, geometry_msgs::Pose target_pose)
  {
    tf::Quaternion q1(init_pose.orientation.x, init_pose.orientation.y,
                      init_pose.orientation.z, init_pose.orientation.w),
                   q2(target_pose.orientation.x, target_pose.orientation.y,
                      target_pose.orientation.z, target_pose.orientation.w);
    tf::Matrix3x3 m1(q1), m2(q2);
    double r1, p1, y1, r2, p2, y2;
    m1.getRPY(r1, p1, y1);
    m2.getRPY(r2, p2, y2);
  
    double dtheta = fabs(y1 - y2);
    dtheta = std::min(dtheta, 2.0*M_PI - dtheta);
    return dtheta;
  }

  // Change TEB parameters with dynamic reconfigure
  void PalletDockingHandler::setConfigs(bool set)
  {
    dynamic_reconfigure::Reconfigure reconfigure;
    dynamic_reconfigure::DoubleParameter double_param;
    std::vector<std::string> parameters = {"xy_goal_tolerance", "yaw_goal_tolerance",
                                           "max_vel_x", "max_vel_x_backwards", "max_vel_theta"};
    std::vector<double> custom_values = {xy_tolerance_, yaw_tolerance_, 0.11, 0.11, 0.11};
    if(set)
    {
      std::string prefix = "/move_base/TebLocalPlannerROS/";
      double value;
      for(size_t i = 0; i < custom_values.size(); i++)
      {
        nh_handler_.getParam(prefix + parameters[i], value);
        default_values_.push_back(value);
        double_param.name = parameters[i];
        double_param.value = custom_values[i];
        reconfigure.request.config.doubles.push_back(double_param);
      }
    }
    else
    {
      for(size_t i = 0; i < default_values_.size(); i++)
      {
        double_param.name = parameters[i];
        double_param.value = default_values_[i];
        reconfigure.request.config.doubles.push_back(double_param);
      }
      default_values_.clear();
    }

    if(!teb_client_.call(reconfigure))
      ROS_WARN("[%s] Failed to reconfigure TEB configs", name_.c_str());
  }
}
