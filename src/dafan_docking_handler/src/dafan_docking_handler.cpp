#include <dafan_docking_handler/dafan_docking_handler.h>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(dafan_docking_handler::DafanDockingHandler, task_supervisor::TaskHandler);

namespace dafan_docking_handler
{
  DafanDockingHandler::DafanDockingHandler() {}

  bool DafanDockingHandler::setupHandler()
  {
    if (!loadParams())
      return false;

    docking_status_sub_ = nh_handler_.subscribe("/docking_status", 1, &DafanDockingHandler::dockingStatusCb, this);
    start_docking_clt_ = nh_handler_.serviceClient<dafan_docking::ToggleDocking>("/toggle_docking");
    stop_control_pub_ = nh_handler_.advertise<std_msgs::Bool>("/stop_now", 1);
    
    return true;
  }

  bool DafanDockingHandler::loadParams()
  {
    ros_utils::ParamLoader param_loader(nh_handler_);

    param_loader.get_required("dafan_docking_launch_package", launch_pkg_);
    param_loader.get_required("dafan_docking_launch_file", launch_file_);
    param_loader.get_required("loop_rate", loop_rate_);
    
    return param_loader.params_valid();
  }

  void DafanDockingHandler::startDocking()
  {
    docking_success_ = false;
    dafan_docking::ToggleDocking toggle_docking_srv;
    toggle_docking_srv.request.toggle = true;
    start_docking_clt_.call(toggle_docking_srv);
  }

  task_supervisor::ReturnCode DafanDockingHandler::runTask(movel_seirios_msgs::Task& task,
                                                           std::string& error_message)
  {
    task_active_ = true;
    task_paused_ = false;

    // launch nodes
    docking_launch_id_ = startLaunch(launch_pkg_, launch_file_, "");
    if (!docking_launch_id_)
    {
      ROS_INFO("[%s] Failed to launch docking nodes", name_.c_str());
      setTaskResult(false);
      return code_;
    }

    // call docking service
    if (!start_docking_clt_.waitForExistence(ros::Duration(10.0)))
    {
      ROS_INFO("[%s] Docking start service failed to manifest", name_.c_str());
      setTaskResult(false);
      return code_;
    }
    startDocking();    

    // wait for completion loop
    ros::Rate r(loop_rate_);
    bool prev_pause = false;
    while(ros::ok())
    {
      // active check
      if (!isTaskActive())
      {
        stopDocking();
        tearDown();
        break;
      }

      // pause check
      if (isTaskPaused() && !prev_pause) // pause rising edge
      {
        stopDocking();
        prev_pause = isTaskPaused();
      }
      // resume check
      else if (!isTaskPaused() && prev_pause) // pause falling edge
      {
        startDocking();
        prev_pause = isTaskPaused();
      }

      // success check
      if (docking_success_)
      {
        setTaskResult(true);
        tearDown();
        return code_;
      }

      r.sleep();
    }

    setTaskResult(false);
    return code_;
  }

  bool DafanDockingHandler::healthCheck()
  {
    static int fail_count = 0;
    // TODO: add health check after merge with the reporting branch

    return true;
  }

  void DafanDockingHandler::stopDocking()
  {
    std_msgs::Bool stop_msg;
    stop_msg.data = true;
    stop_control_pub_.publish(stop_msg);

    dafan_docking::ToggleDocking toggle_docking_srv;
    toggle_docking_srv.request.toggle = false;
    start_docking_clt_.call(toggle_docking_srv);
  }

  void DafanDockingHandler::tearDown()
  {
    if (docking_launch_id_)
    {
      stopLaunch(docking_launch_id_);
      while(docking_launch_id_)
        ;
    }
  }

  void DafanDockingHandler::dockingStatusCb(std_msgs::Int8 msg)
  {
    if (isTaskActive())
    {
      if (msg.data == -1)
        task_active_ = false;
      else if (msg.data == 1)
        docking_success_ = true;
    }
  }
}