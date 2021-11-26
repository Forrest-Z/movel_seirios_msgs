#include <rtabmap_pcl_slam_handler/rtabmap_pcl_slam_handler.h>

PLUGINLIB_EXPORT_CLASS(rtabmap_pcl_slam_handler::RtabmapPclSlamHandler, task_supervisor::TaskHandler);

namespace rtabmap_pcl_slam_handler
{

bool RtabmapPclSlamHandler::onSaveServiceCall(movel_seirios_msgs::StringTrigger::Request& req,
                                            movel_seirios_msgs::StringTrigger::Response& res)
{
  res.success = saveMap(req.input);
  saved_ = true;
  return true;
}

bool RtabmapPclSlamHandler::onAsyncSave(movel_seirios_msgs::StringTrigger::Request& req,
                                 movel_seirios_msgs::StringTrigger::Response& res)
{
  res.success = saveMap(req.input);
  return true;
}

bool RtabmapPclSlamHandler::saveMap(std::string map_name)
{
  ROS_INFO("[%s] Saving map %s", name_.c_str(), map_name.size() != 0 ? ("to" + map_name).c_str() : "");

  // PointCloud to PCD
  /** Call conversion node service  **/
  movel_seirios_msgs::StringTrigger srv;
  srv.request.input = map_name;

  if (save_map_client_.call(srv))
    ROS_INFO("[%s] PCL Map Save complete", name_.c_str());
  else
  {
    ROS_ERROR("[%s] Failed to save PCL", name_.c_str());
    return false;
  }

  // Set path to save file
  std::string launch_args = " map_topic:=" + p_map_topic_;
  if (!map_name.empty())
  {
    launch_args = launch_args + " file_path:=" + map_name;
    // launch_args = launch_args + " pcd_path:=" + map_name + ".pcd";
    std::string map_name_nav (map_name);
    std::string key ("/");
    std::size_t idx = map_name_nav.rfind(key);
    if (idx != std::string::npos)
    {
      map_name_nav.replace(idx, key.length(), "/nav/");
      launch_args = launch_args + " file_path_nav:=" + map_name_nav;
    }
  }

  unsigned int map_saver_id = startLaunch(p_map_saver_package_, p_map_saver_launch_, launch_args);
  // Check if startLaunch succeeded
  if (!map_saver_id)
  {
    ROS_ERROR("[%s] Failed to start map saver", name_.c_str());
    return false;
  }

  // While loop until timeout
  ros::Rate r(p_loop_rate_);
  ros::Time start_time = ros::Time::now();
  while (ros::Time::now().toSec() - start_time.toSec() < p_save_timeout_)
  {
    // TODO map_saver might die before saving
    if (!launchExists(map_saver_id))
    {
      ROS_INFO("[%s] Save complete", name_.c_str());
      // stopLaunch(conversion_id);

      // 3D map checking
      FILE* pcd = fopen( (map_name + ".pcd").c_str(), "r");
      if (pcd == NULL)
      {
          ROS_ERROR("[%s] 3D map file is NOT FOUND in %s", name_.c_str(), (map_name + ".pcd").c_str());
          return false;
      }
      fclose(pcd);
      ROS_INFO("[%s] 3D map file is AVAILABLE in %s", name_.c_str(), (map_name + ".pcd").c_str());
      
      // 2D map checking
      FILE* pgm = fopen( (map_name + ".pgm").c_str(), "r");
      if (pgm == NULL)
      {
          ROS_ERROR("[%s] 2D map file is NOT FOUND in %s", name_.c_str(), (map_name + ".pgm").c_str());
          return false;
      }
      fclose(pgm);
      ROS_INFO("[%s] 2D map file is AVAILABLE in %s", name_.c_str(), (map_name + ".pgm").c_str());

      return true;
    }
    r.sleep();
  }
  ROS_WARN("[%s] Timeout occurred, save failed", name_.c_str());

  stopLaunch(map_saver_id);
  return false;
}

bool RtabmapPclSlamHandler::runMapping()
{
  ROS_INFO("[%s] Starting RTABMAP PCL Slam package: %s, launch file: %s", name_.c_str(), p_rtabmap_pcl_slam_launch_package_.c_str(),
           p_rtabmap_pcl_slam_launch_.c_str());

  // Run mapping asynchronously
  rtabmap_pcl_slam_launch_id_ = startLaunch(p_rtabmap_pcl_slam_launch_package_, p_rtabmap_pcl_slam_launch_, "");
  if (!rtabmap_pcl_slam_launch_id_)
  {
    ROS_ERROR("[%s] Failed to launch RTABMAP PCL Slam launch file", name_.c_str());
    return false;
  }

  // Loop until save callback is called
  ros::Rate r(p_loop_rate_);
  while (!saved_)
  {
    // Check if cancellation has been called
    if (!isTaskActive())
    {
      ROS_INFO("[%s] Waiting for RTABMAP PCL Slam thread to exit", name_.c_str());
      stopLaunch(rtabmap_pcl_slam_launch_id_);
      while (launchExists(rtabmap_pcl_slam_launch_id_))
        ;
      return false;
    }
    r.sleep();
  }

  ROS_INFO("[%s] Stopping RTABMAP PCL Slam", name_.c_str());
  stopLaunch(rtabmap_pcl_slam_launch_id_);
  while (launchExists(rtabmap_pcl_slam_launch_id_))
    ;
  rtabmap_pcl_slam_launch_id_ = 0;
  ROS_INFO("[%s] RTABMAP PCL Slam stopped", name_.c_str());

  saved_ = false;
  return true;
}

/**
 * runTask is called by task_supervisor once it receives a goal that includes a task of type 2. No payload is processed
 * by mapping_handler
 *
 * Once called, this method advertises the save_map service, as well as calls the runMapping method, which will carry
 * out all required tasks to start mapping.
 *
 * Once mapping is complete, save_map service is shut down and the task is signalled as complete
 */
task_supervisor::ReturnCode RtabmapPclSlamHandler::runTask(movel_seirios_msgs::Task& task, std::string& error_message)
{
  task_active_ = true;
  task_parsed_ = false;
  start_ = ros::Time::now();

  ros::ServiceServer serv_save_ = nh_handler_.advertiseService("save_pcl_map", &RtabmapPclSlamHandler::onSaveServiceCall, this);
  ros::ServiceServer serv_save_async_ = nh_handler_.advertiseService("save_pcl_map_async", &RtabmapPclSlamHandler::onAsyncSave, this);
  save_map_client_ = nh_handler_.serviceClient<movel_seirios_msgs::StringTrigger>("/pointcloud_saver/export_pcd");

  bool mapping_done = runMapping();

  // Reset all state variables and shutdown services
  setTaskResult(mapping_done);

  return code_;
}

/**
 * Loads all required and optional parameters for the use of mapping_handler. Parameters can be configured
 * in the task_supervisor's yaml file, under the 'mapping_handler' section
 *
 * Parameters are loaded on startup of task_supervisor, not on task execution
 */
bool RtabmapPclSlamHandler::loadParams()
{
  ros_utils::ParamLoader param_loader(nh_handler_);

  param_loader.get_optional("loop_rate", p_loop_rate_, 5.0);
  param_loader.get_optional("save_timeout", p_save_timeout_, 5.0);
  param_loader.get_optional("map_topic", p_map_topic_, std::string("/map"));
  param_loader.get_optional("resolution", p_resolution_, 0.05);
  param_loader.get_optional("utm", p_utm_, false);

  param_loader.get_required("rtabmap_pcl_slam_launch_package", p_rtabmap_pcl_slam_launch_package_);
  param_loader.get_required("rtabmap_pcl_slam_launch", p_rtabmap_pcl_slam_launch_);
  param_loader.get_required("rtabmap_pcl_map_saver_package", p_map_saver_package_);
  param_loader.get_required("rtabmap_pcl_map_saver_launch", p_map_saver_launch_);

  return param_loader.params_valid();
}

/**
 *	Setup method called once task_supervisor starts and loads all plugins (task handlers)
 */
bool RtabmapPclSlamHandler::setupHandler()
{
  if (!loadParams())
    return false;
  else
  {
    health_check_pub_ = nh_handler_.advertise<movel_seirios_msgs::Reports>("/task_supervisor/health_report", 1);
    return true;
  }
}

bool RtabmapPclSlamHandler::healthCheck()
{
  static int fail_count = 0;
  // ROS_INFO("rtabmap_pcl slam handler health check");
  if (rtabmap_pcl_slam_launch_id_ == 0)
  {
    fail_count = 0;
    return true;
  }
  if (task_active_)
  {
    if (!launchStatus(rtabmap_pcl_slam_launch_id_))
    {
      fail_count++;
      ROS_INFO("[%s] fail count %d", name_.c_str(), fail_count);
      if (fail_count >= 30*p_watchdog_rate_)
      {
        ROS_INFO("[%s] unhealthy", name_.c_str());

        // prep health report
        movel_seirios_msgs::Reports health_report;
        health_report.header.stamp = ros::Time::now();
        health_report.handler = "rtabmap_pcl_slam_handler";
        health_report.task_type = task_type_;
        health_report.message = "one or more 3D mapping node has failed";
        health_check_pub_.publish(health_report);
      
        // trigger task cancel
        // setTaskResult(false);
        cancelTask();
        fail_count = 0;
        // rtabmap_pcl_slam_launch_id_ = 0;
        return false;
      }
    }
    else
      fail_count = 0;
  }
  return true;
}
}  // namespace task_supervisor

