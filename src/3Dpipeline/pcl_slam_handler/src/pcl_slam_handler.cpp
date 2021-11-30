#include <pcl_slam_handler/pcl_slam_handler.h>

PLUGINLIB_EXPORT_CLASS(task_supervisor::PCLSlamHandler, task_supervisor::TaskHandler);

namespace task_supervisor
{
/**
 * Callback for /task_supervisor/map_saver/save_map service. Once save_map service is called, this method
 * starts the map_saver launch file and sets the map_topic depending on the yaml config file of task_supervisor.
 *
 * Where the map is saved is determined during the call to save_map service, a fully defined path can be passed
 * as an argument, otherwise the default location of $HOME is used.
 *
 * Default location can be modified in the launch file of map_saver, found in
 * (task_supervisor package)/launch/map_saver.launch
 */
bool PCLSlamHandler::onSaveServiceCall(movel_seirios_msgs::StringTrigger::Request& req,
                                       movel_seirios_msgs::StringTrigger::Response& res)
{
  res.success = saveMap(req.input);
  saved_ = true;
  return true;
}

bool PCLSlamHandler::onAsyncSave(movel_seirios_msgs::StringTrigger::Request& req,
                                 movel_seirios_msgs::StringTrigger::Response& res)
{
  res.success = saveMap(req.input);
  return true;
}

bool PCLSlamHandler::saveMap(std::string map_name)
{
  /** NORMAL PCL SLAM **/
  if(!p_use_rtabmap_)
  {
    // Save PCL to PCD
    // Call map saving through launch manager service
    ROS_INFO("[%s] Saving map %s", name_.c_str(), map_name.size() != 0 ? ("to" + map_name).c_str() : "");
    hdl_graph_slam::SaveMap srv;
    srv.request.utm = p_utm_;
    srv.request.resolution = p_resolution_;
    srv.request.destination = map_name+".pcd";

    if (save_map_client_.call(srv))
      ROS_INFO("[%s] PCL Map Save complete", name_.c_str());
    else
    {
      ROS_ERROR("[%s] Failed to save PCL", name_.c_str());
      return false;
    }
    std::string map_name_nav;
    // Set path to save file
    std::string launch_args = " map_topic:=" + p_map_topic_;
    if (!map_name.empty())
    {
      launch_args = launch_args + " file_path:=" + map_name;
      launch_args = launch_args + " pcd_path:=" + map_name + ".pcd";
      map_name_nav = map_name;
      std::string key ("/");
      std::size_t idx = map_name_nav.rfind(key);
      if (idx != std::string::npos)
      {
        map_name_nav.replace(idx, key.length(), "/nav/");
        launch_args = launch_args + " file_path_nav:=" + map_name_nav;
      }
    }

    // Convert PCD to 2D (3D to 2D) and save
    unsigned int conversion_id;
    if (!p_use_dynamic_2d_)
      conversion_id = startLaunch(p_3Dto2D_package_, p_3Dto2D_launch_, launch_args);

    unsigned int  map_saver_id = startLaunch(p_map_saver_package_, p_map_saver_launch_, launch_args);

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
        if (!p_use_dynamic_2d_)
          stopLaunch(conversion_id);

        ROS_INFO("[%s] Checking for 3D and 2D map files", name_.c_str());

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

  }
  else      /** RTABMAP PCL SLAM **/
  {
    ROS_INFO("[%s] Saving map %s", name_.c_str(), map_name.size() != 0 ? ("to" + map_name).c_str() : "");

    // PointCloud to PCD
    /** Call conversion node service  **/
    movel_seirios_msgs::StringTrigger srv;
    srv.request.input = map_name;

    if (save_map_client_rtabmap_.call(srv))
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

        ROS_INFO("[%s] Checking for 3D and 2D map files", name_.c_str());

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
  }
  return false;
}

/**
 * Method called by runTask() which starts up the mapping node. Mapping package and launch file is defined in the
 * task_supervisor
 * yaml under 'pcl_slam_launch_package' and 'pcl_slam_launch_file'. Launch file should only launch the mapping node,
 * such
 * as gmapping,
 * and no other nodes eg move_base.
 *
 * Once mapping is started by task_supervisor, mapping_handler eventually blocks in this method and waits for
 * cancellation
 * or saving to be done via save_map service. Once save_map service is called, this method will return
 * and the mapping node will be shut down.
 */
bool PCLSlamHandler::runMapping()
{
  if (!p_use_rtabmap_)
  {
    ROS_INFO("[%s] Starting PCL Slam package: %s, launch file: %s", name_.c_str(), p_pcl_slam_launch_package_.c_str(),
             p_pcl_slam_launch_.c_str());
  }
  else
  {
    ROS_INFO("[%s] Starting PCL Slam package: %s, launch file: %s", name_.c_str(), p_rtabmap_pcl_slam_launch_package_.c_str(),
             p_rtabmap_pcl_slam_launch_.c_str());
  }

  // Run mapping asynchronously
  if (!p_use_rtabmap_)
  {
    pcl_slam_launch_id_ = startLaunch(p_pcl_slam_launch_package_, p_pcl_slam_launch_, "");
  }
  else
  {
     pcl_slam_launch_id_ = startLaunch(p_rtabmap_pcl_slam_launch_package_, p_rtabmap_pcl_slam_launch_, "");
  }

  if (!pcl_slam_launch_id_)
  {
    ROS_ERROR("[%s] Failed to launch PCL Slam launch file", name_.c_str());
    return false;
  }

  // Loop until save callback is called
  ros::Rate r(p_loop_rate_);
  while (!saved_)
  {
    // Check if cancellation has been called
    if (!isTaskActive())
    {
      ROS_INFO("[%s] Waiting for PCL Slam thread to exit", name_.c_str());
      stopLaunch(pcl_slam_launch_id_);
      while (launchExists(pcl_slam_launch_id_))
        ;
      return false;
    }
    r.sleep();
  }

  ROS_INFO("[%s] Stopping PCL Slam", name_.c_str());
  stopLaunch(pcl_slam_launch_id_);
  while (launchExists(pcl_slam_launch_id_))
    ;
  pcl_slam_launch_id_ = 0;
  ROS_INFO("[%s] PCL Slam stopped", name_.c_str());

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
task_supervisor::ReturnCode PCLSlamHandler::runTask(movel_seirios_msgs::Task& task, std::string& error_message)
{
  task_active_ = true;
  task_parsed_ = false;
  start_ = ros::Time::now();

  ros::ServiceServer serv_save_ = 
    nh_handler_.advertiseService("save_pcl_map", &PCLSlamHandler::onSaveServiceCall, this);
  ros::ServiceServer serv_save_async_ =
    nh_handler_.advertiseService("save_pcl_map_async", &PCLSlamHandler::onAsyncSave, this);
  save_map_client_ = nh_handler_.serviceClient<hdl_graph_slam::SaveMap>("/hdl_graph_slam/save_map");
  save_map_client_rtabmap_ = nh_handler_.serviceClient<movel_seirios_msgs::StringTrigger>("/pointcloud_saver/export_pcd");

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
bool PCLSlamHandler::loadParams()
{
  ros_utils::ParamLoader param_loader(nh_handler_);

  param_loader.get_optional("loop_rate", p_loop_rate_, 5.0);
  param_loader.get_optional("save_timeout", p_save_timeout_, 5.0);
  param_loader.get_optional("map_topic", p_map_topic_, std::string("/map"));
  param_loader.get_optional("resolution", p_resolution_, 0.05);
  param_loader.get_optional("utm", p_utm_, false);
  param_loader.get_optional("use_dynamic_2d", p_use_dynamic_2d_, false);

  param_loader.get_required("pcl_slam_launch_package", p_pcl_slam_launch_package_);
  param_loader.get_required("pcl_slam_launch", p_pcl_slam_launch_);
  param_loader.get_required("pcl_map_saver_package", p_map_saver_package_);
  param_loader.get_required("pcl_map_saver_launch", p_map_saver_launch_);
  param_loader.get_required("three_to_two_package", p_3Dto2D_package_);
  param_loader.get_required("three_to_two_launch", p_3Dto2D_launch_);

  param_loader.get_required("rtabmap_pcl_slam_launch_package", p_rtabmap_pcl_slam_launch_package_);
  param_loader.get_required("rtabmap_pcl_slam_launch", p_rtabmap_pcl_slam_launch_);

  param_loader.get_required("use_rtabmap", p_use_rtabmap_);
  return param_loader.params_valid();
}

/**
 *	Setup method called once task_supervisor starts and loads all plugins (task handlers)
 */
bool PCLSlamHandler::setupHandler()
{
  if (!loadParams())
    return false;
  else
  {
    health_check_pub_ = nh_handler_.advertise<movel_seirios_msgs::Reports>("/task_supervisor/health_report", 1);
    return true;
  }
}

bool PCLSlamHandler::healthCheck()
{
  static int fail_count = 0;
  // ROS_INFO("pcl slam handler health check");
  if (pcl_slam_launch_id_ == 0)
  {
    fail_count = 0;
    return true;
  }
  if (task_active_)
  {
    if (!launchStatus(pcl_slam_launch_id_))
    {
      fail_count++;
      ROS_INFO("[%s] fail count %d", name_.c_str(), fail_count);
      if (fail_count >= 30*p_watchdog_rate_)
      {
        ROS_INFO("[%s] unhealthy", name_.c_str());

        // prep health report
        movel_seirios_msgs::Reports health_report;
        health_report.header.stamp = ros::Time::now();
        health_report.handler = "pcl_slam_handler";
        health_report.task_type = task_type_;
        health_report.message = "one or more 3D mapping node has failed";
        health_check_pub_.publish(health_report);
      
        // trigger task cancel
        // setTaskResult(false);
        cancelTask();
        fail_count = 0;
        // pcl_slam_launch_id_ = 0;
        return false;
      }
    }
    else
      fail_count = 0;
  }
  return true;
}
}  // namespace task_supervisor

