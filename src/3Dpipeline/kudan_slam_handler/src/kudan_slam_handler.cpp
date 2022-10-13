#include <kudan_slam_handler/kudan_slam_handler.h>



PLUGINLIB_EXPORT_CLASS(task_supervisor::KudanSlamHandler, task_supervisor::TaskHandler);

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
bool KudanSlamHandler::onSaveServiceCall(movel_seirios_msgs::StringTrigger::Request& req,
                                       movel_seirios_msgs::StringTrigger::Response& res)
{
  res.success = saveMap(req.input);
  saved_ = true;
  return true;
}

bool KudanSlamHandler::onAsyncSave(movel_seirios_msgs::StringTrigger::Request& req,
                                 movel_seirios_msgs::StringTrigger::Response& res)
{
  res.success = saveMap(req.input);
  return true;
}

bool KudanSlamHandler::saveMap(std::string map_name)
{
  map_name_save_ = map_name;
  ROS_INFO("[%s] Saving map %s", name_.c_str(), map_name.size() != 0 ? ("to" + map_name).c_str() : "");
  /** NORMAL Kudan SLAM **/
  // Save Kudan to kdlm
  // Call map saving through launch manager service
  kdlidar_ros_msgs::SaveFile srv;
  srv.request.name = map_name;

  if (save_map_client_.call(srv))
    ROS_INFO("[%s] Kudan Map Save complete", name_.c_str());
  else
  {
    ROS_ERROR("[%s] Failed to save Kudan", name_.c_str());
    return false;
  }
  // Set path to save file
  std::string launch_args = " map_topic:=" + p_map_topic_;
  if (!map_name.empty())
  {
    launch_args = launch_args + " file_path:=" + map_name;
    launch_args = launch_args + " pcd_path:=" + map_name + ".kdlm";
    map_name_nav_ = map_name;
    std::string key ("/");
    std::size_t idx = map_name_nav_.rfind(key);
    if (idx != std::string::npos)
    {
      map_name_nav_.replace(idx, key.length(), "/nav/");
      launch_args = launch_args + " file_path_nav:=" + map_name_nav_;
    }
  }

  // Convert kdlm to 2D (3D to 2D) and save
  unsigned int conversion_id;
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
      stopLaunch(conversion_id);

      ROS_INFO("[%s] Checking for 3D and 2D map files", name_.c_str());

      // 3D map checking
      FILE* kdlm = fopen( (map_name + ".kdlm").c_str(), "r");
      if (kdlm == NULL)
      {
          ROS_ERROR("[%s] 3D map file is NOT FOUND in %s", name_.c_str(), (map_name + ".kdlm").c_str());
          return false;
      }
      fclose(kdlm);
      ROS_INFO("[%s] 3D map file is AVAILABLE in %s", name_.c_str(), (map_name + ".kdlm").c_str());
      
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
bool KudanSlamHandler::runMapping()
{
  ROS_INFO("[%s] Starting Kudan Slam package: %s, launch file: %s", name_.c_str(), p_kudan_slam_launch_package_.c_str(),
             p_kudan_slam_launch_.c_str());

  // Run mapping asynchronously
  kudan_slam_launch_id_ = startLaunch(p_kudan_slam_launch_package_, p_kudan_slam_launch_, "");

  if (!kudan_slam_launch_id_)
  {
    ROS_ERROR("[%s] Failed to launch Kudan Slam launch file", name_.c_str());
    return false;
  }
  mapping_ = true;
  map_health_timer_.start();
  // Loop until save callback is called
  ros::Rate r(p_loop_rate_);
  while (!saved_)
  {
    // Check if cancellation has been called
    if (!isTaskActive())
    {
      ROS_INFO("[%s] Waiting for Kudan Slam thread to exit", name_.c_str());
      stopLaunch(kudan_slam_launch_id_);
      while (launchExists(kudan_slam_launch_id_))
        ;
      return false;
    }
    r.sleep();
  }
  map_health_timer_.stop();


  // Append a type to the yaml file
  ROS_INFO("[%s] Configuring yaml file!!!", name_.c_str());
  std::ofstream yaml_file(map_name_save_ + ".yaml", std::ios::out | std::ios::app);
  if(yaml_file.is_open())
  {
    yaml_file<<"type: 3d_rtabmap\n";
    yaml_file.close();
  }
  else
  {
    ROS_ERROR("[%s] Error opening a yaml file!", name_.c_str());
  }

// Create DB file for UI to load 3D handler
  try {
    ROS_INFO("[%s] Creating DB file!", name_.c_str());
    std::ofstream (map_name_save_ + ".db");
  }
  catch (...) {
    ROS_ERROR("[%s] Error creating DB file!", name_.c_str());
  }

// Edit yaml file to change origin: 'nan' to 0
  try {
    ROS_INFO("[%s] Editing YAML file!", name_.c_str());
    YAML::Node map_yaml = YAML::LoadFile(map_name_save_ + ".yaml");
    YAML::Node map_nav_yaml = YAML::LoadFile(map_name_nav_ + ".yaml");
    map_yaml["origin"][2] = 0;
    map_nav_yaml["origin"][2] = 0;
    std::ofstream fout(map_name_save_ + ".yaml"); 
    std::ofstream dout(map_name_nav_ + ".yaml"); 
    fout << map_yaml; // dump it back into the file
    dout << map_nav_yaml; // dump it back into the file
  }
  catch (...) {
    ROS_ERROR("[%s] Error editing YAML file!", name_.c_str());
  }

  

  // Killing Kudan SLAM
  ROS_INFO("[%s] Stopping Kudan Slam", name_.c_str());
  stopLaunch(kudan_slam_launch_id_);
  while (launchExists(kudan_slam_launch_id_))
    ;
  kudan_slam_launch_id_ = 0;
  ROS_INFO("[%s] Kudan Slam stopped", name_.c_str());

  mapping_ = false;

  saved_ = false;

  ros::Duration(3.0).sleep();
  try
  {
    // Copy DB File
    boost::filesystem::path mySourcePath(p_map_name_+".db");
    boost::filesystem::path myTargetPath(map_name_save_+".db");
    boost::filesystem::copy_file(mySourcePath, myTargetPath, boost::filesystem::copy_option::overwrite_if_exists);
    boost::filesystem::remove(p_map_name_+".db");
    ROS_INFO("[%s] DB file has been copied", name_.c_str());
  }
  catch(...)
  {
    ROS_WARN("[%s] Something went wrong while copying DB", name_.c_str());
  }
  
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
task_supervisor::ReturnCode KudanSlamHandler::runTask(movel_seirios_msgs::Task& task, std::string& error_message)
{
  task_active_ = true;
  task_parsed_ = false;
  start_ = ros::Time::now();
  // p_map_name_ = "/home/movel/.config/movel/maps/temp_rtabmap_save_";

  ros::ServiceServer serv_save_ = 
    nh_handler_.advertiseService("/task_supervisor/pcl_slam_handler/save_pcl_map", &KudanSlamHandler::onSaveServiceCall, this);
  ros::ServiceServer serv_save_async_ =
    nh_handler_.advertiseService("/task_supervisor/pcl_slam_handler/save_pcl_map_async", &KudanSlamHandler::onAsyncSave, this);
  save_map_client_ = nh_handler_.serviceClient<kdlidar_ros_msgs::SaveFile>("/kdlidar_ros_pcl/save_map");

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
bool KudanSlamHandler::loadParams()
{
  ros_utils::ParamLoader param_loader(nh_handler_);

  param_loader.get_optional("loop_rate", p_loop_rate_, 5.0);
  param_loader.get_optional("save_timeout", p_save_timeout_, 5.0);
  param_loader.get_optional("map_topic", p_map_topic_, std::string("/map"));
  param_loader.get_optional("resolution", p_resolution_, 0.05);
  param_loader.get_optional("utm", p_utm_, false);

  param_loader.get_required("kudan_slam_launch_package", p_kudan_slam_launch_package_);
  param_loader.get_required("kudan_slam_launch", p_kudan_slam_launch_);
  param_loader.get_required("kudan_map_saver_package", p_map_saver_package_);
  param_loader.get_required("kudan_map_saver_launch", p_map_saver_launch_);
  param_loader.get_required("three_to_two_package", p_3Dto2D_package_);
  param_loader.get_required("three_to_two_launch", p_3Dto2D_launch_);

  param_loader.get_optional("temp_map_name", p_map_name_, std::string("/home/movel/.config/movel/maps/temp_rtabmap_save_"));
  return param_loader.params_valid();
}

/**
 *	Setup method called once task_supervisor starts and loads all plugins (task handlers)
 */
bool KudanSlamHandler::setupHandler()
{
  if (!loadParams())
    return false;

  // Health Check timer
  double timer_rate = 2.0;
  if (p_watchdog_rate_ > 1e-2)
  {
    timer_rate = p_watchdog_rate_;
  }
  map_health_timer_ = nh_handler_.createTimer(ros::Duration(1.0/timer_rate), &KudanSlamHandler::healthTimerCb, this);
  map_health_timer_.stop();

  return true;
}

void KudanSlamHandler::healthTimerCb(const ros::TimerEvent& te)
{
  healthCheck();
}

bool KudanSlamHandler::healthCheck()
{
  static int fail_count = 0;

  if (isTaskActive() && mapping_)
  {
    if (!launchStatus(kudan_slam_launch_id_))
    {
      fail_count++;
      int fail_max_count = 4;
      if(p_watchdog_rate_ > 1.0e-2)
      {
        fail_max_count = 30*p_watchdog_rate_;
      }
      ROS_INFO("[%s] fail count %d/%d", 
               name_.c_str(), fail_count, fail_max_count);
      
      if (fail_count >= fail_max_count)
      {
        ROS_INFO("[%s] Kudan SLAM is unhealthy. Cancel Task!", name_.c_str());

        ROS_INFO("[%s] Stopping Kudan Slam", name_.c_str());
        stopLaunch(kudan_slam_launch_id_);
        while (launchExists(kudan_slam_launch_id_))
          ;
        kudan_slam_launch_id_ = 0;
        ROS_INFO("[%s] Kudan Slam stopped", name_.c_str());
        cancelTask();
        
        mapping_ = false;
        fail_count = 0;
        // kudan_slam_launch_id_ = 0;
        return false;
      }
    }
    else
      fail_count = 0;
  }
  else
  {
    fail_count = 0;
  }
  return true;
}
}  // namespace task_supervisor

