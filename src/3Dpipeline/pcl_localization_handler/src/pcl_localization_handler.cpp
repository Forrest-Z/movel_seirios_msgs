#include <pcl_localization_handler/pcl_localization_handler.h>
#include <pluginlib/class_list_macros.h>         //For pluginlib registration
#include <ros_utils/ros_utils.h>                 //For loadParams function contents
#include <boost/algorithm/string/predicate.hpp>  //For case insensitive string comparison
#include <ros/master.h>                          //For checking currently running nodes
#include <string.h>                              //Payload parsing
#include <stdio.h>                               //Check if file exists

PLUGINLIB_EXPORT_CLASS(task_supervisor::PCLLocalizationHandler, task_supervisor::TaskHandler);

namespace task_supervisor
{
PCLLocalizationHandler::PCLLocalizationHandler()
{
  localizing_.data = false;
}

bool PCLLocalizationHandler::onStatus(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
  res.success = launchStatus(localization_launch_id_);
  return true;
}
/** SERVICE CALL FOR START-STOP LOCALIZATION **/
bool PCLLocalizationHandler::startLocalizationCB(movel_seirios_msgs::StringTrigger::Request& req,
                                              movel_seirios_msgs::StringTrigger::Response& res)
{
  ROS_INFO("[%s] Start localization service called", name_.c_str());
  movel_seirios_msgs::Task task;
  std::string error_message;
  task.type = 31;
  task.payload = std::string("start");

  // Add map name
  if (!req.input.empty())
    task.payload = task.payload + " " + req.input;

  // Start localization
  if (runTask(task, error_message) == ReturnCode::SUCCESS)
  {
    res.success = true;
  }
  else
  {
    res.success = false;
    res.message = message_;
  }

  return true;
}

bool PCLLocalizationHandler::stopLocalizationCB(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res)
{
  ROS_INFO("[%s] Stop localization service called", name_.c_str());
  movel_seirios_msgs::Task task;
  std::string error_message;
  task.type = 31;
  task.payload = std::string("stop");

  // Stop Localization
  if (runTask(task, error_message) == ReturnCode::SUCCESS)
  {
    res.success = true;
  }
  else
  {
    res.success = false;
    res.message = message_;
  }

  return true;
}

// Callback for map topic (For checking purposes)
void PCLLocalizationHandler::mapCB(const nav_msgs::OccupancyGrid::ConstPtr& map)
{
  map_ = *map;

  try
  {
    // Lookup transform for map to odom when mapping is running
    // Tranform from base to map because pose is inverse of transform
    tf_listener_.lookupTransform(p_map_frame_, p_base_link_frame_, ros::Time(0), tf_base_to_map_);
  }
  catch (tf::TransformException& ex)
  {
    ROS_ERROR("[%s] Transform error: %s", name_.c_str(), ex.what());
  }

  // Convert from transform to pose
  pose_.header.stamp = ros::Time::now();
  pose_.header.frame_id = tf_base_to_map_.frame_id_;  // TODO: Check if this is correct

  pose_.pose.pose.position.x = tf_base_to_map_.getOrigin().getX();
  pose_.pose.pose.position.y = tf_base_to_map_.getOrigin().getY();
  pose_.pose.pose.position.z = tf_base_to_map_.getOrigin().getZ();

  pose_.pose.pose.orientation.x = tf_base_to_map_.getRotation().getX();
  pose_.pose.pose.orientation.y = tf_base_to_map_.getRotation().getY();
  pose_.pose.pose.orientation.z = tf_base_to_map_.getRotation().getZ();
  pose_.pose.pose.orientation.w = tf_base_to_map_.getRotation().getW();
}

bool PCLLocalizationHandler::loadParams()
{
  ros_utils::ParamLoader param_loader(nh_handler_);

  // Task Parameters
  param_loader.get_optional("loop_rate", p_loop_rate_, 5.0);
  param_loader.get_optional("set_map_timeout", p_set_map_timeout_, 10.0);

  // Map Topic
  param_loader.get_optional("localization_map_topic", p_loc_map_topic_, std::string("/map"));
  param_loader.get_optional("navigation_map_topic", p_nav_map_topic_, std::string("/map_nav"));
  param_loader.get_optional("map_frame", p_map_frame_, std::string("map"));
  param_loader.get_optional("base_link_frame", p_base_link_frame_, std::string("base_link"));

  param_loader.get_optional("set_map_service", p_set_map_srv_, std::string("/set_map"));

  // Normal Localization laucnh package
  param_loader.get_required("pcl_localization_launch_package", p_localization_launch_package_);
  param_loader.get_required("pcl_localization_launch_file", p_localization_launch_file_);

  param_loader.get_required("localization_map_dir", loc_map_dir_);
  param_loader.get_required("navigation_map_dir", nav_map_dir_);

  // Optional
  param_loader.get_required("pcl_localization_launch_nodes", p_localization_launch_nodes_);

  // Dynamic Mapping package
  param_loader.get_required("dynamic_mapping_move_base_launch_file", p_dyn_map_move_base_launch_file_);

  param_loader.get_required("dynamic_mapping_launch_package", p_dyn_map_launch_package_);
  param_loader.get_required("dynamic_mapping_launch_file", p_dyn_map_launch_file_);

  // Map Server 
  param_loader.get_required("dynamic_map_saver_package", p_map_saver_package_);
  param_loader.get_required("dynamic_map_saver_launch", p_map_saver_launch_);

  return param_loader.params_valid();
}

// Called on handler initialization
bool PCLLocalizationHandler::setupHandler()
{
  if (!loadParams())
    return false;

  // Publish latched topic of localization state
  localizing_pub_ = nh_handler_.advertise<std_msgs::Bool>("localizing", 1, true);
  localizing_pub_.publish(localizing_);

  // Health Report
  loc_health_pub_ = nh_handler_.advertise<movel_seirios_msgs::Reports>("/task_supervisor/health_report", 1);
  double timer_rate = 2.0;
  if (p_watchdog_rate_ > 1e-2)
  {
    timer_rate = p_watchdog_rate_;
  }
  loc_health_timer_ = nh_handler_.createTimer(ros::Duration(1.0/timer_rate), &PCLLocalizationHandler::healthTimerCb, this);
  loc_health_timer_.stop();

  //Advertise service for manually starting and stopping localization
  start_srv_serv_ = nh_handler_.advertiseService("start", &PCLLocalizationHandler::startLocalizationCB, this);
  stop_srv_serv_ = nh_handler_.advertiseService("stop", &PCLLocalizationHandler::stopLocalizationCB, this);
  status_srv_serv_ = nh_handler_.advertiseService("status", &PCLLocalizationHandler::onStatus, this);

  // Subscribe to map topic that is usually published by a mapping node, for example gmapping
  map_subscriber_ = nh_handler_.subscribe(p_loc_map_topic_, 1, &PCLLocalizationHandler::mapCB, this);

 // Advertise for relaunch navigation map after the map getting edited
  relaunch_serv_ = nh_handler_.advertiseService("relaunch_map", &PCLLocalizationHandler::relaunchMapCb, this);

  // Call service for clearing costmaps
  clear_costmap_serv_ = nh_handler_.serviceClient<std_srvs::Empty>("/move_base/clear_costmaps");

  // Dynamic Mapping Service
  dyn_mapping_start_serv_ = nh_handler_.advertiseService("start_dynamic_mapping", &PCLLocalizationHandler::startDynamicMappingCB, this);
  dyn_mapping_save_serv_ = nh_handler_.advertiseService("save_dynamic_mapping", &PCLLocalizationHandler::saveDynamicMappingCB, this);
  dyn_mapping_cancel_serv_ = nh_handler_.advertiseService("cancel_dynamic_mapping", &PCLLocalizationHandler::cancelDynamicMappingCB, this);

  start_dyn_mapping_ = nh_handler_.serviceClient<std_srvs::Empty>("/rtabmap/set_mode_mapping");
  stop_dyn_mapping_ = nh_handler_.serviceClient<std_srvs::Empty>("/rtabmap/set_mode_localization");

  return true;
}

/* --------------------------------
        NORMAL 3D LOCALIZATION
   --------------------------------
*/
bool PCLLocalizationHandler::startLocalization()
{
  ROS_INFO("Started!");

  if (!localizing_.data)
  {
    // Shutdown map subscriber, no longer listening to map produced by mapping
    map_subscriber_.shutdown();

    // Movebase launch file payload maker.
    std::string launch_args = "";
    if (!loc_map_path_.empty())
    {
      launch_args += " use_map_topic:=false";
      launch_args += " pcd:="+loc_map_path_+".pcd";
      launch_args += " database_path:="+loc_map_path_+".db";
    }
    
    // Launch 3d movebase launch file
    ROS_INFO("[%s] Launching 3D localization", name_.c_str());
    localization_launch_id_ = startLaunch(p_localization_launch_package_, p_localization_launch_file_, launch_args);
    if (!localization_launch_id_)
    {
      ROS_ERROR("[%s] Failed to launch localization launch file", name_.c_str());
      message_ = "Failed to launch localization launch file";
      return false;
    }

    // Start map server if path is specified
    if (!loc_map_path_.empty())
    {
      ROS_INFO("[%s] Localization Map file specified, launching map server to load map", name_.c_str());
      loc_map_server_launch_id_ = startLaunch("task_supervisor", "map_server.launch", "file_path:=" + loc_map_path_ +".yaml");
      ROS_INFO("[%s] Localization Map server launched", name_.c_str());
      if (!loc_map_server_launch_id_)
      {
        ROS_ERROR("[%s] Failed to launch localization map server launch file", name_.c_str());
        message_ = "Failed to launch localization map server launch file";
        return false;
      }

      // Start Navigation Map
      ROS_INFO("[%s] Navigation Map file specified, launching map server to load map", name_.c_str());
      nav_map_server_launch_id_ = startLaunch("task_supervisor", "map_server_nav.launch", "file_path:=" + nav_map_path_);
      ROS_INFO("[%s] Navigation Map server launched", name_.c_str());
      if (!nav_map_server_launch_id_)
      {
        ROS_ERROR("[%s] Failed to launch navigation map server launch file", name_.c_str());
        message_ = "Failed to launch navigation map server launch file";
        return false;
      }

      // Start Map name pub
      map_name_pub_id_ = startLaunch("task_supervisor", "map_name_pub.launch", "map_name:=" + loc_map_path_);
      if (!map_name_pub_id_)
      {
        ROS_ERROR("[%s] Failed to launch map name publisher launch file", name_.c_str());
        message_ = "Failed to launch publisher launch file";
        return false;
      }

      // Start map editor
      map_editor_id_ = startLaunch("map_editor", "map_editor.launch", "is_3d:=true");
      if (!map_editor_id_)
      {
        ROS_ERROR("[%s] Failed to launch map editor launch file", name_.c_str());
        message_ = "Failed to launch map editor launch file";
        return false;
      }
    }
    /**
     * OBSOLETE. ITS AMCL node.
     * **/
    // No map path specified
    else            
    {
      set_map_client_ = nh_handler_.serviceClient<nav_msgs::SetMap>(p_set_map_srv_);
      if (!map_.data.empty())
      {
        // map_ var not empty and no file defined, wait for set_map service from amcl to set map
        if (!ros::service::waitForService(p_set_map_srv_, ros::Duration(p_set_map_timeout_)))
        {
          ROS_ERROR("[%s] Set map service from amcl not available, timed out. Unable to set initial pose and map",
                    name_.c_str());
          message_ = "Set map service from amcl not available, timed out. Unable to set initial pose and map";
        }

        else
        {
          ROS_INFO("[%s] Set map and initial pose", name_.c_str());
          nav_msgs::SetMap set_map;
          set_map.request.initial_pose = pose_;
          set_map.request.map = map_;
          set_map_client_.call(set_map);
        }
      }
    }
  }
  localizing_.data = true;
  localizing_pub_.publish(localizing_);
  return true;
}

bool PCLLocalizationHandler::stopLocalization()
{
  if (!localizing_.data)
  {
    ROS_WARN("[%s] Could not stop, localization was not started", name_.c_str());
    return false;
  }

  // Close map server if it was opened
  if (loc_map_server_launch_id_ && nav_map_server_launch_id_)
  {
    stopLaunch(loc_map_server_launch_id_, "/map_server");
    stopLaunch(nav_map_server_launch_id_, "/map_server_nav");
    stopLaunch(map_name_pub_id_, "/map_name_pub");
    stopLaunch(map_editor_id_, "/map_editor");
    loc_map_server_launch_id_ = 0;    
    nav_map_server_launch_id_ = 0;
    map_name_pub_id_ = 0;
    map_editor_id_ = 0;
  }

  ROS_INFO("[%s] Stopping localization", name_.c_str());
  stopLaunch(localization_launch_id_, p_localization_launch_nodes_);
  localization_launch_id_ = 0;

  if (isDynamicMapping_)
  {
    isDynamicMapping_ = false;
    ROS_INFO("[%s] Stopping dynamic mapping", name_.c_str());
    stopLaunch(dynamic_map_launch_id_, p_localization_launch_nodes_);
    dynamic_map_launch_id_ = 0;
    // Delete DB
    try
    {
      boost::filesystem::remove(loc_map_path_+"_temp.db");
    }
    catch (const boost::filesystem::filesystem_error& e)
    {
      ROS_ERROR("[%s] ERROR: %s ", name_.c_str(),  e.code().message().c_str());
    }
  }

  // Subscribe to map topic when amcl is not running to get latest map from mapping
  map_subscriber_ = nh_handler_.subscribe(p_loc_map_topic_, 1, &PCLLocalizationHandler::mapCB, this);

  localizing_.data = false;
  localizing_pub_.publish(localizing_);

  loc_health_timer_.stop();
  return true;
}

std::vector<std::string> PCLLocalizationHandler::parseArgs(std::string payload)
{
  std::vector<std::string> parsed_args;
  std::string delim = " ";
  size_t pos;

  // Check if there is at least 1 occurence of delim
  if (payload.find(delim) != std::string::npos)
  {
    // Append space to the back, otherwise while loop clears all args except last
    payload = payload + " ";

    while ((pos = payload.find(delim)) != std::string::npos)
    {
      std::string token = payload.substr(0, pos);
      parsed_args.push_back(token);
      payload.erase(0, pos + delim.length());
    }
  }

  else
    parsed_args.push_back(payload);
  return parsed_args;
}

ReturnCode PCLLocalizationHandler::runTask(movel_seirios_msgs::Task& task, std::string& error_message)
{
  task_active_ = true;
  task_parsed_ = false;
  start_ = ros::Time::now();
  ROS_INFO("1. Parsing payload: %s", task.payload.c_str());

  // Parse payload to check if map_path is specified
  std::vector<std::string> parsed_args = parseArgs(task.payload);
  ROS_INFO("2. Payload parsed, executing '%s' command", parsed_args.front().c_str());

  // Check first arg of task payload to see if start or stop, uses boost case insensitive string compare
  bool result;

  // Initialize loc_map_path and nav_map path
  loc_map_path_ = "";
  nav_map_path_ = "";
  
  if (boost::iequals(parsed_args.front(), "start"))       // START Localization
  {
    if (parsed_args.size() == 2)
    {
      std::string loc_map_file = parsed_args.back();
      std::string::size_type wo_ext = loc_map_file.rfind(".yaml");
      std::string file_wo_ext = loc_map_file.substr(0, wo_ext);
      std::string loc_yaml = loc_map_file;
      std::string pgm = loc_map_dir_ + "/" + file_wo_ext + ".pgm";
      // std::string pcd = loc_map_dir_ + "/" + file_wo_ext + ".pcd";
      std::string loc_map_file_abs = loc_map_dir_ + "/" + loc_yaml;
      ROS_INFO("3. Loc map: %s", loc_map_file_abs.c_str());

      // Navigation map file name parse
      std::string nav_yaml = loc_map_file;
      std::string nav_map_file_abs = nav_map_dir_ + "/" + nav_yaml;
      ROS_INFO("4 Nav map: %s", nav_map_file_abs.c_str());

      // Check if file exists
      FILE* loc_yaml_file = fopen(loc_map_file_abs.c_str(), "r");
      FILE* pgm_file = fopen(pgm.c_str(), "r");
      // FILE* pcd_file = fopen(pcd.c_str(), "r");

      if (loc_yaml_file == NULL)
      {
        error_message = "[" + name_ + "] Map for localization: " + parsed_args.back() + "specified does not exist in " + loc_map_dir_ +
                        " or can't be opened";
	      ROS_ERROR("[%s] %s does not exist", name_.c_str(), loc_yaml.c_str() );
        setTaskResult(false);
        return code_;
      }
      else if (pgm_file == NULL)
      {
        error_message = "[" + name_ + "] Map for localzation " + parsed_args.back() + "specified does not exist in " + loc_map_dir_ +
                        " or can't be opened";
	      ROS_ERROR("[%s] %s does not exist", name_.c_str(), pgm.c_str() );
        setTaskResult(false);
        return code_;
      }
      // else if (pcd_file == NULL)
      // {
      //   error_message = "[" + name_ + "] Map for localization " + parsed_args.back() + "specified does not exist in " + loc_map_dir_ +
      //                   " or can't be opened";
	    //   ROS_ERROR("[%s] %s does not exist", name_.c_str(), pcd.c_str() );
      //   setTaskResult(false);
      //   return code_;
      // }

      FILE* nav_yaml_file = fopen(nav_map_file_abs.c_str(), "r");
      if (nav_yaml_file == NULL)
      {
	      ROS_ERROR("[%s] Navigation map not found. Using localization map for navigation.", name_.c_str());
        
        // If nav map can't be opened, use loc map instead
        nav_map_path_ = loc_map_file_abs;
      }

      loc_map_path_ = loc_map_dir_ + "/" + file_wo_ext;

      // Assign nav map path if the file can be opened
      if (nav_map_path_ == "")
        nav_map_path_ = nav_map_file_abs;

      fclose(loc_yaml_file);
      fclose(pgm_file);
      // fclose(pcd_file);
      if ( !(nav_yaml_file == NULL))
        fclose(nav_yaml_file);
    }
    else
    {
      error_message = "[" + name_ + "] Payload incorrect. \nUsage for start: 'start *full path to map*'";
      setTaskResult(false);
      return code_;
    }

    result = startLocalization();
  }

  else if (boost::iequals(parsed_args.front(), "stop"))           // STOP localization
  {
    if (parsed_args.size() > 1)
    {
      error_message = "[" + name_ + "] Payload incorrect. \nUsage for stop: 'stop'";
      setTaskResult(false);
      return code_;
    }

    result = stopLocalization();
  }

  else          // Invalid Payload
  {
    result = false;
    error_message = "[" + name_ + "] Payload command format invalid, input 'start *optional full path to map*' or "
                                  "'stop'";
  }

  // begin monitoring localization health
  loc_health_timer_.start();

  setTaskResult(result);
  return code_;
}

/* --------------------------------
        MAP EDITOR RELAUNCH MAP_NAV
   --------------------------------
*/
bool PCLLocalizationHandler::relaunchMapCb(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res)
{
  if (nav_map_server_launch_id_)
  {
    ROS_INFO("[%s] Relaunching Map Server for Navigation!", name_.c_str());
    nav_map_server_launch_id_ = startLaunch("task_supervisor", "map_server_nav.launch", "file_path:=" + nav_map_path_ );
    if (!nav_map_server_launch_id_)
    {
      ROS_ERROR("[%s] Failed to launch navigation map server launch file", name_.c_str());
      message_ = "Failed to launch navigation map server  launch file";
      res.success = false;
      res.message = message_;
      return true;
    }
    ROS_INFO("[%s] Clearing Costmaps", name_.c_str());
    // Clear costmaps
    std_srvs::Empty clear_cm;
    if (!clear_costmap_serv_.call(clear_cm))
    {
      ROS_ERROR("[%s] Failed to clear costmaps", name_.c_str());
    }
  }
  res.success = true;
  return true;
}

/* --------------------------------
        HEALTH CHECK THINGY
   --------------------------------
*/
void PCLLocalizationHandler::healthTimerCb(const ros::TimerEvent& te)
{
  healthCheck();
}

bool PCLLocalizationHandler::healthCheck()
{
  static int fail_count = 0;
  bool healthy = true;
  if (localizing_.data)
  {
    healthy = healthy && launchStatus(localization_launch_id_);
    if (!healthy)
      // ROS_INFO("[%s] localization down", name_.c_str());
    healthy = healthy && launchStatus(loc_map_server_launch_id_);
    if (!healthy)
      // ROS_INFO("[%s] map server down", name_.c_str());
    healthy = healthy && launchStatus(nav_map_server_launch_id_);
    if (!healthy)
    {
      fail_count += 1;
      int fail_max_count = 4;
      if (p_watchdog_rate_ > 1.0e-2)
      {
        fail_max_count = 2*p_watchdog_rate_;
      }
      ROS_INFO("[%s] fail count %d/%d", 
               name_.c_str(), fail_count, fail_max_count);
      if (fail_count >= fail_max_count)
      {
        // prep report
        movel_seirios_msgs::Reports report;
        report.header.stamp = ros::Time::now();
        report.handler = "pcl_localization_handler";
        report.task_type = task_type_;
        report.healthy = healthy;
        report.message = "one or more localization node is down";
        loc_health_pub_.publish(report);

        stopLocalization();

        return false;
      }
    }
    else
    {
      fail_count = 0;
    }
  }
  return true;
}

/* --------------------------------
        DYNAMIC MAPPING
   --------------------------------
*/

/* ------------ START -------------*/
bool PCLLocalizationHandler::startDynamicMappingLaunch()
{
  if(!localizing_.data)
  {
    // copy db file to temp
    try
    {
      boost::filesystem::path mySourcePath(loc_map_path_+".db");
      boost::filesystem::path myTargetPath(loc_map_path_+"_temp.db");
      boost::filesystem::copy_file(mySourcePath, myTargetPath, boost::filesystem::copy_option::overwrite_if_exists);
      ROS_INFO("[%s] Make a copy of db file!", name_.c_str());
    }
    catch (const boost::filesystem::filesystem_error& e)
    {
      std::cout << "Error Message: " << e.code().message() << '\n';
      message_ = e.code().message();
    }

    // Create payload for move base
    std::string launch_args = "";
    if (!loc_map_path_.empty())
    {
      launch_args += " database_path:="+loc_map_path_+"_temp.db";
    }

    // Launch 3d movebase launch file
    ROS_INFO("[%s] Launching Move Base Launch File", name_.c_str());
    localization_launch_id_ = startLaunch(p_localization_launch_package_, p_dyn_map_move_base_launch_file_, launch_args);
    if (!localization_launch_id_)
    {
      ROS_ERROR("[%s] Failed to launch move base launch file", name_.c_str());
      message_ = "Failed to launch move base launch file";
      return false;
    }

    // Dynamic mapping launch
    launch_args += " localization:=true";
    ROS_INFO("[%s] Launching Mapping Launch File", name_.c_str());
    dynamic_map_launch_id_ = startLaunch(p_dyn_map_launch_package_, p_dyn_map_launch_file_, launch_args);\
    if (!dynamic_map_launch_id_)
    {
      ROS_ERROR("[%s] Failed to launch mapping launch file", name_.c_str());
      message_ = "Failed to launch mapping launch file";
      return false;
    }

  }
  localizing_.data = true;
  return true;
}

bool PCLLocalizationHandler::startDynamicMappingCB(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res)
{
  if (!localizing_.data && !isDynamicMapping_)
  {
    ROS_ERROR("[%s] Could not start dynamic mapping, localization was not started. Please start localization first!", name_.c_str());
    message_ = "Localization was not started. Please start localization first!";
    res.success = false;
    res.message = message_;
    return true;
  }
  else if (localizing_.data && isDynamicMapping_)
  {
    ROS_ERROR("[%s] Dynamic mapping is already running", name_.c_str());
    message_ = "Dynamic mapping is already running";
    res.success = false;
    res.message = message_;
    return true;
  }

  loc_health_timer_.stop();

  // Kill map_server and its friend
  if (loc_map_server_launch_id_ && nav_map_server_launch_id_)
  {
    stopLaunch(loc_map_server_launch_id_, "/map_server");
    stopLaunch(nav_map_server_launch_id_, "/map_server_nav");
    stopLaunch(map_name_pub_id_, "/map_name_pub");
    stopLaunch(map_editor_id_, "/map_editor");
    loc_map_server_launch_id_ = 0;    
    nav_map_server_launch_id_ = 0;
    map_name_pub_id_ = 0;
    map_editor_id_ = 0;
  }

  // Stopping localization node (RTABMAP)
  ROS_INFO("[%s] Stopping localization", name_.c_str());
  stopLaunch(localization_launch_id_, p_localization_launch_nodes_);
  localization_launch_id_ = 0;
  localizing_.data = false;

  // Relaunch new movebase launch
  bool status = startDynamicMappingLaunch();
  if(!status)
  {
    ROS_INFO("[%s] Failed to launch dynamic mapping!",name_.c_str());
    message_ = "Failed to launch dynamic mapping!";
    localizing_.data =false;
    stopLaunch(localization_launch_id_, p_localization_launch_nodes_);
    localization_launch_id_ = 0;
    res.success = false;
    res.message = message_;
    return true;
  }

  // Waiting for the service to come up
  ros::service::waitForService("/rtabmap/set_mode_mapping", ros::Duration(20));
  ros::service::waitForService("/move_base/clear_costmaps", ros::Duration(20));

  // Call service to switch mode to mapping
  std_srvs::Empty switch_mode;
  if(!start_dyn_mapping_.call(switch_mode))
  {
    ROS_INFO("[%s] Failed to switch mode to mapping mode", name_.c_str());
  }

  ROS_INFO("[%s] Dynamic Mapping has been started !", name_.c_str());
  isDynamicMapping_ = true;

  res.success = true;
  return true;
}

/* ------------ SAVE -------------*/
bool PCLLocalizationHandler::saveDynamicMappingCB(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res)
{
  if (!localizing_.data || !isDynamicMapping_)
  {
    ROS_ERROR("[%s] Could not save the updated map, please start dynamic mapping first!", name_.c_str());
    message_ = "Could not save the updated map, please start dynamic mapping first!";
    res.success = false;
    res.message = message_;
    return true;
  }

  isDynamicMapping_ = false;
  // Call service to switch mode to localization
  std_srvs::Empty switch_mode;
  if(!stop_dyn_mapping_.call(switch_mode))
  {
    ROS_INFO("[%s] Failed to switch mode to localization mode", name_.c_str());
    message_ = "Failed to launch dynamic mapping!";
    res.success = false;
    res.message = message_;
    return true;
  }

  // Save 2d map. 
  bool status = save2Dmap();
  if (!status)
  {
    ROS_ERROR("[%s] Failed to update 2D map!", name_.c_str());
    message_ = "Failed to update the 2Dmap!";
    res.success = false;
    res.message = message_;
    return true;
  }

  // Kill dynamic mapping move_base
  ROS_INFO("[%s] Stopping movebase launch", name_.c_str());
  stopLaunch(localization_launch_id_, p_localization_launch_nodes_);
  localization_launch_id_ = 0;

  ROS_INFO("[%s] Stopping dynamic mapping", name_.c_str());
  stopLaunch(dynamic_map_launch_id_, p_localization_launch_nodes_);
  dynamic_map_launch_id_ = 0;
  localizing_.data = false;

  // Copy map
  try
  {
    boost::filesystem::path mySourcePath(loc_map_path_+"_temp.db");
    boost::filesystem::path myTargetPath(loc_map_path_+".db");
    boost::filesystem::copy_file(mySourcePath, myTargetPath, boost::filesystem::copy_option::overwrite_if_exists);
    boost::filesystem::remove(loc_map_path_+"_temp.db");
    ROS_INFO("[%s] Save the temporary db file to the original one!", name_.c_str());
  }
  catch (const boost::filesystem::filesystem_error& e)
  {
    ROS_ERROR("[%s] ERROR: %s ", name_.c_str(),  e.code().message().c_str());
    message_ = e.code().message();
    res.success = false;
    res.message = message_;
    return true;
  }

  // Relaunch all the things
  bool result = startLocalization();
  if (!result)
  {
    ROS_ERROR("[%s] Error while starting localization. Please reload the map!", name_.c_str());
    message_ = "Error while starting localization. Please reload the map!";
    res.success = false;
    res.message = message_;
    return true;
  }
  res.success = true;
  return true;
}

bool PCLLocalizationHandler::save2Dmap()
{
  std::string launch_args = " map_topic:=/map";
  if (!loc_map_path_.empty())
  {
    launch_args = launch_args + " file_path:=" + loc_map_path_;
    launch_args = launch_args + " file_path_nav:=" + nav_map_path_;
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
  while (ros::Time::now().toSec() - start_time.toSec() < 5.0)
  {
    // TODO map_saver might die before saving
    if (!launchExists(map_saver_id))
    {
      ROS_INFO("[%s] Save complete", name_.c_str());
      // stopLaunch(conversion_id);

      ROS_INFO("[%s] Checking for 3D and 2D map files", name_.c_str());
      
      // 2D map checking
      FILE* pgm = fopen( (loc_map_path_ + ".pgm").c_str(), "r");
      if (pgm == NULL)
      {
          ROS_ERROR("[%s] 2D map file is NOT FOUND in %s", name_.c_str(), (loc_map_path_ + ".pgm").c_str());
          return false;
      }
      fclose(pgm);
      ROS_INFO("[%s] 2D map file is AVAILABLE in %s", name_.c_str(), (loc_map_path_ + ".pgm").c_str());

      return true;
    }
    r.sleep();
  }
  return true;
}

/* ------------ CANCEL -------------*/
bool PCLLocalizationHandler::cancelDynamicMappingCB(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res)
{
  if (!localizing_.data || !isDynamicMapping_)
  {
    ROS_ERROR("[%s] Dynamic Mapping as not started. Nothing happened", name_.c_str());
    message_ = "Dynamic Mapping as not started. Nothing happened";
    res.success = false;
    res.message = message_;
    return true;
  }

  isDynamicMapping_ = false;
   // Call service to switch mode to localization
  std_srvs::Empty switch_mode;
  if(!stop_dyn_mapping_.call(switch_mode))
  {
    ROS_INFO("[%s] Failed to switch mode to localization mode", name_.c_str());
    message_ = "Failed to switch mode to localization mode";
    res.success = false;
    res.message = message_;
    return true;
  }

  // Stopping Dynamic Mapping
  ROS_INFO("[%s] Stopping localization", name_.c_str());
  stopLaunch(localization_launch_id_, p_localization_launch_nodes_);
  localization_launch_id_ = 0;
  
  ROS_INFO("[%s] Stopping dynamic mapping", name_.c_str());
  stopLaunch(dynamic_map_launch_id_, p_localization_launch_nodes_);
  dynamic_map_launch_id_ = 0;
  localizing_.data = false;

  // Delete DB
  try
  {
    boost::filesystem::remove(loc_map_path_+"_temp.db");
  }
  catch (const boost::filesystem::filesystem_error& e)
  {
    ROS_ERROR("[%s] ERROR: %s ", name_.c_str(),  e.code().message().c_str());
  }

  // relaunch original localiation
  bool result = startLocalization();
  if (!result)
  {
    ROS_ERROR("[%s] Error while starting localization. Please reload the map!", name_.c_str());
    message_ = "Error while starting localization. Please reload the map!";
    res.success = false;
    res.message = message_;
    return true;
  }
  res.success = true;
  return true;
}


}
