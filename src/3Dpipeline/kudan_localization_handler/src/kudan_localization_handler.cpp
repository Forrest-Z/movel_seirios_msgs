#include <kudan_localization_handler/kudan_localization_handler.h>

#include <pluginlib/class_list_macros.h>         //For pluginlib registration
#include <ros_utils/ros_utils.h>                 //For loadParams function contents
#include <boost/algorithm/string/predicate.hpp>  //For case insensitive string comparison
#include <ros/master.h>                          //For checking currently running nodes
#include <string.h>                              //Payload parsing
#include <stdio.h>                               //Check if file exists

PLUGINLIB_EXPORT_CLASS(task_supervisor::KudanLocalizationHandler, task_supervisor::TaskHandler);

namespace task_supervisor
{
KudanLocalizationHandler::KudanLocalizationHandler() : tf_ear_(tf_buffer_)
{
  localizing_.data = false;
}

bool KudanLocalizationHandler::onStatus(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
  res.success = launchStatus(localization_launch_id_);
  return true;
}
/** SERVICE CALL FOR START-STOP LOCALIZATION **/
bool KudanLocalizationHandler::startLocalizationCB(movel_seirios_msgs::StringTrigger::Request& req,
                                              movel_seirios_msgs::StringTrigger::Response& res)
{
  ROS_INFO("[%s] Start localization service called", name_.c_str());
  movel_seirios_msgs::Task task;
  std::string error_message;
  task.type = 31;
  task.payload = std::string("start");

  preempted_by_user_ = false;
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

bool KudanLocalizationHandler::stopLocalizationCB(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res)
{
  ROS_INFO("[%s] Stop localization service called", name_.c_str());
  movel_seirios_msgs::Task task;
  std::string error_message;
  task.type = 31;
  task.payload = std::string("stop");

  preempted_by_user_ = true;
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

  loc_health_timer_.stop();

  return true;
}

// Callback for map topic (For checking purposes)
void KudanLocalizationHandler::mapCB(const nav_msgs::OccupancyGrid::ConstPtr& map)
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

void KudanLocalizationHandler::costmapProhibCB(const dynamic_reconfigure::Config::ConstPtr& msg)
{
  if (pcl_localization_)
  {
  ROS_INFO("[KudanLocalizationHandler] Costmap prohibition callback initiated");
  prohib_layer_client_ = nh_handler_.serviceClient<movel_seirios_msgs::StringTrigger>("/prohibition_layer_mongo/get_prohib_layer");  
  movel_seirios_msgs::StringTrigger prohib_layer_srv;
  prohib_layer_srv.request.input = map_name_;
  if(!prohib_layer_client_.call(prohib_layer_srv))
  {
    ROS_ERROR("[%s] Failed to call /prohibition_layer_mongo/get_prohib_layer service", name_.c_str());
  }
  }
}

void KudanLocalizationHandler::initialposeCB(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
  geometry_msgs::PointStamped initial_pose;
  initial_pose.header = msg->header;
  initial_pose.point.x = msg->pose.pose.position.x;
  initial_pose.point.y = msg->pose.pose.position.y;
  clicked_points_pub_.publish(initial_pose);
}

bool KudanLocalizationHandler::loadParams()
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

  // Normal Localization launch package
  param_loader.get_required("kudan_localization_launch_package", p_localization_launch_package_);
  param_loader.get_required("kudan_localization_launch_file", p_localization_launch_file_);

  param_loader.get_required("localization_map_dir", loc_map_dir_);
  param_loader.get_required("navigation_map_dir", nav_map_dir_);

  // Move base launch package
  param_loader.get_required("move_base_launch_package", p_move_base_launch_package_);
  param_loader.get_required("move_base_launch_file", p_move_base_launch_file_);


  // Optional
  param_loader.get_required("kudan_localization_launch_nodes", p_localization_launch_nodes_);

  // Point Based Mapping
  param_loader.get_optional("temp_map_name", p_map_name_, std::string("/home/movel/.config/movel/maps/temp_rtabmap_save_"));
  
  return param_loader.params_valid();
}

// Called on handler initialization
bool KudanLocalizationHandler::setupHandler()
{
  if (!loadParams())
    return false;

  // Publish latched topic of localization state
  localizing_pub_ = nh_handler_.advertise<std_msgs::Bool>("/task_supervisor/pcl_localization_handler/localizing", 1, true);
  localizing_pub_.publish(localizing_);

  // Health Report
  double timer_rate = 2.0;
  if (p_watchdog_rate_ > 1e-2)
  {
    timer_rate = p_watchdog_rate_;
  }
  loc_health_timer_ = nh_handler_.createTimer(ros::Duration(1.0/timer_rate), &KudanLocalizationHandler::healthTimerCb, this);
  loc_health_timer_.stop();

  //Advertise service for manually starting and stopping localization
  start_srv_serv_ = nh_handler_.advertiseService("/task_supervisor/pcl_localization_handler/start", &KudanLocalizationHandler::startLocalizationCB, this);
  stop_srv_serv_ = nh_handler_.advertiseService("/task_supervisor/pcl_localization_handler/stop", &KudanLocalizationHandler::stopLocalizationCB, this);
  status_srv_serv_ = nh_handler_.advertiseService("/task_supervisor/pcl_localization_handler/status", &KudanLocalizationHandler::onStatus, this);

  // Subscribe to map topic that is usually published by a mapping node, for example gmapping
  map_subscriber_ = nh_handler_.subscribe(p_loc_map_topic_, 1, &KudanLocalizationHandler::mapCB, this);

 // Advertise for relaunch navigation map after the map getting edited
  relaunch_serv_ = nh_handler_.advertiseService("/task_supervisor/pcl_localization_handler/relaunch_map", &KudanLocalizationHandler::relaunchMapCb, this);

  // Call service for clearing costmaps
  clear_costmap_serv_ = nh_handler_.serviceClient<std_srvs::Empty>("/move_base/clear_costmaps");

  initpose_pub_ = nh_handler_.advertise<geometry_msgs::PoseWithCovarianceStamped>("/initialpose",10);
  timeout_pub_ = nh_handler_.advertise<std_msgs::Bool>("/task_supervisor/pcl_localization_handler/timeout_flag",1);

  // Task cancel publisher
  cancel_task_ = nh_handler_.advertise<actionlib_msgs::GoalID>("/task_supervisor/cancel",1 );

  //Subscriber for costmap prohibition layer
  prohib_layer_subscriber_ = nh_handler_.subscribe("/move_base/local_costmap/costmap_prohibition_layer/parameter_updates", 1, &KudanLocalizationHandler::costmapProhibCB, this);

  //Subscriber to publish initial pose
  initial_pose_sub = nh_handler_.subscribe("/initialpose",1,&KudanLocalizationHandler::initialposeCB, this);
  clicked_points_pub_ = nh_handler_.advertise<geometry_msgs::PointStamped>("/clicked_point",1);

  return true;
}

/* --------------------------------
        NORMAL 3D LOCALIZATION
   --------------------------------
*/
bool KudanLocalizationHandler::startLocalization()
{
  ROS_INFO("Started!");

  if (!localizing_.data)
  {
    // Shutdown map subscriber, no longer listening to map produced by mapping
    map_subscriber_.shutdown();

    //Check if 3d localization is enabled
    pcl_localization_ = true;

    // Movebase launch file payload maker.
    std::string launch_args = "";
    std::string move_base_launch_args = "";
    if (!loc_map_path_.empty())
    {
      launch_args += " map_path:="+loc_map_path_+".kdlm";
    }
    
    // Launch 3d movebase launch file
    ROS_INFO("[%s] Launching 3D localization", name_.c_str());
    localization_launch_id_ = startLaunch(p_localization_launch_package_, p_localization_launch_file_, launch_args);
    move_base_launch_id_ = startLaunch(p_move_base_launch_package_, p_move_base_launch_file_, move_base_launch_args);
  
    if (!localization_launch_id_)
    {
      ROS_ERROR("[%s] Failed to launch localization launch file", name_.c_str());
      message_ = "Failed to launch localization launch file";
      return false;
    }
    
    if (!move_base_launch_id_)
    {
      ROS_ERROR("[%s] Failed to launch move_base launch file", name_.c_str());
      message_ = "Failed to launch move_base launch file";
      return false;
    }

    // Start map server if path is specified
    if (!loc_map_path_.empty())
    {
      ros::ServiceClient speed_zone_client = nh_handler_.serviceClient<movel_seirios_msgs::StringTrigger>("/mongo_bridge/get_speed_zones");
      movel_seirios_msgs::StringTrigger speed_zone_srv;
      speed_zone_srv.request.input = map_name_;
      if(!speed_zone_client.call(speed_zone_srv))
      {
        ROS_ERROR("[%s] Failed to call /mongo_bridge/get_speed_zones service", name_.c_str());
      }

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

    }
    /**
     * OBSOLETE. ITS AMCL node.
     * **/
    // No map path specified
  }
  localizing_.data = true;
  localizing_pub_.publish(localizing_);
  return true;
}

bool KudanLocalizationHandler::stopLocalization()
{
  if (!localizing_.data)
  {
    ROS_WARN("[%s] Could not stop, localization was not started", name_.c_str());
    return false;
  }

  //Check if 3d localization is enabled
  pcl_localization_ = false;

  // Close map server if it was opened
  if (loc_map_server_launch_id_ && nav_map_server_launch_id_)
  {
    stopLaunch(loc_map_server_launch_id_, "/map_server");
    stopLaunch(nav_map_server_launch_id_, "/map_server_nav");
    stopLaunch(map_name_pub_id_, "/map_name_pub");
    // stopLaunch(map_editor_id_, "/map_editor");
    loc_map_server_launch_id_ = 0;    
    nav_map_server_launch_id_ = 0;
    map_name_pub_id_ = 0;
    // map_editor_id_ = 0;
  }

  ROS_INFO("[%s] Stopping localization", name_.c_str());
  stopLaunch(localization_launch_id_, p_localization_launch_nodes_);
  localization_launch_id_ = 0;

  ROS_INFO("[%s] Stopping move_base", name_.c_str());
  stopLaunch(move_base_launch_id_, "/move_base");
  move_base_launch_id_ = 0;
  

  // Subscribe to map topic when amcl is not running to get latest map from mapping
  map_subscriber_ = nh_handler_.subscribe(p_loc_map_topic_, 1, &KudanLocalizationHandler::mapCB, this);

  localizing_.data = false;
  localizing_pub_.publish(localizing_);

  loc_health_timer_.stop();
  return true;
}

std::vector<std::string> KudanLocalizationHandler::parseArgs(std::string payload)
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

ReturnCode KudanLocalizationHandler::runTask(movel_seirios_msgs::Task& task, std::string& error_message)
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
      map_name_ = file_wo_ext;
      std::string loc_yaml = loc_map_file;
      std::string pgm = loc_map_dir_ + "/" + file_wo_ext + ".pgm";
      // std::string pcd = loc_map_dir_ + "/" + file_wo_ext + ".pcd";
      std::string kdlm = loc_map_dir_ + "/" + file_wo_ext + ".kdlm";
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
bool KudanLocalizationHandler::relaunchMapCb(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res)
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
void KudanLocalizationHandler::healthTimerCb(const ros::TimerEvent& te)
{
  healthCheck();
}

bool KudanLocalizationHandler::healthCheck()
{
  static int fail_count = 0;
  int fail_max_count = p_watchdog_rate_ > 1.0e-2 ? 20 * p_watchdog_rate_ : 4;

  bool healthy = true;
  std::string logs = "";
  healthy = healthy && launchStatus(localization_launch_id_, !preempted_by_user_);
  if (!healthy && logs.empty())
    logs += "localization nodes down ";
  healthy = healthy && launchStatus(loc_map_server_launch_id_, !preempted_by_user_);
  if (!healthy && logs.empty())
    logs += "localization map_server down ";
  healthy = healthy && launchStatus(nav_map_server_launch_id_, !preempted_by_user_);
  if (!healthy && logs.empty())
    logs += "navigation map_server down ";
  if (!healthy && !preempted_by_user_)
  {
    fail_count += 1;
    ROS_INFO("[%s] fail count %d/%d", 
              name_.c_str(), fail_count, fail_max_count);
    if (fail_count >= fail_max_count)
    {
      stopLocalization();
      fail_count = 0;
      return false;
    }
  }
  else
  {
    fail_count = 0;
  }
  
  return true;
}

}
