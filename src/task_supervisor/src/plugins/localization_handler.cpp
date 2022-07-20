#include <task_supervisor/plugins/localization_handler.h>
#include <pluginlib/class_list_macros.h>         //For pluginlib registration
#include <ros_utils/ros_utils.h>                 //For loadParams function contents
#include <boost/algorithm/string/predicate.hpp>  //For case insensitive string comparison
#include <boost/filesystem.hpp>             //Check if file exists
#include <ros/master.h>                          //For checking currently running nodes
#include <string.h>                              //Payload parsing
#include <stdio.h>                               //Check if file exists

PLUGINLIB_EXPORT_CLASS(task_supervisor::LocalizationHandler, task_supervisor::TaskHandler);

namespace task_supervisor
{
LocalizationHandler::LocalizationHandler()
{
  localizing_.data = false;
}

bool LocalizationHandler::onStatus(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
  res.success = launchStatus(localization_launch_id_);
  return true;
}

bool LocalizationHandler::startLocalizationCB(movel_seirios_msgs::StringTrigger::Request& req,
                                              movel_seirios_msgs::StringTrigger::Response& res)
{
  ROS_INFO("[%s] Start localization service called", name_.c_str());
  movel_seirios_msgs::Task task;
  std::string error_message;
  task.type = 1;
  task.payload = std::string("start");

  if (!req.input.empty())
    task.payload = task.payload + " " + req.input;

  if (runTask(task, error_message) == ReturnCode::SUCCESS)
    res.success = true;
  else
    res.success = false;

  res.message = message_;
  return true;
}

bool LocalizationHandler::stopLocalizationCB(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res)
{
  ROS_INFO("[%s] Stop localization service called", name_.c_str());
  movel_seirios_msgs::Task task;
  std::string error_message;
  task.type = 1;
  task.payload = std::string("stop");

  if (runTask(task, error_message) == ReturnCode::SUCCESS)
    res.success = true;
  else
    res.success = false;

  res.message = message_;
  return true;
}

// Callback for map topic
void LocalizationHandler::mapCB(const nav_msgs::OccupancyGrid::ConstPtr& map)
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

bool LocalizationHandler::loadParams()
{
  ros_utils::ParamLoader param_loader(nh_handler_);
  
  param_loader.get_optional("watchdog_rate", p_timer_rate_, 2.0);

  param_loader.get_optional("loop_rate", p_loop_rate_, 5.0);
  param_loader.get_optional("set_map_timeout", p_set_map_timeout_, 10.0);

  param_loader.get_optional("localization_map_topic", p_loc_map_topic_, std::string("/map"));
  param_loader.get_optional("navigation_map_topic", p_nav_map_topic_, std::string("/map_nav"));

  param_loader.get_optional("map_frame", p_map_frame_, std::string("map"));
  param_loader.get_optional("base_link_frame", p_base_link_frame_, std::string("base_link"));
  param_loader.get_optional("set_map_service", p_set_map_srv_, std::string("/set_map"));

  param_loader.get_optional("large_map", p_large_map_, false);
  param_loader.get_optional("large_map_mode", p_large_map_mode_, 0);

  param_loader.get_required("localization_launch_package", p_localization_launch_package_);
  param_loader.get_required("localization_launch_file", p_localization_launch_file_);

  param_loader.get_required("localization_map_dir", loc_map_dir_);
  param_loader.get_required("navigation_map_dir", nav_map_dir_);
  
  param_loader.get_required("localization_launch_nodes", p_localization_launch_nodes_);
  
  if (nh_handler_.hasParam("/task_supervisor/rtabmap_handler/camera_names"))
    nh_handler_.getParam("/task_supervisor/rtabmap_handler/camera_names", p_camera_names_);
  else
    return false;
    
  if (p_camera_names_.size() == 0)
    return false; 

  param_loader.get_optional("orb_slam", p_orb_slam_, false);
  param_loader.get_optional("rgb_color_topic", p_rgb_color_topic_, std::string("/camera/rgb/image_raw" ));
  param_loader.get_optional("rgbd_depth_topic", p_rgbd_depth_topic_, std::string("/camera/depth_registered/image_raw"));
  param_loader.get_optional("rgbd_camera_info", p_rgbd_camera_info_, std::string("/camera/rgb/camera_info"));

  param_loader.get_optional("use_aruco", p_use_aruco_, false);

  if(p_orb_slam_)
  {
    param_loader.get_required("orb_loc_launch_package", p_orb_loc_launch_package_);
    param_loader.get_required("orb_loc_launch_file", p_orb_loc_launch_file_);
    param_loader.get_required("orb_loc_launch_nodes", p_orb_loc_launch_nodes_);
  }
  
  return param_loader.params_valid();
}

// Called on handler initialization
bool LocalizationHandler::setupHandler()
{
  if (!loadParams())
    return false;

  // Publish latched topic of localization state
  localizing_pub_ = nh_handler_.advertise<std_msgs::Bool>("localizing", 1, true);
  localizing_pub_.publish(localizing_);

  //Advertise service for manually starting and stopping localization
  start_srv_serv_ = nh_handler_.advertiseService("start", &LocalizationHandler::startLocalizationCB, this);
  stop_srv_serv_ = nh_handler_.advertiseService("stop", &LocalizationHandler::stopLocalizationCB, this);
  status_srv_serv_ = nh_handler_.advertiseService("status", &LocalizationHandler::onStatus, this);

  // Subscribe to map topic that is usually published by a mapping node, for example gmapping
  map_subscriber_ = nh_handler_.subscribe(p_loc_map_topic_, 1, &LocalizationHandler::mapCB, this);

  // Advertise for relaunch navigation map after the map getting edited
  relaunch_serv_ = nh_handler_.advertiseService("relaunch_map", &LocalizationHandler::relaunchMapCb, this);

  // Call service for clearing costmaps
  clear_costmap_serv_ = nh_handler_.serviceClient<std_srvs::Empty>("/move_base/clear_costmaps");

  // Localization Timer
  loc_health_timer_ = nh_handler_.createTimer(ros::Duration(1.0 / p_timer_rate_), &LocalizationHandler::onHealthTimerCallback, this);
  return true;
}

bool LocalizationHandler::startLocalization()
{
  // Start amcl launch file
  if (!localizing_.data)
  {
    // Shutdown map subscriber, no longer listening to map produced by mapping
    map_subscriber_.shutdown();

    // If map is specified, change amcl to block on a "get_map" service on node startup
    std::string amcl_args = "";
    if (!loc_map_path_.empty())
      amcl_args += " use_map_topic:=false";

    std::string map_name = loc_map_path_.substr(loc_map_dir_.size() + 1);
    map_name = map_name.substr(0, map_name.find(".yaml"));

    // For rtabmap launch arguments
    std::string rtabmap_args= " database_path:=" + loc_map_dir_ + "/" + map_name + ".db";

    size_t camera_quantity = p_camera_names_.size();
    std::string camera_names_args, camera_quantity_args;

    if(camera_quantity >= 1 && camera_quantity <= 3)
      camera_quantity_args = " rgbd_cameras:=" + std::to_string((int)camera_quantity);
    else if(camera_quantity > 3)
      camera_quantity_args = " rgbd_cameras:=" + std::to_string((int)4);

    if(camera_quantity == 1)
      camera_names_args = " camera:=" + p_camera_names_[0];
    else if(camera_quantity == 2)
      camera_names_args = " camera1:=" + p_camera_names_[0] +
                          " camera2:=" + p_camera_names_[1];
    else if (camera_quantity == 3)
      camera_names_args = " camera1:=" + p_camera_names_[0] +
                          " camera2:=" + p_camera_names_[1] +
                          " camera3:=" + p_camera_names_[2];
    else if (camera_quantity >= 4)
      camera_names_args = " camera1:=" + p_camera_names_[0] +
                          " camera2:=" + p_camera_names_[1] +
                          " camera3:=" + p_camera_names_[2] +
                          " camera4:=" + p_camera_names_[3];

    rtabmap_args += camera_names_args + camera_quantity_args;

    // Start amcl using launch_manager
    ROS_INFO("[%s] Launching localization", name_.c_str());
    localization_launch_id_ = startLaunch(p_localization_launch_package_, p_localization_launch_file_, amcl_args + rtabmap_args);
    if (!localization_launch_id_)
    {
      ROS_ERROR("[%s] Failed to launch localization launch file", name_.c_str());
      message_ = "Failed to launch localization launch file";
      return false;
    }

    if(p_use_aruco_)
    {
      std::string aruco_path = loc_map_path_.substr(0, loc_map_path_.find("yaml"));
      aruco_path += "txt";
      ROS_INFO("[%s] aruco_path including txt", aruco_path.c_str());
      ros::ServiceClient aruco_client = nh_handler_.serviceClient<movel_seirios_msgs::StringTrigger>("/movel_aruco_amcl/load_aruco");
      movel_seirios_msgs::StringTrigger load_aruco;
      load_aruco.request.input = aruco_path;
      aruco_client.call(load_aruco);
      if (load_aruco.response.success == false)
      {
          ROS_ERROR("[%s] Failed to load aruco file", aruco_path.c_str());
      }
    }

    // Start map server if path is specified
    if (!loc_map_path_.empty())
    {
      ros::ServiceClient speed_zone_client = nh_handler_.serviceClient<movel_seirios_msgs::StringTrigger>("/mongo_bridge/get_speed_zones");
      movel_seirios_msgs::StringTrigger speed_zone_srv;
      speed_zone_srv.request.input = map_name;
      if(!speed_zone_client.call(speed_zone_srv))
      {
        ROS_ERROR("[%s] Failed to call /mongo_bridge/get_speed_zones service", name_.c_str());
      }

      if (!p_large_map_)
      {
        // Start Localization Map
        ROS_INFO("[%s] Localization Map file specified, launching map server to load map", name_.c_str());
        loc_map_server_launch_id_ = startLaunch("task_supervisor", "map_server.launch", "file_path:=" + loc_map_path_);      
        ROS_INFO("[%s] Localization Map server launched", name_.c_str());
        if (!loc_map_server_launch_id_)
        {
          ROS_ERROR("[%s] Failed to launch localization map server launch file", name_.c_str());
          message_ = "Failed to launch localization map server launch file";
          return false;
        }

        // Start Navigation Map
        nav_map_server_launch_id_ = startLaunch("task_supervisor", "map_server_nav.launch", "file_path:=" + nav_map_path_ );
        if (!nav_map_server_launch_id_)
        {
          ROS_ERROR("[%s] Failed to launch navigation map server launch file", name_.c_str());
          message_ = "Failed to launch navigation map server launch file";
          return false;
        }
      }
      else
      {
        if (p_large_map_mode_ == 0) // full res localization, low res navigation
        {
          // full res localization
          ROS_INFO("[%s] Localization Map file specified, launching map server to load map", name_.c_str());
          loc_map_server_launch_id_ = startLaunch("task_supervisor", "map_server.launch", "file_path:=" + loc_map_path_);      
          ROS_INFO("[%s] Localization Map server launched", name_.c_str());
          if (!loc_map_server_launch_id_)
          {
            ROS_ERROR("[%s] Failed to launch localization map server launch file", name_.c_str());
            message_ = "Failed to launch localization map server launch file";
            return false;
          }

          // low res navigation
          std::string nav_map_stem;
          std::string key(".yaml");
          std::size_t idx = loc_map_path_.rfind(key);
          nav_map_stem = loc_map_path_.substr(0, idx);
          
          std::string nav_map_path = nav_map_stem + "/scaled.yaml";
          nav_map_server_launch_id_ = startLaunch("task_supervisor", "map_server_nav.launch", "file_path:="+nav_map_path);
          if (!nav_map_server_launch_id_)
          {
            ROS_ERROR("[%s] Failed to launch navigation map server launch file", name_.c_str());
            message_ = "Failed to launch navigation map server launch file";
            return false;
          }
        }
        else if (p_large_map_mode_ == 1) // swappable localization, low res navigation
        {
          std::string nav_map_stem;
          std::string key(".yaml");
          std::size_t idx = loc_map_path_.rfind(key);
          nav_map_stem = loc_map_path_.substr(0, idx);

          // swappable localization
          // setup init_map param
          std::string map_swapper_args = "init_map:="+nav_map_stem;
          ROS_INFO("map_swapper args %s", map_swapper_args.c_str());

          // bringup map_swapper
          loc_map_server_launch_id_ = startLaunch("map_swapper", "loc_map_swapper.launch", map_swapper_args);
          if (!loc_map_server_launch_id_)
          {
            ROS_ERROR("[%s] Failed to launch map swapper with args %s", name_.c_str(), map_swapper_args.c_str());
            message_ = "Failed to launch map swapper";
            return false;
          }

          // low res navigation
          nav_map_path_ = nav_map_stem + "/scaled.yaml";
          nav_map_server_launch_id_ = startLaunch("task_supervisor", "map_server_nav.launch", "file_path:="+nav_map_path_);
          if (!nav_map_server_launch_id_)
          {
            ROS_ERROR("[%s] Failed to launch navigation map server launch file", name_.c_str());
            message_ = "Failed to launch navigation map server launch file";
            return false;
          }
        }
        else if (p_large_map_mode_ == 2)
        {
          // swappable localization and navigation
          ROS_INFO("[%s] large map mode 2 is not implemented. Try another one", name_.c_str());
          return false;
        }
        else
        {
          ROS_INFO("[%s] %d is an invalid large map mode. Try another one", name_.c_str(), p_large_map_mode_);
          return false;
        }
      }

      // map_name_pub_id_ = startLaunch("task_supervisor", "map_name_pub.launch", "map_name:=" + map_name);
      // if (!map_name_pub_id_)
      // {
      //   ROS_ERROR("[%s] Failed to launch map name publisher launch file", name_.c_str());
      //   message_ = "Failed to launch publisher launch file";
      //   return false;
      // }

      // map_editor_id_ = startLaunch("map_editor", "map_editor.launch", "");
      
      // if (!map_editor_id_)
      // {
      //   ROS_ERROR("[%s] Failed to launch map editor launch file", name_.c_str());
      //   message_ = "Failed to launch map editor launch file";
      //   return false;
      // }else{
      //   ROS_INFO("Map editor launched in localization handler");
      // }
    }

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

    if(p_orb_slam_)
    {
      if(!boost::filesystem::exists( loc_map_dir_ + "/" + map_name + ".csv" ))
      {
        ROS_WARN("[%s] Could not find orbslam transform, going into normal localization", name_.c_str());
      }
      else 
      {

        std::string orb_map_name_ = "map_name:=" + loc_map_dir_ + "/" + map_name;
        std::string rgb_color_topic_ = " rgb_color_topic:=" + p_rgb_color_topic_;
        std::string rgbd_depth_topic_ = " rgbd_depth_topic:=" + p_rgbd_depth_topic_;
        std::string rgbd_camera_info_topic_ = " rgbd_camera_info:=" + p_rgbd_camera_info_;

        orb_loc_launch_id_ = startLaunch(p_orb_loc_launch_package_, p_orb_loc_launch_file_, orb_map_name_ + 
                                                                                            rgb_color_topic_ +
                                                                                            rgbd_depth_topic_ +
                                                                                            rgbd_camera_info_topic_);
        if (!orb_loc_launch_id_)
        {
          ROS_ERROR("[%s] Failed to launch orbslam localization launch file", name_.c_str());
          return false;
        }
      }
    }
  }
  localizing_.data = true;
  localizing_pub_.publish(localizing_);
  return true;
}

bool LocalizationHandler::stopLocalization()
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
    // stopLaunch(map_editor_id_, "/map_editor");
    loc_map_server_launch_id_ = 0;
    nav_map_server_launch_id_ = 0;
    map_name_pub_id_ = 0;
    // map_editor_id_ = 0;
  }

  if(p_orb_slam_)
  {
    stopLaunch(orb_loc_launch_id_, p_orb_loc_launch_nodes_);
    orb_loc_launch_id_ = 0;
    ROS_INFO("[%s] Orb slam localization stopped", name_.c_str());
  } 

  ROS_INFO("[%s] Stopping localization", name_.c_str());
  stopLaunch(localization_launch_id_, p_localization_launch_nodes_);
  localization_launch_id_ = 0;

  // Subscribe to map topic when amcl is not running to get latest map from mapping
  map_subscriber_ = nh_handler_.subscribe(p_loc_map_topic_, 1, &LocalizationHandler::mapCB, this);

  localizing_.data = false;
  localizing_pub_.publish(localizing_);
  return true;
}

std::vector<std::string> LocalizationHandler::parseArgs(std::string payload)
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

ReturnCode LocalizationHandler::runTask(movel_seirios_msgs::Task& task, std::string& error_message)
{
  task_active_ = true;
  task_parsed_ = false;
  start_ = ros::Time::now();

  // Parse payload to check if map_path is specified
  std::vector<std::string> parsed_args = parseArgs(task.payload);

  // Check first arg of task payload to see if start or stop, uses boost case insensitive string compare
  bool result;

  // loc_map and nav_map path
  loc_map_path_ = "";
  nav_map_path_ = "";
  
  if (boost::iequals(parsed_args.front(), "start"))
  {
    if (parsed_args.size() == 2)
    {
      // Check if file exists
      std::string loc_map_file_abs = loc_map_dir_ + "/" + parsed_args.back();
      FILE* file_loc = fopen(loc_map_file_abs.c_str(), "r");
      if (file_loc == NULL)
      {
        error_message = "[" + name_ + "] Map for localization: " + parsed_args.back() + "specified does not exist in " + loc_map_dir_ +
                        " or can't be opened";
        setTaskResult(false);
        return code_;
      }

      std::string nav_map_file_abs = nav_map_dir_ + "/" + parsed_args.back();
      FILE* file_nav = fopen(nav_map_file_abs.c_str(), "r");
      if (file_nav == NULL)
      {
        ROS_ERROR("[%s] Navigation map not found. Using localization map for navigation.", name_.c_str());
        
        // If nav map can't be opened, use loc map instead
        nav_map_path_ = loc_map_file_abs;
      }

      // Assign loc map path
      loc_map_path_ = loc_map_file_abs; 

      // Assign nav map path if the file can be opened
      if (nav_map_path_ == "")
        nav_map_path_ = nav_map_file_abs;

    }

    else
    {
      error_message = "[" + name_ + "] Payload incorrect. \nUsage for start: 'start *full path to map*'";
      setTaskResult(false);
      return code_;
    }

    result = startLocalization();
  }

  else if (boost::iequals(parsed_args.front(), "stop"))
  {
    if (parsed_args.size() > 1)
    {
      error_message = "[" + name_ + "] Payload incorrect. \nUsage for stop: 'stop'";
      setTaskResult(false);
      return code_;
    }

    result = stopLocalization();
  }

  else
  {
    result = false;
    error_message = "[" + name_ + "] Payload command format invalid, input 'start *optional full path to map*' or "
                                  "'stop'";
  }

  setTaskResult(result);
  return code_;
}

bool LocalizationHandler::relaunchMapCb(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res)
{
  if (nav_map_server_launch_id_)
  {
    ROS_INFO("[%s] Relaunching Map Server for Navigation!", name_.c_str());
    nav_map_server_launch_id_ = startLaunch("task_supervisor", "map_server_nav.launch", "file_path:=" + nav_map_path_ );
    if (!nav_map_server_launch_id_)
    {
      ROS_ERROR("[%s] Failed to launch navigation map server launch file", name_.c_str());
      message_ = "Failed to launch navigation map server  launch file";
      return false;
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

void LocalizationHandler::onHealthTimerCallback(const ros::TimerEvent& timer_event)
{
  if (localizing_.data)
  {
    bool isHealthy = launchStatus(localization_launch_id_);
    if (!isHealthy)
    {
      ROS_INFO("[%s] Some nodes are disconnected", name_.c_str());
      stopLocalization();
    } 
  }
}

}
