#include <task_supervisor/plugins/localization_handler.h>
#include <pluginlib/class_list_macros.h>         //For pluginlib registration
#include <ros_utils/ros_utils.h>                 //For loadParams function contents
#include <boost/algorithm/string/predicate.hpp>  //For case insensitive string comparison
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
  param_loader.get_optional("map_topic", p_map_topic_, std::string("/map"));
  param_loader.get_optional("map_frame", p_map_frame_, std::string("map"));
  param_loader.get_optional("base_link_frame", p_base_link_frame_, std::string("base_link"));
  param_loader.get_optional("set_map_service", p_set_map_srv_, std::string("/set_map"));

  param_loader.get_required("localization_launch_package", p_localization_launch_package_);
  param_loader.get_required("localization_launch_file", p_localization_launch_file_);
  param_loader.get_required("map_dir", map_dir_);
  param_loader.get_required("localization_launch_nodes", p_localization_launch_nodes_);

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
  map_subscriber_ = nh_handler_.subscribe(p_map_topic_, 1, &LocalizationHandler::mapCB, this);

  // Health Reporter
  health_check_pub_ = nh_handler_.advertise<movel_seirios_msgs::Reports>("/health_reporter", 1);

  // Localization Timer
  loc_health_timer_ = nh_handler_.createTimer(ros::Duration(1.0 / p_timer_rate_), &LocalizationHandler::onHealthTimerCallback, this);
  return true;
}

bool LocalizationHandler::startLocalization(std::string map_path)
{
  // Start amcl launch file
  if (!localizing_.data)
  {
    // Shutdown map subscriber, no longer listening to map produced by mapping
    map_subscriber_.shutdown();

    // If map is specified, change amcl to block on a "get_map" service on node startup
    std::string amcl_args = "";
    if (!map_path.empty())
      amcl_args += " use_map_topic:=false";

    // Start amcl using launch_manager
    ROS_INFO("[%s] Launching localization", name_.c_str());
    localization_launch_id_ = startLaunch(p_localization_launch_package_, p_localization_launch_file_, amcl_args);
    if (!localization_launch_id_)
    {
      ROS_ERROR("[%s] Failed to launch localization launch file", name_.c_str());
      message_ = "Failed to launch localization launch file";
      return false;
    }

    // Start map server if path is specified
    if (!map_path.empty())
    {
      ROS_INFO("[%s] Map file specified, launching map server to load map", name_.c_str());
      map_server_launch_id_ = startLaunch("task_supervisor", "map_server.launch", "file_path:=" + map_path);
      ROS_INFO("[%s] Map server launched", name_.c_str());
      if (!map_server_launch_id_)
      {
        ROS_ERROR("[%s] Failed to launch map server launch file", name_.c_str());
        message_ = "Failed to launch map server  launch file";
        return false;
      }

      std::string map_name = map_path.substr(map_dir_.size() + 1);
      map_name = map_name.substr(0, map_name.find(".yaml"));
      map_name_pub_id_ = startLaunch("task_supervisor", "map_name_pub.launch", "map_name:=" + map_name);
      if (!map_name_pub_id_)
      {
        ROS_ERROR("[%s] Failed to launch map name publisher launch file", name_.c_str());
        message_ = "Failed to launch publisher launch file";
        return false;
      }
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
  if (map_server_launch_id_)
  {
    stopLaunch(map_server_launch_id_, "/map_server");
    stopLaunch(map_name_pub_id_, "/map_name_pub");
    map_server_launch_id_ = 0;
    map_name_pub_id_ = 0;
  }

  ROS_INFO("[%s] Stopping localization", name_.c_str());
  stopLaunch(localization_launch_id_, p_localization_launch_nodes_);
  localization_launch_id_ = 0;

  // Subscribe to map topic when amcl is not running to get latest map from mapping
  map_subscriber_ = nh_handler_.subscribe(p_map_topic_, 1, &LocalizationHandler::mapCB, this);

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
  std::string map_path = "";
  if (boost::iequals(parsed_args.front(), "start"))
  {
    if (parsed_args.size() == 2)
    {
      // Check if file exists
      std::string map_file_abs = map_dir_ + "/" + parsed_args.back();
      FILE* file = fopen(map_file_abs.c_str(), "r");
      if (file == NULL)
      {
        error_message = "[" + name_ + "] Map " + parsed_args.back() + "specified does not exist in " + map_dir_ +
                        " or can't be opened";
        setTaskResult(false);
        return code_;
      }

      map_path = map_file_abs;
    }

    else
    {
      error_message = "[" + name_ + "] Payload incorrect. \nUsage for start: 'start *full path to map*'";
      setTaskResult(false);
      return code_;
    }

    result = startLocalization(map_path);
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

void LocalizationHandler::onHealthTimerCallback(const ros::TimerEvent& timer_event)
{
  if (localizing_.data)
  {
    bool isHealthy = launchStatus(localization_launch_id_);
    if (!isHealthy)
    {
      ROS_INFO("[%s] Some nodes are disconnected", name_.c_str());
      movel_seirios_msgs::Reports report;
      report.header.stamp = ros::Time::now();
      report.handler = "localization_handler";
      report.task_type = 1;
      report.message = "some localization nodes are not running";
      health_check_pub_.publish(report);
      stopLocalization();
    } 
  }
}

}
