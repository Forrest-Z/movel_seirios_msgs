#include <pluginlib/class_list_macros.h>
#include <task_supervisor/plugins/multi_session_mapping_handler.h>
#include <ros_utils/ros_utils.h>  //For loadParams function contents

PLUGINLIB_EXPORT_CLASS(task_supervisor::MultiSessionMappingHandler, task_supervisor::TaskHandler);

namespace task_supervisor
{

bool MultiSessionMappingHandler::onStartHandlerCall(movel_seirios_msgs::StringTrigger::Request& req,
                                                    movel_seirios_msgs::StringTrigger::Response& res)
{
  ROS_INFO("[%s] Start multi-session mapping handler service called; starting handler.", name_.c_str());

  // parse request input data
  json input = json::parse(req.input);
  current_map_id_ = input.at("mapId").get<std::string>();
  std::string error_msg = "";

  // stop localization
  ROS_INFO("[%s] Stopping localization handler.", name_.c_str());
  ros::ServiceClient stop_loc_client = nh_handler_.serviceClient<std_srvs::Trigger>("/task_supervisor/localization_handler/stop");
  std_srvs::Trigger stop_loc;
  if (!stop_loc_client.call(stop_loc))
  {
    error_msg = "Failed to stop localization handler";
    res.success = false;
    res.message = error_msg;
    ROS_WARN("[%s] Start multi-session mapping error: %s.", name_.c_str(), error_msg.c_str());
    return true;
  }

  // start map_expander launch
  map_path_ = "";
  std::string map_file_abs = p_map_dir_ + "/" + current_map_id_ + ".yaml";

  ROS_INFO("[%s] Checking map file at %s.", name_.c_str(), map_file_abs.c_str());

  FILE* file_loc = fopen(map_file_abs.c_str(), "r");
  if (file_loc == NULL)
  {
    error_msg = "Map file at " + map_file_abs + " not found";
    res.success = false;
    res.message = error_msg;
    ROS_WARN("[%s] Start multi-session mapping error: %s.", name_.c_str(), error_msg.c_str());
    return true;
  }

  map_path_ = map_file_abs;

  map_expander_launch_id_ = startLaunch(p_multi_session_mapping_launch_package_, p_map_expander_launch_file_, "previous_map_path:=" + map_path_);
  if (!map_expander_launch_id_)
  {
    error_msg = "Failed to launch nodes from " + p_map_expander_launch_file_;
    res.success = false;
    res.message = error_msg;
    ROS_WARN("[%s] Start multi-session mapping error: %s.", name_.c_str(), error_msg.c_str());
    return true;
  }

  // start mapping and navigation launch
  ROS_INFO("[%s] Starting mapping launch: %s", name_.c_str(), p_mapping_launch_file_.c_str());
  
  mapping_launch_id_ = startLaunch(p_multi_session_mapping_launch_package_, p_mapping_launch_file_, "");
  if(!mapping_launch_id_)
  {
    error_msg = "Failed to launch mapping";
    res.success = false;
    res.message = error_msg;
    ROS_WARN("[%s] Start multi-session mapping error: %s.", name_.c_str(), error_msg.c_str());
    return true;
  }

  ROS_INFO("[%s] Starting navigation launch: %s", name_.c_str(), p_dyn_mapping_nav_launch_file_.c_str());
  dyn_mapping_nav_launch_id_ = startLaunch(p_multi_session_mapping_launch_package_, p_dyn_mapping_nav_launch_file_, "");
  if (!dyn_mapping_nav_launch_id_)
  {
    error_msg = "Failed to launch navigation";
    res.success = false;
    res.message = error_msg;
    ROS_WARN("[%s] Start multi-session mapping error: %s.", name_.c_str(), error_msg.c_str());
    return true;
  }

  mapping_started_ = true;
  started_via_service_ = true;
  res.success = true;
  return true;
}

bool MultiSessionMappingHandler::onStartMappingCall(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res)
{
  ROS_INFO("[%s] Starting mapping launch: %s", name_.c_str(), p_mapping_launch_file_.c_str());
  
  mapping_launch_id_ = startLaunch(p_multi_session_mapping_launch_package_, p_mapping_launch_file_, "");
  if(!mapping_launch_id_)
  {
    ROS_ERROR("[%s] Failed to start mapping.", name_.c_str());
    res.success = false;
    return true;
  }

  ROS_INFO("[%s] Starting navigation launch: %s", name_.c_str(), p_dyn_mapping_nav_launch_file_.c_str());
  dyn_mapping_nav_launch_id_ = startLaunch(p_multi_session_mapping_launch_package_, p_dyn_mapping_nav_launch_file_, "");
  if (!dyn_mapping_nav_launch_id_)
  {
    ROS_ERROR("[%s] Failed to start navigation.", name_.c_str());
    res.success = false;
    return true;
  }

  mapping_started_ = true;
  res.success = true;
  return true;
}

/**
 * Callback for /task_supervisor/multi_session_mapping_handler/multisession_save_map. Once save_map service is called, this method
 * starts the map_saver launch file and sets the map_topic depending on the yaml config file of task_supervisor.
 *
 * Where the map is saved is determined during the call to save_map service, a fully defined path can be passed
 * as an argument, otherwise the default location of $HOME is used.
 *
 * Default location can be modified in the launch file of map_saver, found in
 * (task_supervisor package)/launch/map_saver.launch
 */
bool MultiSessionMappingHandler::onSaveServiceCall(movel_seirios_msgs::StringTrigger::Request& req,
                                                   movel_seirios_msgs::StringTrigger::Response& res)
{
  if (!mapping_started_)
  {
    res.success = false;
    res.message = "Mapping not started yet";
    return true;
  }

  // parse request input data
  json input = json::parse(req.input);
  bool resume = input.at("resume").get<bool>();
  std::string error_msg = "";

  std::string full_map_name = p_map_dir_ + "/" + current_map_id_;

  res.success = saveMap(full_map_name, error_msg);
  res.message = error_msg;

  if (!resume)
  {
    if (started_via_service_)
      stopAll();
    else
      saved_ = true;
  }

  return true;
}

bool MultiSessionMappingHandler::onStatus(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res)
{
  res.success = launchStatus(mapping_launch_id_) && launchStatus(dyn_mapping_nav_launch_id_);
  return true;
}

bool MultiSessionMappingHandler::saveMap(std::string map_name, std::string& error_msg)
{
  // Set path to save file
  std::string launch_args = " map_topic:=" + p_merged_map_topic_;
  
  if (!map_name.empty())
  {
    launch_args = launch_args + " file_path:=" + map_name;
    std::string map_nav_name (map_name);
    std::string key ("/");
    std::size_t idx = map_name.rfind(key);
    if (idx != std::string::npos)
    {
      map_nav_name.replace(idx, key.length(), "/nav/");
      ROS_INFO("map name %s", map_name.c_str());
      ROS_INFO("map nav name %s", map_nav_name.c_str());
      launch_args = launch_args + " file_path_nav:=" + map_nav_name;
    }
  }

  ROS_INFO("launch args %s", launch_args.c_str());
  // Call map saving through launch manager service
  ROS_INFO("[%s] Saving map %s", name_.c_str(), map_name.size() != 0 ? ("to" + map_name).c_str() : "");
  unsigned int map_saver_id = startLaunch("task_supervisor", "map_saver.launch", launch_args);
  
  // Check if startLaunch succeeded
  if (!map_saver_id)
  {
    error_msg = "Failed to start map saver";
    ROS_ERROR("[%s] %s", name_.c_str(), error_msg.c_str());
    return false;
  }

  // While loop until timeout
  ros::Time start_time = ros::Time::now();
  ros::Rate r(p_loop_rate_);
  while (ros::Time::now().toSec() - start_time.toSec() < p_save_timeout_)
  {
    // TODO map_saver might die before saving
    if (!launchExists(map_saver_id))
    {
      ROS_INFO("[%s] Save complete", name_.c_str());
      return true;
    }

    r.sleep();
  }

  error_msg = "Timeout occurred";
  ROS_WARN("[%s] Timeout occurred, save failed", name_.c_str());
  stopLaunch(map_saver_id);
  return false;
}

bool MultiSessionMappingHandler::run(std::string payload, std::string& error_message)
{
  // Run map expander asynchronously
  ROS_INFO("[%s] Starting map expander package: %s, launch file: %s", name_.c_str(), p_multi_session_mapping_launch_package_.c_str(),
           p_map_expander_launch_file_.c_str());

  // Parse payload to check if map_path is specified
  std::vector<std::string> parsed_args = parseArgs(payload);

  map_path_ = "";

  if (parsed_args.size() == 1)
  {
    current_map_id_ = parsed_args.back();
    std::string map_file_abs = p_map_dir_ + "/" + current_map_id_ + ".yaml"; // map directory + previous_map + .yaml
    FILE* file_loc = fopen(map_file_abs.c_str(), "r");
    if (file_loc == NULL)
    {
      error_message = "[" + name_ + "] Load previous map: " + parsed_args.back() + " specified does not exist in " + p_map_dir_ +
                      " or can't be opened";
      ROS_ERROR("%s", error_message.c_str());
      return false;
    }

    map_path_ = map_file_abs;
  }
  else
  {
    error_message = "[" + name_ + "] Payload incorrect. \nUsage for start: 'start *full path to map*'";
    ROS_ERROR("%s", error_message.c_str());
    return false;
  }

  map_expander_launch_id_ = startLaunch(p_multi_session_mapping_launch_package_, p_map_expander_launch_file_, "previous_map_path:=" + map_path_);
  if (!map_expander_launch_id_)
  {
    error_message = "[" + name_ + "] Failed to launch map expander launch file";
    ROS_ERROR("%s", error_message.c_str());
    // ROS_ERROR("[%s] Failed to launch map expander launch file", name_.c_str());
    return false;
  }

  // Loop until mapping is called
  ros::Rate r(p_loop_rate_);
  while (!mapping_started_)
  {
    // Check if cancellation has been called
    if (!isTaskActive())
    {
      ROS_INFO("[%s] Waiting for mapping thread to exit", name_.c_str());
      stopLaunch(map_expander_launch_id_);
      while (launchExists(map_expander_launch_id_))
        ;
      return false;
    }

    r.sleep();
  }

  // Loop until save map is called
  while (!saved_)
  {
    // Check if cancellation has been called
    if (!isTaskActive())
    {
      ROS_INFO("[%s] Waiting for mapping thread to exit", name_.c_str());
      stopLaunch(mapping_launch_id_);
      while (launchExists(mapping_launch_id_))
        ;
      stopLaunch(dyn_mapping_nav_launch_id_);
      while (launchExists(dyn_mapping_nav_launch_id_))
        ;
      stopLaunch(map_expander_launch_id_);
      while (launchExists(map_expander_launch_id_))
        ;
      return false;
    }

    r.sleep();
  }

  ROS_INFO("[%s] Stopping mapping", name_.c_str());
  stopLaunch(mapping_launch_id_);
  while (launchExists(mapping_launch_id_))
    ;
  stopLaunch(dyn_mapping_nav_launch_id_);
  while (launchExists(dyn_mapping_nav_launch_id_))
    ;
  stopLaunch(map_expander_launch_id_);
  while (launchExists(map_expander_launch_id_))
    ;

  map_expander_launch_id_ = 0;
  dyn_mapping_nav_launch_id_ = 0;
  mapping_launch_id_ = 0;
  ROS_INFO("[%s] Mapping stopped", name_.c_str());

  saved_ = false;
  mapping_started_ = false;
  return true;
  //
}

/* When stop is called, map will be saved to a temporary file path. Then stop the launches. */
bool MultiSessionMappingHandler::onStopCall(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res) {
  ROS_INFO("[%s] Stopping multi session mapping", name_.c_str());

  stopAll();

  res.success = true;
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
ReturnCode MultiSessionMappingHandler::runTask(movel_seirios_msgs::Task& task, std::string& error_message)
{
  task_active_ = true;
  task_parsed_ = false;
  start_ = ros::Time::now();

  ros::ServiceServer serv_mapping_ = nh_handler_.advertiseService("goal/multisession_start_mapping", &MultiSessionMappingHandler::onStartMappingCall, this);

  bool mapping_done = run(task.payload, error_message);

  // Reset all state variables and shutdown services
  setTaskResult(mapping_done);

  return code_;
}

std::vector<std::string> MultiSessionMappingHandler::parseArgs(std::string payload)
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

/**
 * @brief Stop all launchfiles
 */
void MultiSessionMappingHandler::stopAll()
{
  if (mapping_launch_id_ != 0)
  {
    stopLaunch(mapping_launch_id_);
    while (launchExists(mapping_launch_id_))
      ;
    mapping_launch_id_ = 0;
  }
  
  if (dyn_mapping_nav_launch_id_ != 0)
  {
    stopLaunch(dyn_mapping_nav_launch_id_);
    while (launchExists(dyn_mapping_nav_launch_id_))
      ;
    dyn_mapping_nav_launch_id_ = 0;
  }

  if (map_expander_launch_id_ != 0)
  {
    stopLaunch(map_expander_launch_id_);
    while (launchExists(map_expander_launch_id_))
      ;
    map_expander_launch_id_ = 0;
  }

  mapping_started_ = false;
}

/**
 * Loads all required and optional parameters for the use of mapping_handler. Parameters can be configured
 * in the task_supervisor's yaml file, under the 'mapping_handler' section
 *
 * Parameters are loaded on startup of task_supervisor, not on task execution
 */
bool MultiSessionMappingHandler::loadParams()
{
  ros_utils::ParamLoader param_loader(nh_handler_);

  param_loader.get_optional("loop_rate", p_loop_rate_, 5.0);
  param_loader.get_optional("save_timeout", p_save_timeout_, 5.0);
  param_loader.get_optional("merged_map_topic", p_merged_map_topic_, std::string("/map"));
  param_loader.get_required("multi_session_mapping_launch_package", p_multi_session_mapping_launch_package_);
  param_loader.get_required("mapping_launch_file", p_mapping_launch_file_);
  param_loader.get_required("map_expander_launch_file", p_map_expander_launch_file_);
  param_loader.get_required("dynamic_mapping_nav_launch_file", p_dyn_mapping_nav_launch_file_);
  param_loader.get_required("previous_map_dir", p_map_dir_);
  //param_loader.get_required("mapping_launch_file", p_mapping_launch_file_);
  //param_loader.get_required("mapping_launch_nodes", p_mapping_launch_nodes_);
  // param_loader.get_required("map_expander_launch_nodes", p_map_expander_launch_nodes_);
  // param_loader.get_required("dyn_move_base_launch_nodes", p_dyn_move_base_launch_nodes_);

  return param_loader.params_valid();
}

/**
 *	Setup method called once task_supervisor starts and loads all plugins (task handlers)
 */
bool MultiSessionMappingHandler::setupHandler()
{
  if (!loadParams())
    return false;
  else
  {
    // services for web
    start_full_srv_ = nh_handler_.advertiseService("start_full", &MultiSessionMappingHandler::onStartHandlerCall, this);
    save_srv_ = nh_handler_.advertiseService("save_map", &MultiSessionMappingHandler::onSaveServiceCall, this);
    stop_srv_ = nh_handler_.advertiseService("stop", &MultiSessionMappingHandler::onStopCall, this);
    status_srv_ = nh_handler_.advertiseService("status", &MultiSessionMappingHandler::onStatus, this);

    started_via_service_ = false;

    return true;
  }
}

bool MultiSessionMappingHandler::healthCheck()
{
  static int failcount = 0;
  if (mapping_launch_id_ == 0 || map_expander_launch_id_ == 0 || dyn_mapping_nav_launch_id_ == 0)
  {
    failcount = 0;
    return true;
  }
  bool healthy = launchStatus(mapping_launch_id_) && launchStatus(map_expander_launch_id_) && launchStatus(dyn_mapping_nav_launch_id_);
  if (!healthy && mapping_launch_id_ && map_expander_launch_id_ && dyn_mapping_nav_launch_id_)
  {
    // it is possible for launchStatus to return false right after the nodes are launched
    // so failure assessment must give it time to stabilise
    // --> only declare unhealth after several seconds of consistent report
    failcount += 1;
    ROS_INFO("[%s] fail count %d", name_.c_str(), failcount);
    if (failcount >= 30*p_watchdog_rate_)
    {
      // report bad health
      ROS_INFO("[%s] one or more multi-session mapping nodes have failed %d, %5.2f", 
        name_.c_str(), failcount, 2*p_watchdog_rate_);

      // tear down task
      cancelTask();

      // reset flags
      failcount = 0;
    }
  }
  else
    failcount = 0;
  return healthy;
}
}  // namespace task_supervisor
