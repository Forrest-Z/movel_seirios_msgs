#include <pluginlib/class_list_macros.h>
#include <task_supervisor/plugins/mapping_handler.h>
#include <ros_utils/ros_utils.h>  //For loadParams function contents

PLUGINLIB_EXPORT_CLASS(task_supervisor::MappingHandler, task_supervisor::TaskHandler);

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
bool MappingHandler::onSaveServiceCall(movel_seirios_msgs::StringTrigger::Request& req,
                                       movel_seirios_msgs::StringTrigger::Response& res)
{
  res.success = saveMap(req.input);
  saved_ = true;
  return true;
}

bool MappingHandler::onAsyncSave(movel_seirios_msgs::StringTrigger::Request& req,
                                 movel_seirios_msgs::StringTrigger::Response& res)
{
  res.success = saveMap(req.input);
  return true;
}

bool MappingHandler::onStatus(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res)
{
  res.success = launchStatus(mapping_launch_id_);
  return true;
}

bool MappingHandler::saveMap(std::string map_name)
{
  // Set path to save file
  std::string launch_args = " map_topic:=" + p_map_topic_;
  if (!map_name.empty())
    launch_args = launch_args + " file_path:=" + map_name;

  // Call map saving through launch manager service
  ROS_INFO("[%s] Saving map %s", name_.c_str(), map_name.size() != 0 ? ("to" + map_name).c_str() : "");
  unsigned int map_saver_id = startLaunch("task_supervisor", "map_saver.launch", launch_args);

  // Check if startLaunch succeeded
  if (!map_saver_id)
  {
    ROS_ERROR("[%s] Failed to start map saver", name_.c_str());
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

  ROS_WARN("[%s] Timeout occurred, save failed", name_.c_str());
  stopLaunch(map_saver_id);
  return false;
}

/**
 * Method called by runTask() which starts up the mapping node. Mapping package and launch file is defined in the
 * task_supervisor
 * yaml under 'mapping_launch_package' and 'mapping_launch_file'. Launch file should only launch the mapping node, such
 * as gmapping,
 * and no other nodes eg move_base.
 *
 * Once mapping is started by task_supervisor, mapping_handler eventually blocks in this method and waits for
 * cancellation
 * or saving to be done via save_map service. Once save_map service is called, this method will return
 * and the mapping node will be shut down.
 */
bool MappingHandler::runMapping()
{
  ROS_INFO("[%s] Starting mapping package: %s, launch file: %s", name_.c_str(), p_mapping_launch_package_.c_str(),
           p_mapping_launch_file_.c_str());

  // Run mapping asynchronously
  mapping_launch_id_ = startLaunch(p_mapping_launch_package_, p_mapping_launch_file_, "");
  if (!mapping_launch_id_)
  {
    ROS_ERROR("[%s] Failed to launch mapping launch file", name_.c_str());
    return false;
  }

  // Loop until save callback is called
  ros::Rate r(p_loop_rate_);
  while (!saved_)
  {
    // Check if cancellation has been called
    if (!isTaskActive())
    {
      ROS_INFO("[%s] Waiting for mapping thread to exit", name_.c_str());
      stopLaunch(mapping_launch_id_, p_mapping_launch_nodes_);
      while (launchExists(mapping_launch_id_))
        ;
      return false;
    }

    r.sleep();
  }

  ROS_INFO("[%s] Stopping mapping", name_.c_str());
  stopLaunch(mapping_launch_id_, p_mapping_launch_nodes_);
  while (launchExists(mapping_launch_id_))
    ;
  mapping_launch_id_ = 0;
  ROS_INFO("[%s] Mapping stopped", name_.c_str());

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
ReturnCode MappingHandler::runTask(movel_seirios_msgs::Task& task, std::string& error_message)
{
  task_active_ = true;
  task_parsed_ = false;
  start_ = ros::Time::now();

  ros::ServiceServer serv_status_ = nh_handler_.advertiseService("status", &MappingHandler::onStatus, this);
  ros::ServiceServer serv_save_ = nh_handler_.advertiseService("save_map", &MappingHandler::onSaveServiceCall, this);
  ros::ServiceServer serv_save_async_ =
      nh_handler_.advertiseService("save_map_async", &MappingHandler::onAsyncSave, this);

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
bool MappingHandler::loadParams()
{
  ros_utils::ParamLoader param_loader(nh_handler_);

  param_loader.get_optional("loop_rate", p_loop_rate_, 5.0);
  param_loader.get_optional("save_timeout", p_save_timeout_, 5.0);
  param_loader.get_optional("map_topic", p_map_topic_, std::string("/map"));

  param_loader.get_required("mapping_launch_package", p_mapping_launch_package_);
  param_loader.get_required("mapping_launch_file", p_mapping_launch_file_);
  param_loader.get_required("mapping_launch_nodes", p_mapping_launch_nodes_);

  return param_loader.params_valid();
}

/**
 *	Setup method called once task_supervisor starts and loads all plugins (task handlers)
 */
bool MappingHandler::setupHandler()
{
  if (!loadParams())
    return false;
  else
    return true;
}
}  // namespace task_supervisor