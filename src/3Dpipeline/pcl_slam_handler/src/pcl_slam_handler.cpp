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

// Not implemented due to the expensive computational load required.
/*
bool PCLSlamHandler::onAsyncSave(movel_seirios_msgs::StringTrigger::Request& req,
                                 movel_seirios_msgs::StringTrigger::Response& res)
{
  res.success = saveMap(req.input);
  return true;
}
*/

bool PCLSlamHandler::saveMap(std::string map_name)
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

  // Set path to save file
  std::string launch_args = " map_topic:=" + p_map_topic_;
  if (!map_name.empty())
    launch_args = launch_args + " file_path:=" + map_name;
    launch_args = launch_args + " pcd_path:=" + map_name + ".pcd";
  // Convert PCD to 2D (3D to 2D) and save
  unsigned int conversion_id = startLaunch(p_3Dto2D_package_, p_3Dto2D_launch_, launch_args);
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
      stopLaunch(conversion_id);
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
bool PCLSlamHandler::runMapping()
{
  ROS_INFO("[%s] Starting PCL Slam package: %s, launch file: %s", name_.c_str(), p_pcl_slam_launch_package_.c_str(),
           p_pcl_slam_launch_.c_str());

  // Run mapping asynchronously
  pcl_slam_launch_id_ = startLaunch(p_pcl_slam_launch_package_, p_pcl_slam_launch_, "");
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
 * Once mapping is complete, save_map service is shut down and the tas Bah
k is signalled as complete
 */
task_supervisor::ReturnCode PCLSlamHandler::runTask(movel_seirios_msgs::Task& task, std::string& error_message)
{
  task_active_ = true;
  task_parsed_ = false;
  start_ = ros::Time::now();

  ros::ServiceServer serv_save_ =
      nh_handler_.advertiseService("save_pcl_map", &PCLSlamHandler::onSaveServiceCall, this);
  // ros::ServiceServer serv_save_async_ =
  // nh_handler_.advertiseService("save_map_async", &PCLSlamHandler::onAsyncSave, this);
  save_map_client_ = nh_handler_.serviceClient<hdl_graph_slam::SaveMap>("/hdl_graph_slam/save_map");

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

  param_loader.get_required("pcl_slam_launch_package", p_pcl_slam_launch_package_);
  param_loader.get_required("pcl_slam_launch", p_pcl_slam_launch_);
  param_loader.get_required("pcl_map_saver_package", p_map_saver_package_);
  param_loader.get_required("pcl_map_saver_launch", p_map_saver_launch_);
  param_loader.get_required("three_to_two_package", p_3Dto2D_package_);
  param_loader.get_required("three_to_two_launch", p_3Dto2D_launch_);

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
    return true;
}
}  // namespace task_supervisor
