#include <pluginlib/class_list_macros.h>
#include <task_supervisor/plugins/point_based_mapping_handler.h>
#include <ros_utils/ros_utils.h> 

PLUGINLIB_EXPORT_CLASS(task_supervisor::PointBasedMappingHandler, task_supervisor::TaskHandler);

namespace task_supervisor {

bool PointBasedMappingHandler::loadParams()
{
  ros_utils::ParamLoader param_loader(nh_handler_);

  param_loader.get_optional("loop_rate", p_loop_rate_, 5.0);
  param_loader.get_optional("save_timeout", p_save_timeout_, 5.0);
  param_loader.get_optional("map_topic", p_map_topic_, std::string("/map"));
  param_loader.get_required("mapping_launch_package", p_mapping_launch_package_);
  param_loader.get_required("mapping_launch_file", p_mapping_launch_file_);
  param_loader.get_required("mapping_launch_nodes", p_mapping_launch_nodes_);
  param_loader.get_required("previous_map_dir", p_map_dir_);

  return param_loader.params_valid();
}

bool PointBasedMappingHandler::setupHandler()
{
  if (!loadParams()) {
    return false;
  }
	else {
    serv_status_ = nh_handler_.advertiseService("point_based_mapping_status", &PointBasedMappingHandler::onStatus, this);
    serv_save_ = nh_handler_.advertiseService("point_based_mapping_save_map", &PointBasedMappingHandler::onSaveServiceCall, this);
    serv_save_async_ = nh_handler_.advertiseService("point_based_mapping_save_map_async", &PointBasedMappingHandler::onAsyncSave, this);
    serv_cancel_ = nh_handler_.advertiseService("point_based_mapping_stop", &PointBasedMappingHandler::onStopCall, this);
    health_check_pub_ = nh_handler_.advertise<movel_seirios_msgs::Reports>("/task_supervisor/health_report", 1);
    return true;
  }
}

/* 
 * Start a different move_base to run point based mapping
 */
bool PointBasedMappingHandler::runPointBasedMapping() {
  ROS_INFO("[%s] Starting point based mapping package: %s, launch file: %s", name_.c_str(), p_mapping_launch_package_.c_str(),p_mapping_launch_file_.c_str());
  
  //pb_mapping_launch_id_ = startLaunch(p_mapping_launch_package_, p_mapping_launch_file_, "");
  pb_mapping_launch_id_ = startLaunch("movel", "point_based_mapping.launch", "");
  if (!pb_mapping_launch_id_)
  {
    ROS_ERROR("[%s] Failed to launch point based mapping launch file", name_.c_str());
    return false;
  }
  else
  {
    mapping_started_ = true;
    return true;
  }  
}

/*
 * Call map saver to save the map
 */
bool PointBasedMappingHandler::saveMap(std::string map_name)
{
  // Set path to save file
  std::string launch_args = " map_topic:=" + p_map_topic_;
  
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

bool PointBasedMappingHandler::onSaveServiceCall(movel_seirios_msgs::StringTrigger::Request& req, movel_seirios_msgs::StringTrigger::Response& res) 
{
  res.success = saveMap(req.input);
  saved_ = true;
  return true;
}

bool PointBasedMappingHandler::onAsyncSave(movel_seirios_msgs::StringTrigger::Request& req, movel_seirios_msgs::StringTrigger::Response& res)
{
  res.success = saveMap(req.input);
  return true;
}

/* When stop is called, map will be saved to a temporary file path. Then stop the launches. */
bool PointBasedMappingHandler::onStopCall(movel_seirios_msgs::StringTrigger::Request& req, movel_seirios_msgs::StringTrigger::Response& res) {
  ROS_WARN("[%s] Stopping point based mapping", name_.c_str());
  ROS_INFO("[%s] Saving the map", name_.c_str());

  std::string temp_file_path = "/home/movel/.config/movel/maps/unfinished_pb_map";
  bool map_saved = saveMap(temp_file_path);
  if(!map_saved) {
    saved_ = false;
    ROS_ERROR("[%s] Problem saving the map, stopping launch...", name_.c_str());
  }
  stopLaunch(pb_mapping_launch_id_, p_mapping_launch_nodes_);
  pb_mapping_launch_id_ = 0;
  mapping_started_ = false;
  res.success = true;
  return true;
}

bool PointBasedMappingHandler::onStatus(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res)
{
  res.success = launchStatus(pb_mapping_launch_id_);
  return true;
}




ReturnCode PointBasedMappingHandler::runTask(movel_seirios_msgs::Task& task, std::string& error_message) 
{
  task_active_ = true;
  task_parsed_ = false;
  start_ = ros::Time::now();

	bool mapping_done = runPointBasedMapping();
	setTaskResult(mapping_done);
	return code_;
}

bool PointBasedMappingHandler::healthCheck()
{
  static int failcount = 0;
  if (pb_mapping_launch_id_ == 0)
  {
    failcount = 0;
    return true;
  }
  bool healthy = launchStatus(pb_mapping_launch_id_);
  if (!healthy && pb_mapping_launch_id_)
  {
    // it is possible for launchStatus to return false right after the nodes are launched
    // so failure assessment must give it time to stabilise
    // --> only declare unhealth after several seconds of consistent report
    failcount += 1;
    ROS_INFO("[%s] fail count %d", name_.c_str(), failcount);
    if (failcount >= 30*p_watchdog_rate_)
    {
      // report bad health
      ROS_INFO("[%s] one or more point based mapping nodes have failed %d, %5.2f", 
        name_.c_str(), failcount, 2*p_watchdog_rate_);
      movel_seirios_msgs::Reports report;
      report.header.stamp = ros::Time::now();
      report.handler = "point_based_mapping_handler";
      report.task_type = task_type_;
      report.healthy = false;
      report.message = "some point based mapping nodes are not running";
      health_check_pub_.publish(report);

      // tear down task
      cancelTask();
      // stopLaunch(mapping_launch_id_);
      // saved_ = true;
      // setTaskResult(false);

      // reset flags
      failcount = 0;
      // mapping_launch_id_ = 0;
    }
  }
  else
    failcount = 0;
  return healthy;
}

}