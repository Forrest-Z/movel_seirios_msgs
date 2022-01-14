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
  if (!loadParams())
    return false;
	else {
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

ReturnCode PointBasedMappingHandler::runTask(movel_seirios_msgs::Task& task, std::string& error_message) {
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