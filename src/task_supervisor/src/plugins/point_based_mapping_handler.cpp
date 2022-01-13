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
	else
		return true;
}

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

}