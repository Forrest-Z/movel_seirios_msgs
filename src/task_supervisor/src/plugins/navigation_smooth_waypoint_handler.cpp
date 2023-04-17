#include <task_supervisor/plugins/base/navigation_handler_base.h>
#include <movel_common_libs/json.hpp>
#include <pluginlib/class_list_macros.h>
#include <movel_fms_utils/path_dist_utils.hpp>


using json = nlohmann::json;

namespace task_supervisor
{

class NavigationSmoothWaypointHandler : public NavigationHandlerBase
{
  /**
   * @brief Method called by task_supervisor when a navigation task is received
   * @param task Relevant task passed to handler by task supervisor
   * @param error_message Error message returned by this handler if execution fails
   * @return ReturnCode which indicates failure, cancellation or success
   */
  virtual ReturnCode runTask(movel_seirios_msgs::Task& task, std::string& error_message);
};


PLUGINLIB_EXPORT_CLASS(task_supervisor::NavigationSmoothWaypointHandler, task_supervisor::TaskHandler);


ReturnCode NavigationSmoothWaypointHandler::runTask(movel_seirios_msgs::Task& task, std::string& error_message)
{
  task_active_ = true;
  task_parsed_ = false;
  isHealthy_ = true;
  start_ = ros::Time::now();

  // single map navigation is never going to transit point to switch maps
  is_navigating_to_transit_point_ = false;

  ros::ServiceClient clear_costmap_serv_ = nh_handler_.serviceClient<std_srvs::Empty>("/move_base/clear_costmaps");

  if(ros::service::waitForService("/move_base/clear_costmaps",ros::Duration(2.0))){
    std_srvs::Empty clear_costmap_msg;
    clear_costmap_serv_.call(clear_costmap_msg);
  }
  else{
    ROS_WARN("[%s] Could not contact clear_costmap service", name_.c_str());
  }
  
  if (start_ActionClient()) {
    ROS_INFO("[%s] Task payload %s", name_.c_str(), task.payload.c_str());
    json payload = json::parse(task.payload);
    if (payload.find("path") != payload.end()) {
      // input waypoints
      std::vector<geometry_msgs::Pose> waypoints{};
      for (auto& elem : payload["path"]) {
        geometry_msgs::Pose waypoint;
        waypoint.position.x = elem["position"]["x"].get<double>();
        waypoint.position.y = elem["position"]["y"].get<double>();
        waypoint.position.z = elem["position"]["z"].get<double>();
        waypoint.orientation.x = elem["orientation"]["x"].get<double>();
        waypoint.orientation.y = elem["orientation"]["y"].get<double>();
        waypoint.orientation.z = elem["orientation"]["z"].get<double>();
        waypoint.orientation.w = elem["orientation"]["w"].get<double>();
        waypoints.push_back(waypoint);
      }
      // start at nearest point
      int start_at_idx = 0;
      bool start_at_nearest_point = false;
      if (payload.find("start_at_nearest_point") != payload.end()) {
        start_at_nearest_point = payload["start_at_nearest_point"].get<bool>();
      }
      if (start_at_nearest_point) {
        ros::Duration(0.2).sleep();   // give /pose time to update
        start_at_idx = movel_fms_utils::getNearestWaypointIdx(waypoints, robot_pose_, movel_fms_utils::DistMetric::EUCLIDEAN);
        ROS_WARN("[%s] NEAREST IDX %d", name_.c_str(), start_at_idx);
      }
      // navigation
      runTaskChooseNav(waypoints, start_at_idx);
    }
    else {
      setMessage("Malformed payload");
      error_message = message_;
      setTaskResult(false);
    }
  }
  else {
    setMessage("Unable to talk to Move Base action server on " + p_navigation_server_);
    error_message = message_;
    setTaskResult(false);
  }
  return code_;
}

}  // namespace task_supervisor