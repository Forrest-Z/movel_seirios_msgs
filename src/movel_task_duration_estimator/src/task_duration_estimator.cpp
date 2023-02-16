#include "task_duration_estimator.h"

TaskDurationEstimator::TaskDurationEstimator(ros::NodeHandle& nh, ros::NodeHandle& priv_nh) : nh_(nh), priv_nh_(priv_nh)
{
  duration_estimator_server = nh_.advertiseService("task_duration", &TaskDurationEstimator::durationCb, this);
  planner_request_client = nh_.serviceClient<nav_msgs::GetPlan>("/make_plan");
}

bool TaskDurationEstimator::durationCb(movel_seirios_msgs::GetTaskDurationEstimate::Request& req,
                                       movel_seirios_msgs::GetTaskDurationEstimate::Response& res)
{
  ROS_INFO("Received request");
  std::deque<geometry_msgs::PoseStamped> target_poses;
  // Parse JSON Payload to coordinates
  // for (auto tasks = std::begin(req.tasks); tasks != std::end(req.tasks); ++tasks)
  for (auto tasks : req.tasks.tasks){
    json payload = json::parse(tasks.payload);
    if (payload.find("path") != payload.end()) {
      // input waypoints
      for (auto& elem : payload["path"]) {
        geometry_msgs::Pose waypoint;
        waypoint.position.x = elem["position"]["x"].get<double>();
        waypoint.position.y = elem["position"]["y"].get<double>();
        waypoint.position.z = elem["position"]["z"].get<double>();
        waypoint.orientation.x = elem["orientation"]["x"].get<double>();
        waypoint.orientation.y = elem["orientation"]["y"].get<double>();
        waypoint.orientation.z = elem["orientation"]["z"].get<double>();
        waypoint.orientation.w = elem["orientation"]["w"].get<double>();
        std_msgs::Header dummy_header;
        dummy_header.frame_id = "map";
        geometry_msgs::PoseStamped temp_pose_stamped;
        temp_pose_stamped.pose = waypoint;
        temp_pose_stamped.header = dummy_header;
        target_poses.push_back(temp_pose_stamped);
      }
    }
  }

  int task_sz = sizeof(req.tasks.tasks) / sizeof(req.tasks.tasks[0]);

  geometry_msgs::PoseStamped current_pos;
  current_pos.pose = req.current_pose;
  current_pos.header.frame_id = "map";

  geometry_msgs::PoseStamped target_pos = target_poses.front();

  if (task_sz > 1){
    // Get global plan to first waypoint
    nav_msgs::Path global_plan = get_global_plan(current_pos, target_pos);
    float distance = calculateDist(global_plan);

    for (auto tasks : req.tasks.tasks){
      if (tasks.type == 3){
        
      }
    }
  }
  return true;
}

nav_msgs::Path TaskDurationEstimator::get_global_plan(geometry_msgs::PoseStamped start, geometry_msgs::PoseStamped goal){
  nav_msgs::GetPlan srv;
  nav_msgs::Path path_;
  srv.request.start = start;
  srv.request.goal = goal;

  ROS_INFO("Sending request");
  if (planner_request_client.call(srv)){
    path_ = srv.response.plan;
    ROS_INFO("Received response");
    return path_;
  }
  else{
    ROS_ERROR("Failed to call service");
    return path_;
  }
}

float TaskDurationEstimator::calculateDist(const nav_msgs::Path& path)
{
  double total_distance = 0.0;
  geometry_msgs::Point prev_point;
  bool first_point = true;
  
  for (const auto& point : path.poses){
      geometry_msgs::Point current_point = point.pose.position;
      if (!first_point){
          double distance = std::sqrt(std::pow(current_point.x - prev_point.x, 2) + 
                                      std::pow(current_point.y - prev_point.y, 2));
          total_distance += distance;
      }
      else{
          first_point = false;
      }
      prev_point = current_point;
  }
  ROS_INFO("Total distance of path: %f meters", total_distance);
  return total_distance;
}
