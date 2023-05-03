#include "movel_task_duration_estimator/task_duration_estimator.h"

TaskDurationEstimator::TaskDurationEstimator(ros::NodeHandle& nh, ros::NodeHandle& priv_nh) : nh_(nh), priv_nh_(priv_nh)
{
  duration_estimator_server = nh_.advertiseService("task_duration", &TaskDurationEstimator::durationCb, this);
  planner_request_client = nh_.serviceClient<nav_msgs::GetPlan>("/move_base/GlobalPlanner/make_plan");
  robot_pose_sub_ = nh_.subscribe("/pose", 1, &TaskDurationEstimator::robotPoseCB, this);

  //Dynamic reconfigure
  dynamic_reconf_server_.reset(new dynamic_reconfigure::Server<movel_task_duration_estimator::DurationEstimatorConfig>(priv_nh));
  dynamic_reconfigure_callback_ = boost::bind(&TaskDurationEstimator::reconfCB, this, _1, _2);
  dynamic_reconf_server_->setCallback(dynamic_reconfigure_callback_);
}

void TaskDurationEstimator::robotPoseCB(const geometry_msgs::Pose::ConstPtr& msg)
{
  robot_pose_ = *msg;
}

bool TaskDurationEstimator::durationCb(movel_seirios_msgs::GetTaskDurationEstimate::Request& req,
                                       movel_seirios_msgs::GetTaskDurationEstimate::Response& res)
{
  // ROS_INFO("Received request");
  std::deque<geometry_msgs::PoseStamped> target_poses;
  std::vector<geometry_msgs::Pose> waypoints{};
  bool start_at_nearest_point = false;
  int start_at_idx = 0;
  // Parse JSON Payload to coordinates
  // for (auto tasks = std::begin(req.tasks); tasks != std::end(req.tasks); ++tasks)
  for (auto tasks : req.tasks.tasks){
    json payload = json::parse(tasks.payload);
    if (payload.find("path") != payload.end()) {
      // Input waypoints
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
        waypoints.push_back(waypoint);
        // ROS_INFO("Parsed a waypoint : [%lf , %lf]", waypoint.position.x, waypoint.position.y);
      }
    }

    if (payload.find("start_at_nearest_point") != payload.end()) {
      start_at_nearest_point = payload["start_at_nearest_point"].get<bool>();
    }
    
    if (start_at_nearest_point) {
      ros::Duration(0.2).sleep();   // give /pose time to update
      start_at_idx = movel_fms_utils::getNearestWaypointIdx(waypoints, robot_pose_, movel_fms_utils::DistMetric::EUCLIDEAN);
      // Delete poses before start_at_idx
      if (start_at_idx != 0){
        for (int i = 0; i < start_at_idx; i++){
          target_poses.pop_front();
        }
      }

    }
  }

  int task_sz = std::end(req.tasks.tasks) - std::begin(req.tasks.tasks);

  geometry_msgs::PoseStamped current_pos;
  current_pos.pose = req.current_pose;
  current_pos.header.frame_id = "map";

  geometry_msgs::PoseStamped target_pos = target_poses.front();
  // target_poses.pop_front();
  // ROS_INFO("Task size : %d", task_sz);

  float distance = 0, est_time = 0;

  if (task_sz >= 1){
    // ROS_INFO("Getting first global plan");
    // Get global plan to first waypoint
    // nav_msgs::Path global_plan = get_global_plan(current_pos, target_pos);
    // float distance = calculateDist(global_plan);
    // target_poses.pop_front();

    nav_msgs::Path global_plan;
    
    for (auto tasks : req.tasks.tasks){
      for (auto target_point:target_poses){
        target_pos = target_point;
        if (tasks.type == 3){
          global_plan = get_global_plan(current_pos, target_pos);
          distance += calculateDist(global_plan);
        }
        else if (tasks.type == 6 || tasks.type == 4){
          distance += calculateEuclidianDist(current_pos, target_pos);
        }
        current_pos = target_pos;
        target_poses.pop_front();
      }
      est_time += distance/tasks.linear_velocity;
    }
    ROS_INFO("Total distance : %f", distance);
    // ROS_INFO("Estimated time original : [%f] seconds", est_time);
    ROS_INFO("Estimated time : [%f] seconds", est_time * multiplication_factor);
  }
  res.estimate_secs = est_time * multiplication_factor;
  
  return true;
}

nav_msgs::Path TaskDurationEstimator::get_global_plan(geometry_msgs::PoseStamped start, geometry_msgs::PoseStamped goal){
  nav_msgs::GetPlan srv;
  nav_msgs::Path path_;
  srv.request.start = start;
  srv.request.goal = goal;

  // ROS_INFO("Sending request");
  if (planner_request_client.call(srv)){
    path_ = srv.response.plan;
    // ROS_INFO("Received response");
    return path_;
  }
  else{
    ROS_ERROR("Failed to call make_plan service");
    return path_;
  }
}

void TaskDurationEstimator::reconfCB(movel_task_duration_estimator::DurationEstimatorConfig &config, uint32_t level){
  multiplication_factor = config.multiplication_factor;
}

float TaskDurationEstimator::calculateDist(const nav_msgs::Path& path){
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
  // ROS_INFO("Total distance of path: %f meters", total_distance);
  return total_distance;
}

float TaskDurationEstimator::calculateEuclidianDist(geometry_msgs::PoseStamped start_, geometry_msgs::PoseStamped goal_){
  geometry_msgs::Point start_point = start_.pose.position;
  geometry_msgs::Point goal_point = goal_.pose.position;
  float dist_ = std::sqrt(std::pow(start_point.x - goal_point.x, 2) + std::pow(start_point.y - goal_point.y, 2));
  // ROS_INFO("Total distance of path: %f meters", dist_);
  return dist_;
}
