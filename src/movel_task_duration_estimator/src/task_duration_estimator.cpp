#include "movel_task_duration_estimator/task_duration_estimator.h"

TaskDurationEstimator::TaskDurationEstimator(ros::NodeHandle& nh, ros::NodeHandle& priv_nh) : nh_(nh), priv_nh_(priv_nh)
{
  // Services
  planner_request_client = nh_.serviceClient<nav_msgs::GetPlan>("/planner_utils/make_sync_plan");

  // Subscribers
  robot_pose_sub_ = nh_.subscribe("/pose", 1, &TaskDurationEstimator::robotPoseCB, this);
  ts_goal_sub_ = nh_.subscribe("/task_supervisor/goal", 1, &TaskDurationEstimator::tsGoalCB, this);
  current_plan_sub_ = nh_.subscribe("/move_base/GlobalPlanner/plan", 2, &TaskDurationEstimator::currentPlanCB, this);
  ts_result_sub_ = nh_.subscribe("/task_supervisor/result", 1, &TaskDurationEstimator::tsResultCB, this);
  move_base_result_sub_ = nh_.subscribe("/move_base/result", 1, &TaskDurationEstimator::moveBaseResultCB, this);
  ts_cancel_sub_ = nh_.subscribe("/task_supervisor/cancel", 1, &TaskDurationEstimator::tsCancelCB, this);

  // Publishers
  task_duration_pub_ = nh_.advertise<movel_seirios_msgs::TaskDuration>("/task_duration", 1);
  task_duration_only_pub_ = nh_.advertise<std_msgs::Int64>("/task_duration_", 1);

  // Timers
  publish_task_duration_timer_ = nh_.createTimer(ros::Duration(1.0), &TaskDurationEstimator::publishTaskDuration, this);

  //Dynamic reconfigure
  dynamic_reconf_server_.reset(new dynamic_reconfigure::Server<movel_task_duration_estimator::DurationEstimatorConfig>(priv_nh));
  dynamic_reconfigure_callback_ = boost::bind(&TaskDurationEstimator::reconfCB, this, _1, _2);
  dynamic_reconf_server_->setCallback(dynamic_reconfigure_callback_);
}

void TaskDurationEstimator::robotPoseCB(const geometry_msgs::Pose::ConstPtr& msg)
{
  robot_pose_ = *msg;
}

void TaskDurationEstimator::currentPlanCB(const nav_msgs::Path::ConstPtr& msg)
{
  // ROS_INFO("Received plan");
  is_plan_updated_ = true;
  current_plan_ = *msg;
  double current_dist = calculateDist(current_plan_);
  if (!est_times_.empty()){
    // Update the first element est_times_
    est_times_.front() = current_dist/current_lin_vel_;
  }
  // ROS_INFO("Estimated times:");
  // for (auto elem:est_times_) ROS_INFO("%f", elem);
  float accumulated_estimate_weight = 0.7;
  float current_estimate_weight = 0.3;
  est_time_ = (std::accumulate(est_times_.begin(), est_times_.end(), 0.0) * multiplication_factor)*accumulated_estimate_weight + est_time_*current_estimate_weight;
  is_plan_updated_ = false;
}

void TaskDurationEstimator::moveBaseResultCB(const move_base_msgs::MoveBaseActionResult::ConstPtr& msg)
{
  if (msg->status.status == GoalStatus::SUCCEEDED || (msg->status.status == GoalStatus::PREEMPTED && msg->status.text != "")) {
    if(!est_times_.empty())  est_times_.pop_front();
    if(!target_poses_.empty())  target_poses_.pop_front();
    if(!lin_vels_.empty())   lin_vels_.pop_front();
  }
  else if (msg->status.status == GoalStatus::ABORTED){
    est_times_.clear();
    target_poses_.clear();
    lin_vels_.clear();
    est_time_ = 0;
    is_navigating_ = false;
  }
}

void TaskDurationEstimator::tsCancelCB(const actionlib_msgs::GoalID::ConstPtr& msg)
{
  est_times_.clear();
  target_poses_.clear();
  lin_vels_.clear();
  est_time_ = 0;
  is_navigating_ = false;
}

void TaskDurationEstimator::publishTaskDuration(const ros::TimerEvent& event)
{
  movel_seirios_msgs::TaskDuration task_duration_msg;
  std_msgs::Int64 task_duration_int_msg;
  task_duration_msg.task_id = curr_task_id_;

  if(!is_navigating_){
    task_duration_int_msg.data = 0;
    return;
  }

  if (!is_plan_updated_){
    if (est_time_ > 2) est_time_ -= 1; // So that the est_time stays at 1, if set to be > 1 it will be 0, idk why
  }

  task_duration_int_msg.data = est_time_;
  task_duration_only_pub_.publish(task_duration_int_msg);
  
}

void TaskDurationEstimator::tsResultCB(const movel_seirios_msgs::RunTaskListActionResult::ConstPtr& msg)
{
  if (msg->status.status == GoalStatus::SUCCEEDED){
    is_navigating_ = false;
  }
}

void TaskDurationEstimator::tsGoalCB(const movel_seirios_msgs::RunTaskListActionGoal::ConstPtr& msg){
  // ROS_INFO("Received request");
  std::vector<geometry_msgs::Pose> waypoints{};
  geometry_msgs::PoseStamped current_pos;
  current_pos.pose = robot_pose_;
  current_pos.header.frame_id = "map";
  double distance = 0;
  est_time_ = 0;
  current_lin_vel_ = 0;

  // Empty est_times_ and est_time_
  est_times_.clear();
  est_time_ = 0;
  
  bool start_at_nearest_point = false;
  int start_at_idx = 0;
  // Parse JSON Payload to coordinates
  for (auto& tasks: msg->goal.task_list.tasks){
    
    ROS_INFO("Parsing task with type: %d", tasks.type);
    // Initialize empty payload
    json payload = json::object();
    try{
      payload = json::parse(tasks.payload);
    }
    catch (json::parse_error& e){
      ROS_WARN("Parse error: %s", e.what());
      return;
    }

    if (payload.find("path") != payload.end()) {
      // Input waypoints
      if (payload.find("start_at_nearest_point") != payload.end()) {
        start_at_nearest_point = payload["start_at_nearest_point"].get<bool>();
        if (start_at_nearest_point){
          start_at_idx = movel_fms_utils::getNearestWaypointIdx(waypoints, robot_pose_, movel_fms_utils::DistMetric::EUCLIDEAN);
        }
      }
      for (auto& elem : payload["path"]){
        // Skip the first start_at_idx points
        if (start_at_idx != 0){
          start_at_idx--;
          ROS_INFO("Skipping %d waypoints", start_at_idx);
          continue;
        }
        geometry_msgs::Pose waypoint;
        waypoint.position.x = elem["position"]["x"].get<double>();
        waypoint.position.y = elem["position"]["y"].get<double>();
        waypoint.position.z = elem["position"]["z"].get<double>();
        waypoint.orientation.x = elem["orientation"]["x"].get<double>();
        waypoint.orientation.y = elem["orientation"]["y"].get<double>();
        waypoint.orientation.z = elem["orientation"]["z"].get<double>();
        waypoint.orientation.w = elem["orientation"]["w"].get<double>();

        current_lin_vel_ = elem["velocity"]["linear_velocity"].get<double>();

        std_msgs::Header dummy_header;
        dummy_header.frame_id = "map";
        geometry_msgs::PoseStamped temp_pose_stamped; // Target pose
        temp_pose_stamped.pose = waypoint;
        temp_pose_stamped.header = dummy_header;

        nav_msgs::Path global_plan;
        if (tasks.type == 3){
          global_plan = getGlobalPlan(current_pos, temp_pose_stamped);
          distance = calculateDist(global_plan);
        }
        else if (tasks.type == 6){
          distance = calculateEuclidianDist(current_pos, temp_pose_stamped) * trail_multiplication_factor;
        }
        current_pos = temp_pose_stamped;
        est_times_.push_back(distance/current_lin_vel_);
        // ROS_INFO("Estimated time pushed : %f", est_times_.back());
        is_navigating_ = true;
      }
      if (!est_times_.empty()){
        est_time_ = std::accumulate(est_times_.begin(), est_times_.end(), 0.0) * multiplication_factor;
        // ROS_INFO("Estimated time: %f", est_time_);
        std_msgs::Int64 task_duration_int_msg;
        task_duration_int_msg.data = est_time_;
        task_duration_only_pub_.publish(task_duration_int_msg);
      }
    }
    else{
      // Payload is not a path, exit
      ROS_WARN("Payload is not a path");
      return;
    }
  }
}

nav_msgs::Path TaskDurationEstimator::getGlobalPlan(geometry_msgs::PoseStamped start, geometry_msgs::PoseStamped goal){
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
    ROS_ERROR("Failed to call make_plan service, retrying");
    for (int i=0; i<3; i++){
      if (planner_request_client.call(srv)){
        path_ = srv.response.plan;
        // ROS_INFO("Received response");
        return path_;
      }
      ROS_WARN("Attempt %d failed", i);
    }
    return path_;
  }
}

void TaskDurationEstimator::reconfCB(movel_task_duration_estimator::DurationEstimatorConfig &config, uint32_t level){
  multiplication_factor = config.multiplication_factor;
  trail_multiplication_factor = config.trail_multiplication_factor;
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
