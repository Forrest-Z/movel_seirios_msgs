#include <task_supervisor/plugins/base/multi_point_navigation_handler_base.h>

#include <algorithm>

#include <ros_utils/ros_utils.h>
#include <movel_common_libs/json.hpp>

using json = nlohmann::json;

namespace task_supervisor
{
MultiPointNavigationHandlerBase::MultiPointNavigationHandlerBase()
  : task_cancelled_(false)
  , is_healthy_(true)
  , tf_listener_(tf_buffer_)
  , obstructed_(false)
  , recovery_behavior_loader_()
  , robot_pose_available_(false)
{
  path_generator_config_ptr_ = std::make_shared<multi_point_navigation::PathGeneratorConfig>();
  path_generator.setConfig(path_generator_config_ptr_);
}

MultiPointNavigationHandlerBase::~MultiPointNavigationHandlerBase()
{
  recovery_behaviors_.clear();
}

bool MultiPointNavigationHandlerBase::setupHandler()
{
  if (!loadParams())
  {
    ROS_FATAL("[%s] Error during parameter loading. Shutting down.", name_.c_str());
    return false;
  }

  setupDerivedValues();

  setupTopicsAndServices();

  costmap_ptr_ = std::make_shared<costmap_2d::Costmap2DROS>("multi_point_map", tf_buffer_);

  if (p_recovery_behavior_enabled_)
  {
    if (!recovery_behavior_loader_.loadRecoveryBehaviors(nh_handler_, &tf_buffer_, costmap_ptr_.get(),
                                                         costmap_ptr_.get(), recovery_behaviors_))
    {
      ROS_FATAL("[%s] Error during recovery behaviors setup. Shutting down.", name_.c_str());
      return false;
    }
  }

  return true;
}

bool MultiPointNavigationHandlerBase::loadParams()
{
  ros_utils::ParamLoader param_loader(nh_handler_);

  param_loader.get_required("look_ahead_distance", p_look_ahead_dist_);
  param_loader.get_required("goal_tolerance", p_goal_tolerance_);
  param_loader.get_required("angular_tolerance", p_angular_tolerance_);
  param_loader.get_required("spline_enable", p_spline_enable_);
  param_loader.get_required("recovery_behavior_enabled", p_recovery_behavior_enabled_);

  /* params that need validation */

  param_loader.get_required("obstacle_timeout", p_obstruction_timeout_);
  if (p_obstruction_timeout_ < c_min_obstacle_timeout_)
  {
    ROS_WARN("[%s] Obstruction timeout too low, resetting to minimum %f sec", name_.c_str(), c_min_obstacle_timeout_);
    p_obstruction_timeout_ = c_min_obstacle_timeout_;
  }

  param_loader.get_required("obst_check_freq", p_obstacle_check_rate_);
  if (p_obstacle_check_rate_ < c_min_obstacle_check_rate_)
  {
    ROS_WARN("[%s] Obstruction check frequency too low, resetting to minimum min_obstacle_check_rate_ hz",
             name_.c_str());
    p_obstacle_check_rate_ = c_min_obstacle_check_rate_;
  }
  else if (p_obstacle_check_rate_ > c_max_obstacle_check_rate_)
  {
    ROS_WARN("[%s] Obstruction check frequency too high, resetting to maximum %.2f hz", name_.c_str(),
             c_max_obstacle_check_rate_);
    p_obstacle_check_rate_ = c_max_obstacle_check_rate_;
  }

  /* PathGeneratorParams */

  param_loader.get_required("points_distance", path_generator_config_ptr_->point_generation_distance);
  param_loader.get_required("max_spline_bypass_degree", path_generator_config_ptr_->max_bypass_degree);

  return true;
}

void MultiPointNavigationHandlerBase::setupDerivedValues()
{
  obstacle_check_interval_ = 1.0 / p_obstacle_check_rate_;
  look_ahead_points_ = std::max(int(p_look_ahead_dist_ / path_generator_config_ptr_->point_generation_distance),
                                c_min_lookahead_points_);
}

void MultiPointNavigationHandlerBase::setupTopicsAndServices()
{
  cmd_vel_pub_ = nh_handler_.advertise<geometry_msgs::Twist>("/cmd_vel_mux/autonomous", 1);
  coverage_percentage_pub_ = nh_handler_.advertise<std_msgs::Float64>("coverage_percentage", 1);
  current_goal_pub_ = nh_handler_.advertise<geometry_msgs::PoseStamped>("current_goal", 1);
  path_visualize_pub_ = nh_handler_.advertise<visualization_msgs::MarkerArray>("path", 10);

  robot_pose_sub_ = nh_handler_.subscribe("/pose", 1, &MultiPointNavigationHandlerBase::robotPoseCb, this);

  clear_costmap_srv_ =
      nh_handler_.advertiseService("clear_costmap", &MultiPointNavigationHandlerBase::clearCostmapCb, this);
  path_srv_ = nh_handler_.advertiseService("generate_path", &MultiPointNavigationHandlerBase::pathServiceCb, this);
}

ReturnCode MultiPointNavigationHandlerBase::runTask(movel_seirios_msgs::Task& task, std::string& error_message)
{
  task_cancelled_ = false;
  task_active_ = true;
  task_parsed_ = false;
  is_healthy_ = true;
  start_ = ros::Time::now();
  recovery_index_ = 0;

  std::vector<multi_point_navigation::Point> received_points;
  double linear_vel, angular_vel;
  bool start_at_nearest_point;
  if (!parseTask(task, received_points, linear_vel, angular_vel, start_at_nearest_point, error_message))
  {
    ROS_ERROR("[%s] Cannot parse task: %s", name_.c_str(), error_message.c_str());
    setMessage(error_message);
    setTaskResult(false);
    return code_;
  }

  multi_point_navigation::Path path_to_navigate;
  if (!generatePathForNavigation(received_points, start_at_nearest_point, path_to_navigate))
  {
    ROS_ERROR("[%s] Generating path failed", name_.c_str());
  }

  for (int i = 0; i < path_to_navigate.points.size(); ++i)
  {
    visualizePath(path_to_navigate, i, visualization_msgs::Marker::ADD);
    publishCurrentGoal(path_to_navigate.points[i]);

    if (!navigateToPoint(path_to_navigate, i))
    {
      error_message = "Navigation to point unsuccessful";
      ROS_ERROR("[%s] %s", name_.c_str(), error_message.c_str());
      setMessage(error_message);
      setTaskResult(false);
      return code_;
    }

    std::vector<int>::iterator it =
        std::find(path_to_navigate.major_points_idxs.begin(), path_to_navigate.major_points_idxs.end(), i);
    if (it != path_to_navigate.major_points_idxs.end())
    {
      int visited_major_point_idx = std::distance(path_to_navigate.major_points_idxs.begin(), it);
      json handler_feedback(visited_major_point_idx);
      publishHandlerFeedback(handler_feedback);
    }
  }

  ROS_INFO("[%s] Multi-point nav successfully completed", name_.c_str());
  setTaskResult(true);
  return code_;
}

bool MultiPointNavigationHandlerBase::parseTask(const movel_seirios_msgs::Task& task,
                                                std::vector<multi_point_navigation::Point>& major_pts,
                                                double& linear_veloctiy, double& angular_velocity,
                                                bool& start_at_nearest_point, std::string& error_msg)
{
  if (!parseTaskPayload(task.payload, major_pts, error_msg))
  {
    ROS_ERROR("[%s] Cannot parse task: %s", name_.c_str(), error_msg.c_str());
    return false;
  }

  linear_veloctiy = task.linear_velocity;
  angular_velocity = task.angular_velocity;
  validateTaskVelocity(linear_veloctiy, angular_velocity);

  return true;
}

bool MultiPointNavigationHandlerBase::parseTaskPayload(std::string payload_string,
                                                       std::vector<multi_point_navigation::Point>& major_pts,
                                                       std::string& error_msg)
{
  ROS_INFO("[%s] Task payload %s", name_.c_str(), payload_string.c_str());
  json payload = json::parse(payload_string);

  if (payload.find("path") == payload.end())
  {
    error_msg = "malformed payload";
    ROS_ERROR("[%s] Cannot parse task payload: %s", name_.c_str(), error_msg.c_str());
    return false;
  }

  for (auto& e : payload["path"])
  {
    multi_point_navigation::Point point = { .x = e["position"]["x"].get<float>(),
                                            .y = e["position"]["y"].get<float>() };
    major_pts.push_back(point);
  }

  return true;
}

void MultiPointNavigationHandlerBase::validateTaskVelocity(double& linear_veloctiy, double& angular_velocity)
{
  if (linear_veloctiy < c_min_linear_vel_)
    linear_veloctiy = c_min_linear_vel_;
  else if (linear_veloctiy > c_max_angular_vel_)
    linear_veloctiy = c_max_angular_vel_;

  if (angular_velocity < c_min_angular_vel_)
    angular_velocity = c_min_angular_vel_;
  else if (angular_velocity > c_max_angular_vel_)
    angular_velocity = c_max_angular_vel_;
}

void MultiPointNavigationHandlerBase::cancelTask()
{
  stopNavigation();
  setTaskResult(false);
  multi_point_navigation::Path empty_path;
  visualizePath(empty_path, 0, visualization_msgs::Marker::DELETE);

  task_cancelled_ = true;
  task_parsed_ = true;
  task_active_ = true;
  task_paused_ = true;
}

bool MultiPointNavigationHandlerBase::generatePathForNavigation(std::vector<multi_point_navigation::Point> major_pts,
                                                                bool start_at_nearest_point,
                                                                multi_point_navigation::Path& path)
{
  if (!start_at_nearest_point)
  {
    if (!prepareMajorPointsForPathGeneration(major_pts))
      return false;
  }

  if (!path_generator.generatePath(major_pts, path))
  {
    ROS_ERROR("[%s] Failed to generate path from major points for multipoint navigation", name_.c_str());
    return false;
  }

  if (start_at_nearest_point)
  {
    multi_point_navigation::Point current_pose = { .x = robot_pose_.position.x, .y = robot_pose_.position.y };
    int start_idx = path_generator.getIndexNearestPointAheadOnPath(path, current_pose);
    if (start_idx > 0)
    {
      for (int i = 0; i < major_pts.size(); ++i)
      {
        if (path.major_points_idxs[i] <= start_idx)
          major_pts.erase(major_pts.begin());
      }
      major_pts.insert(major_pts.begin(), path.points[start_idx]);
    }

    if (!path_generator.generatePath(major_pts, path))
    {
      ROS_ERROR("[%s] Failed to generate path from major points for multipoint navigation starting at nearest point", name_.c_str());
      return false;
    }
  }

  if (p_spline_enable_)
    path_generator.smoothenPath(path, path);

  ROS_INFO(
      "[%s] Info :\n Major points - %ld,"
      "\n Total nav points - %ld,"
      "\n Spline enable - %d,"
      "\n Obstacle check interval - %0.2f s,"
      "\n Obstacle timeout - %0.2f s,"
      "\n Look ahead points - %d",
      name_.c_str(), major_pts.size(), path.points.size(), p_spline_enable_, obstacle_check_interval_,
      p_obstruction_timeout_, look_ahead_points_);

  return true;
}

bool MultiPointNavigationHandlerBase::prepareMajorPointsForPathGeneration(
    std::vector<multi_point_navigation::Point> major_pts)
{
  ROS_INFO("[%s] Inserting current pose to the beginning of major points", name_.c_str());
  if (!robot_pose_available_)
  {
    ROS_ERROR("[%s] Robot pose unavailable", name_.c_str());
    return false;
  }
  geometry_msgs::Pose current_pose = robot_pose_;
  multi_point_navigation::Point robot_pose_vec = { .x = float(current_pose.position.x),
                                                    .y = float(current_pose.position.y) };
  major_pts.insert(major_pts.begin(), robot_pose_vec);

  return true;
}

bool MultiPointNavigationHandlerBase::checkForObstacle(const std::vector<multi_point_navigation::Point>& path_points,
                                                       int path_point_index, ObstructionType& obstruction_type)
{
  obstruction_type = ObstructionType::FREE;

  if (path_point_index > path_points.size() - 1)
  {
    ROS_WARN("[%s] Path point index should not exceed path size", name_.c_str());
    return false;
  }

  int max_checked_point_idx = std::min(path_point_index + look_ahead_points_, int(path_points.size() - 1));

  for (int i = path_point_index; i <= max_checked_point_idx; ++i)
  {
    double world_x, world_y;

    world_x = path_points[i].x;
    world_y = path_points[i].y;

    getObstructionTypeAtCoordinate(world_x, world_y, obstruction_type);
    if (obstruction_type != ObstructionType::FREE)
      return true;

    // checking midpoint of two path points in case there's a small obstruction inbetween
    if (i > 0)
    {
      world_x = (path_points[i].x + path_points[i - 1].x) / 2;
      world_y = (path_points[i].y + path_points[i - 1].y) / 2;

      getObstructionTypeAtCoordinate(world_x, world_y, obstruction_type);
      if (obstruction_type != ObstructionType::FREE)
        return true;
    }
  }

  ROS_INFO("[%s] No obstruction found at (%.2f, %.2f)", name_.c_str(), path_points[path_point_index].x,
           path_points[path_point_index].y);
  return false;
}

void MultiPointNavigationHandlerBase::getObstructionTypeAtCoordinate(double world_x, double world_y,
                                                                     ObstructionType& obstruction_type)
{
  costmap_2d::Costmap2D* sync_costmap = costmap_ptr_->getCostmap();
  unsigned int map_x, map_y;

  if (!sync_costmap->worldToMap(world_x, world_y, map_x, map_y))
  {
    ROS_WARN("[%s] getObstructionTypeAtCoordinate: coordinate out of bounds", name_.c_str());
    obstruction_type = ObstructionType::LETHAL;
    return;
  }
  unsigned char cost_i = sync_costmap->getCost(map_x, map_y);
  if (cost_i == costmap_2d::INSCRIBED_INFLATED_OBSTACLE)
  {
    ROS_INFO("[%s] Found obstruction - Inscribed inflation", name_.c_str());
    obstruction_type = ObstructionType::INSCRIBED_INFLATED;
    return;
  }
  if (cost_i == costmap_2d::LETHAL_OBSTACLE)
  {
    ROS_INFO("[%s] Found obstruction - Lethal obstacle", name_.c_str());
    obstruction_type = ObstructionType::LETHAL;
    return;
  }
  if (cost_i == costmap_2d::NO_INFORMATION)
  {
    ROS_INFO("[%s] Found obstruction - No Information", name_.c_str());
    obstruction_type = ObstructionType::NO_INFORMATION;
    return;
  }
}

void MultiPointNavigationHandlerBase::visualizePath(multi_point_navigation::Path& path, int point_index,
                                                    int marker_action)
{
  visualization_msgs::MarkerArray marker_array;

  // markers[0] is to visualize major points - sphere list
  visualization_msgs::Marker major_marker;
  major_marker.header.frame_id = "map";
  major_marker.header.stamp = ros::Time();
  major_marker.id = 0;
  major_marker.type = visualization_msgs::Marker::SPHERE_LIST;
  major_marker.action = marker_action;
  major_marker.pose.orientation.w = 1.0;
  major_marker.scale.x = 0.07;
  major_marker.scale.y = 0.07;
  major_marker.scale.z = 0.07;
  major_marker.color.r = 1;
  major_marker.color.g = 0;
  major_marker.color.b = 0;
  major_marker.color.a = 1;
  for (int i = 0; i < path.major_points_idxs.size(); ++i)
  {
    if (path.major_points_idxs[i] < path.points.size())
    {
      geometry_msgs::Point major_point_mark;
      major_point_mark.x = path.points[path.major_points_idxs[i]].x;
      major_point_mark.y = path.points[path.major_points_idxs[i]].y;
      major_marker.points.push_back(major_point_mark);
    }
  }
  marker_array.markers.push_back(major_marker);

  // markers[1] is to visualize generated path - line strip
  visualization_msgs::Marker path_marker;
  path_marker.header.frame_id = "map";
  path_marker.header.stamp = ros::Time();
  path_marker.id = 1;
  path_marker.type = visualization_msgs::Marker::LINE_STRIP;
  path_marker.action = marker_action;
  path_marker.pose.orientation.w = 1.0;
  path_marker.scale.x = 0.01;
  path_marker.color.r = 0;
  path_marker.color.g = 0;
  path_marker.color.b = 0;
  path_marker.color.a = 1;
  for (int i = point_index; i < path.points.size(); i++)
  {
    geometry_msgs::Point minor_point_mark;
    minor_point_mark.x = path.points[i].x;
    minor_point_mark.y = path.points[i].y;
    path_marker.points.push_back(minor_point_mark);
  }
  marker_array.markers.push_back(path_marker);

  // markers[2] is to visualize lookahead path - line strip
  visualization_msgs::Marker look_ahead_path;
  look_ahead_path.header.frame_id = "map";
  look_ahead_path.header.stamp = ros::Time();
  look_ahead_path.id = 2;
  look_ahead_path.type = visualization_msgs::Marker::LINE_STRIP;
  look_ahead_path.action = marker_action;
  look_ahead_path.pose.orientation.w = 1.0;
  look_ahead_path.scale.x = 0.02;
  look_ahead_path.color.r = 0;
  look_ahead_path.color.g = 1;
  look_ahead_path.color.b = 0;
  look_ahead_path.color.a = 1;

  int max_checked_point_idx = std::min(point_index + look_ahead_points_, int(path.points.size() - 1));
  for (int i = point_index; i < max_checked_point_idx; i++)
  {
    geometry_msgs::Point look_ahead_mark;
    look_ahead_mark.x = path.points[i].x;
    look_ahead_mark.y = path.points[i].y;
    look_ahead_path.points.push_back(look_ahead_mark);
  }
  marker_array.markers.push_back(look_ahead_path);

  path_visualize_pub_.publish(marker_array);
}

void MultiPointNavigationHandlerBase::publishCurrentGoal(const multi_point_navigation::Point& point)
{
  geometry_msgs::PoseStamped current_goal_msg;
  current_goal_msg.header.frame_id = "map";
  current_goal_msg.pose.position.x = point.x;
  current_goal_msg.pose.position.y = point.y;
  current_goal_pub_.publish(current_goal_msg);
}

bool MultiPointNavigationHandlerBase::pathServiceCb(movel_seirios_msgs::MultipointPath::Request& req,
                                                    movel_seirios_msgs::MultipointPath::Response& res)
{
  ROS_INFO("[%s] pathServiceCb received request to generate path", name_.c_str());
  if (req.major_points.size() <= 1)
  {
    ROS_ERROR("[%s] pathServiceCb error: not enough major points", name_.c_str());
    res.result = "not enough major points";
    return true;
  }

  std::vector<multi_point_navigation::Point> major_pts;
  for (auto& e : req.major_points)
  {
    multi_point_navigation::Point point = { .x = e.position.x, .y = e.position.y };
    major_pts.push_back(point);
  }

  multi_point_navigation::Path output_path;
  if (!path_generator.generatePath(major_pts, output_path))
  {
    ROS_ERROR("[%s] pathServiceCb error: cannot generate path", name_.c_str());
    res.result = "cannot generate path";
    return true;
  }
  for (auto& e : output_path.points)
  {
    geometry_msgs::Point output_point;
    output_point.x = e.x;
    output_point.y = e.y;
    res.generated_path.push_back(output_point);
  }
  ROS_INFO("[%s] pathServiceCb successfuly generated path", name_.c_str());
  res.result = "success";
  return true;
}

void MultiPointNavigationHandlerBase::robotPoseCb(const geometry_msgs::Pose::ConstPtr& msg)
{
  robot_pose_ = *msg;
  robot_pose_available_ = true;
}

bool MultiPointNavigationHandlerBase::clearCostmapCb(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
{
  ROS_INFO("[%s] Received request to clear costmap", name_.c_str());
  if (costmap_ptr_.get() != nullptr)
  {
    boost::unique_lock<costmap_2d::Costmap2D::mutex_t> planner_lock(*(costmap_ptr_.get()->getCostmap()->getMutex()));
    costmap_ptr_.get()->resetLayers();
    ROS_INFO("[%s] Costmap cleared", name_.c_str());
  }
  else
  {
    ROS_WARN("[%s] Costmap is not initialized, cannot clear costmap", name_.c_str());
  }
  return true;
}

}  // namespace task_supervisor