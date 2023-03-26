#include <task_supervisor/plugins/base/multi_point_navigation_handler_base.h>

#include <algorithm>

#include <ros_utils/ros_utils.h>
#include <movel_common_libs/json.hpp>

using json = nlohmann::json;

namespace task_supervisor
{
MultiPointNavigationHandlerBase::MultiPointNavigationHandlerBase()
  : task_cancelled_(false), is_healthy_(true), tf_listener_(tf_buffer_), obstructed_(false), recovery_behavior_loader_()
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
  setupDynamicReconfigure();

  if (!loadParams())
  {
    ROS_FATAL("[%s] Error during parameter loading. Shutting down.", name_.c_str());
    return false;
  }

  setupDerivedValues();

  setupTopicsAndServices();

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
  param_loader.get_required("recovery_behavior_enabled_", p_recovery_behavior_enabled_);

  /* params that need validation */

  param_loader.get_required("obstacle_timeout", p_obstruction_timeout_);
  if (p_obstruction_timeout_ < min_obstacle_timeout_)
  {
    ROS_WARN("[%s] Obstruction timeout too low, resetting to minimum %f sec", name_.c_str(), min_obstacle_timeout_);
    p_obstruction_timeout_ = min_obstacle_timeout_;
  }

  param_loader.get_required("obst_check_freq", p_obstacle_check_rate_);
  if (p_obstacle_check_rate_ < min_obstacle_check_rate_)
  {
    ROS_WARN("[%s] Obstruction check frequency too low, resetting to minimum min_obstacle_check_rate_ hz",
             name_.c_str());
    p_obstacle_check_rate_ = min_obstacle_check_rate_;
  }
  else if (p_obstacle_check_rate_ > max_obstacle_check_rate_)
  {
    ROS_WARN("[%s] Obstruction check frequency too high, resetting to maximum max_obstacle_check_rate_ hz",
             name_.c_str());
    p_obstacle_check_rate_ = max_obstacle_check_rate_;
  }

  /* PathGeneratorParams */

  param_loader.get_required("points_distance", path_generator_config_ptr_->point_generation_distance);
  param_loader.get_required("max_spline_bypass_degree", path_generator_config_ptr_->max_bypass_degree);
}

void MultiPointNavigationHandlerBase::setupDerivedValues()
{
  obstacle_check_interval_ = 1.0 / p_obstacle_check_rate_;
  look_ahead_points_ =
      std::max(int(p_look_ahead_dist_ / path_generator_config_ptr_->point_generation_distance), min_lookahead_points_);
}

void MultiPointNavigationHandlerBase::setupTopicsAndServices()
{
  cmd_vel_pub_ = nh_handler_.advertise<geometry_msgs::Twist>("/cmd_vel_mux/autonomous", 1);
  coverage_percentage_pub_ = nh_handler_.advertise<std_msgs::Float64>("coverage_percentage", 1);
  current_goal_pub_ = nh_handler_.advertise<movel_seirios_msgs::MultipointProgress>("current_goal", 1);
  path_visualize_pub_ = nh_handler_.advertise<visualization_msgs::MarkerArray>("path", 10);

  robot_pose_sub_ = nh_handler_.subscribe("/pose", 1, &MultiPointNavigationHandlerBase::robotPoseCb, this);

  clear_costmap_srv_ =
      nh_handler_.advertiseService("clear_costmap", &MultiPointNavigationHandlerBase::clearCostmapCb, this);
  path_srv_ = nh_handler_.advertiseService("generate_path", &MultiPointNavigationHandlerBase::pathServiceCb, this);
}

void MultiPointNavigationHandlerBase::setupDynamicReconfigure()
{
  ros::NodeHandle nh("~" + name_);
  dynamic_reconfigure_srv_.reset(new dynamic_reconfigure::Server<multi_point::MultipointConfig>(nh));
  dynamic_reconfigure_cb_ = boost::bind(&MultiPointNavigationHandlerBase::reconfigureCb, this, _1, _2);
  dynamic_reconfigure_srv_->setCallback(dynamic_reconfigure_cb_);
}

bool MultiPointNavigationHandlerBase::generatePathForNavigation(std::vector<multi_point_navigation::Point> major_pts,
                                                                multi_point_navigation::Path& path)
{
  if (!prepareMajorPointsForPathGeneration(major_pts))
    return false;

  if (!path_generator.generatePath(major_pts, path))
  {
    ROS_ERROR("[%s] Failed to generate path from major points for multipoint navigation", name_.c_str());
    return false;
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

  // Print generated nav points
  printGeneratedPath(major_pts);

  return true;
}

bool MultiPointNavigationHandlerBase::prepareMajorPointsForPathGeneration(
    std::vector<multi_point_navigation::Point> major_pts)
{
  if (!start_at_nearest_point_)
  {
    if (at_start_point_)
    {
      ROS_INFO("[%s] Removing first major point because robot is close to it already", name_.c_str());
      major_pts.erase(major_pts.begin());
    }

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
  }
}

}  // namespace task_supervisor