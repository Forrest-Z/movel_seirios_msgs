// Copyright (c) 2020, Samsung Research America
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License. Reserved.

#include "smac_planner/nav_smac_planner_2d.hpp"

#include <algorithm>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "pluginlib/class_list_macros.hpp"
#include "smac_planner/collision_checker.hpp"
#include "smac_planner/types.hpp"

// #define BENCHMARK_TESTING

namespace smac_planner
{
using namespace std::chrono;  // NOLINT

SmacPlanner2D::SmacPlanner2D()
: _a_star(nullptr), _smoother(nullptr), _costmap(nullptr), _costmap_downsampler(nullptr)
{
}

SmacPlanner2D::~SmacPlanner2D()
{
  ROS_INFO_STREAM("[smac_planner] Destroying plugin" << _name.c_str() << "of type SmacPlanner2D");
  if (dsrv_)
    delete dsrv_;
}

void SmacPlanner2D::initialize(std::string name, costmap_2d::Costmap2DROS * costmap_ros)
{
  _costmap = costmap_ros->getCostmap();
  _name = name;
  _global_frame = costmap_ros->getGlobalFrameID();

  ros::NodeHandle node_handle("~/" + name);

  // General planner params
  node_handle.param<float>("tolerance", _tolerance, 0.125);

  node_handle.param<bool>("downsample_costmap", _downsample_costmap, true);
  node_handle.param<int>("downsampling_factor", _downsampling_factor, 1);

  auto allow_unknown{true};
  node_handle.param<bool>("allow_unknown", allow_unknown, true);

  auto max_iterations{-1};
  node_handle.param<int>("max_iterations", max_iterations, -1);

  auto max_on_approach_iterations{1000};
  node_handle.param<int>("max_on_approach_iterations", max_on_approach_iterations, 1000);

  auto smooth_path{false};
  node_handle.param<bool>("smooth_path", smooth_path, false);

  auto minimum_turning_radius{0.2F};
  node_handle.param<float>("minimum_turning_radius", minimum_turning_radius, 0.2);

  node_handle.param<double>("max_planning_time_ms", _max_planning_time, 1000.0);

  std::string motion_model_for_search{"MOORE"};
  node_handle.param<std::string>(
    "motion_model_for_search", motion_model_for_search,
    std::string("MOORE"));

  MotionModel motion_model = fromString(motion_model_for_search);
  if (motion_model == MotionModel::UNKNOWN) {
    ROS_WARN_STREAM(
      "[smac_planner] Unable to get MotionModel search type. Given '" <<
        motion_model_for_search.c_str() <<
        "', "
        "valid options are MOORE, VON_NEUMANN, DUBIN, REEDS_SHEPP.");
  }

  if (max_on_approach_iterations <= 0) {
    ROS_INFO_STREAM(
      "[smac_planner] On approach iteration selected as <= 0, " <<
        "disabling tolerance and on approach iterations.");
    max_on_approach_iterations = std::numeric_limits<int>::max();
  }

  if (max_iterations <= 0) {
    ROS_INFO_STREAM(
      "[smac_planner] maximum iteration selected as <= 0, " <<
        "disabling maximum iterations.");
    max_iterations = std::numeric_limits<int>::max();
  }

  _a_star = std::make_unique<AStarAlgorithm<
        Node2D<GridCollisionChecker<
          smac_planner::FootprintCollisionChecker<costmap_2d::Costmap2D *>, costmap_2d::Costmap2D,
          smac_planner::Footprint>>,
        GridCollisionChecker<
          smac_planner::FootprintCollisionChecker<costmap_2d::Costmap2D *>, costmap_2d::Costmap2D,
          smac_planner::Footprint>,
        costmap_2d::Costmap2D, smac_planner::Footprint>>(motion_model, SearchInfo());
  _a_star->initialize(allow_unknown, max_iterations, max_on_approach_iterations);

  if (smooth_path) {
    _smoother = std::make_unique<Smoother<costmap_2d::Costmap2D>>();

    ros::NodeHandle s_node_handle(node_handle, "smoother");
    ros::NodeHandle smoother_node_handle(s_node_handle, "smoother");
    ros::NodeHandle optimizer_node_handle(s_node_handle, "optimizer");
    ros::NodeHandle advanced_node_handle(optimizer_node_handle, "advanced");

    // Optimizer advanced params
    auto min_line_search_step_size{1e-20};
    advanced_node_handle.param<double>(
      "min_line_search_step_size", _optimizer_params.advanced.min_line_search_step_size, 1e-20);

    advanced_node_handle.param<int>(
      "max_num_line_search_step_size_iterations",
      _optimizer_params.advanced.max_num_line_search_step_size_iterations, 50);

    advanced_node_handle.param<double>(
      "line_search_sufficient_function_decrease",
      _optimizer_params.advanced.line_search_sufficient_function_decrease, 1e-20);

    advanced_node_handle.param<int>(
      "max_num_line_search_direction_restarts",
      _optimizer_params.advanced.max_num_line_search_direction_restarts, 10);

    advanced_node_handle.param<int>(
      "max_line_search_step_expansion", _optimizer_params.advanced.max_line_search_step_expansion,
      50);

    // Optimizer params
    optimizer_node_handle.param<double>("param_tol", _optimizer_params.param_tol, 1e-20);
    optimizer_node_handle.param<double>("fn_tol", _optimizer_params.fn_tol, 1e-7);
    optimizer_node_handle.param<double>("gradient_tol", _optimizer_params.gradient_tol, 1e-10);
    optimizer_node_handle.param<int>("max_iterations", _optimizer_params.max_iterations, 500);
    optimizer_node_handle.param<double>("max_time", _optimizer_params.max_time, 0.100);

    auto debug_optimizer{false};
    optimizer_node_handle.param<bool>("debug_optimizer", debug_optimizer, false);

    // Get SmootherParams
    smoother_node_handle.param<double>("w_curve", _smoother_params.curvature_weight, 1.5);
    smoother_node_handle.param<double>("w_cost", _smoother_params.costmap_weight, 0.0);
    smoother_node_handle.param<double>("w_dist", _smoother_params.distance_weight, 0.0);
    smoother_node_handle.param<double>("w_smooth", _smoother_params.smooth_weight, 15000.0);
    smoother_node_handle.param<double>(
      "cost_scaling_factor", _smoother_params.costmap_factor, 10.0);
    _smoother_params.max_curvature = 1.0f / minimum_turning_radius;
    _smoother->initialize(_optimizer_params);
  }

  if (_downsample_costmap && _downsampling_factor > 1) {
    std::string topic_name = "downsampled_costmap";
    _costmap_downsampler = std::make_unique<CostmapDownsampler<costmap_2d::Costmap2D>>();
    _costmap_downsampler->on_configure(
      []() {}, _global_frame, topic_name, _costmap, _downsampling_factor);
  }

  dsrv_ = new dynamic_reconfigure::Server<smac_planner::SmacPlannerConfig>(ros::NodeHandle("~/" + name));
  dynamic_reconfigure::Server<smac_planner::SmacPlannerConfig>::CallbackType cb = boost::bind(&SmacPlanner2D::reconfigureCB, this, _1, _2);
  dsrv_->setCallback(cb);

  make_plan_srv_ = node_handle.advertiseService("make_plan", &SmacPlanner2D::makePlanService, this);
  _plan_publisher =
    std::make_unique<ros::Publisher>(node_handle.advertise<nav_msgs::Path>("plan", 1));

  ROS_INFO_STREAM(
    "[smac_planner] Configured plugin " << _name.c_str() <<
      " of type SmacPlanner2D with "
      "tolerance " <<
      _tolerance << ", maximum iterations " << max_iterations <<
      ", "
      "max on approach iterations " <<
      max_on_approach_iterations << ", and " <<
    (allow_unknown ? "allowing unknown traversal" :
    "not allowing unknown traversal") <<
      " Using motion model: " << toString(motion_model).c_str() << ".");
}

void SmacPlanner2D::reconfigureCB(smac_planner::SmacPlannerConfig& config, uint32_t level) {
  _tolerance = config.tolerance;
  _max_planning_time = config.max_planning_time_ms;
}

void SmacPlanner2D::activate()
{
  ROS_INFO_STREAM("[smac_planner] Activating plugin " << _name.c_str() << " of type SmacPlanner2D");
  if (_costmap_downsampler) {
    // _costmap_downsampler->on_activate();
  }
}

void SmacPlanner2D::deactivate()
{
  ROS_INFO_STREAM("[smac_planner] Deactivating plugin " << _name.c_str() << " of type SmacPlanner2D");
  if (_costmap_downsampler) {
    // _costmap_downsampler->on_deactivate();
  }
}

void SmacPlanner2D::cleanup()
{
  ROS_INFO_STREAM("[smac_planner] Cleaning up plugin " << _name.c_str() << " of type SmacPlanner2D");
  _a_star.reset();
  _smoother.reset();
  _costmap_downsampler->on_cleanup();
  _costmap_downsampler.reset();
  _plan_publisher.reset();
}

bool SmacPlanner2D::makePlanService(nav_msgs::GetPlan::Request& req, nav_msgs::GetPlan::Response& resp)
{
  makePlan(req.start, req.goal, resp.plan.poses);
  resp.plan.header.stamp = ros::Time::now();
  resp.plan.header.frame_id = _global_frame;
  return true;
}

bool SmacPlanner2D::makePlan(
  const geometry_msgs::PoseStamped & start, const geometry_msgs::PoseStamped & goal,
  std::vector<geometry_msgs::PoseStamped> & planVector)
{
  steady_clock::time_point a = steady_clock::now();
  std::unique_lock<costmap_2d::Costmap2D::mutex_t> lock(*(_costmap->getMutex()));

  // Downsample costmap, if required
  costmap_2d::Costmap2D * costmap = _costmap;
  if (_costmap_downsampler) {
    costmap = _costmap_downsampler->downsample(_downsampling_factor);
  }

  ROS_DEBUG_STREAM(
    "X: " << _costmap->getOriginX() << " Y: " << _costmap->getOriginY() << " \n size: " <<
      _costmap->getSizeInMetersX() << " y: " << _costmap->getSizeInMetersY());

  // Set Costmap
  _a_star->createGraph(costmap->getSizeInCellsX(), costmap->getSizeInCellsY(), 1, costmap);

  // Set starting point
  unsigned int mx, my;
  costmap->worldToMap(start.pose.position.x, start.pose.position.y, mx, my);
  _a_star->setStart(mx, my, 0);

  // Set goal point
  costmap->worldToMap(goal.pose.position.x, goal.pose.position.y, mx, my);
  _a_star->setGoal(mx, my, 0);

  // Setup message
  nav_msgs::Path plan;
  plan.header.stamp = ros::Time::now();
  plan.header.frame_id = _global_frame;
  geometry_msgs::PoseStamped pose;
  pose.header = plan.header;
  pose.pose.position.z = 0.0;
  pose.pose.orientation.x = 0.0;
  pose.pose.orientation.y = 0.0;
  pose.pose.orientation.z = 0.0;
  pose.pose.orientation.w = 1.0;

  // Compute plan
  Node2D<GridCollisionChecker<
      smac_planner::FootprintCollisionChecker<costmap_2d::Costmap2D *>, costmap_2d::Costmap2D,
      smac_planner::Footprint>>::CoordinateVector path;
  int num_iterations = 0;
  std::string error;
  try {
    if (!_a_star->createPath(
        path, num_iterations, _tolerance / static_cast<float>(costmap->getResolution())))
    {
      if (num_iterations < _a_star->getMaxIterations()) {
        error = std::string("no valid path found");
      } else {
        error = std::string("exceeded maximum iterations");
      }
    }
  } catch (const std::runtime_error & e) {
    error = "invalid use: ";
    error += e.what();
  }

  if (!error.empty()) {
    ROS_WARN_STREAM("[smac_planner] " << _name.c_str() << ": failed to create plan, " << error.c_str() << ".");
    return false;
  }

  // Convert to world coordinates and downsample path for smoothing if necesssary
  // We're going to downsample by 4x to give terms room to move.
  const int downsample_ratio = 4;
  std::vector<Eigen::Vector2d> path_world;
  path_world.reserve(_smoother ? path.size() / downsample_ratio : path.size());
  plan.poses.reserve(_smoother ? path.size() / downsample_ratio : path.size());

  for (int i = path.size() - 1; i >= 0; --i) {
    if (_smoother && i % downsample_ratio != 0) {
      continue;
    }

    path_world.push_back(getWorldCoords(path[i].x, path[i].y, costmap));
    pose.pose.position.x = path_world.back().x();
    pose.pose.position.y = path_world.back().y();
    plan.poses.push_back(pose);
    planVector.push_back(pose);
  }

  // Publish raw path for debug
  //if (_raw_plan_publisher->getNumSubscribers() > 0) {
  //  _raw_plan_publisher->publish(plan);
  //}

  // If not smoothing or too short to smooth, return path
  if (!_smoother || path_world.size() < 4) {
#ifdef BENCHMARK_TESTING
    steady_clock::time_point b = steady_clock::now();
    duration<double> time_span = duration_cast<duration<double>>(b - a);
    std::cout << "It took " << time_span.count() * 1000 << " milliseconds with " <<
      num_iterations <<
      " iterations." << std::endl;
#endif
    // Publish raw path for debug
    if (_plan_publisher->getNumSubscribers() > 0) {
      _plan_publisher->publish(plan);
    }
    return true;
  }

  // Find how much time we have left to do smoothing
  steady_clock::time_point b = steady_clock::now();
  duration<double> time_span = duration_cast<duration<double>>(b - a);
  double time_remaining = _max_planning_time - static_cast<double>(time_span.count());
  _smoother_params.max_time = std::min(time_remaining, _optimizer_params.max_time);

  // Smooth plan
  if (!_smoother->smooth(path_world, costmap, _smoother_params)) {
    ROS_WARN_STREAM(
      "[smac_planner] " << _name.c_str() <<
        ": failed to smooth plan, Ceres could not find a usable solution to optimize.");
    return true;
  }

  removeHook(path_world);

  // populate final path
  for (uint i = 0; i != path_world.size(); i++) {
    pose.pose.position.x = path_world[i][0];
    pose.pose.position.y = path_world[i][1];
    plan.poses[i] = pose;
    planVector[i] = pose;
  }
  if (_plan_publisher->getNumSubscribers() > 0) {
    _plan_publisher->publish(plan);
  }
  return true;
}

void SmacPlanner2D::removeHook(std::vector<Eigen::Vector2d> & path)
{
  // Removes the end "hooking" since goal is locked in place
  Eigen::Vector2d interpolated_second_to_last_point;
  interpolated_second_to_last_point = (path.end()[-3] + path.end()[-1]) / 2.0;
  if (
    squaredDistance(path.end()[-2], path.end()[-1]) >
    squaredDistance(interpolated_second_to_last_point, path.end()[-1]))
  {
    path.end()[-2] = interpolated_second_to_last_point;
  }
}

Eigen::Vector2d SmacPlanner2D::getWorldCoords(
  const float & mx, const float & my, const costmap_2d::Costmap2D * costmap)
{
  float world_x = static_cast<float>(costmap->getOriginX()) + (mx + 0.5) * costmap->getResolution();
  float world_y = static_cast<float>(costmap->getOriginY()) + (my + 0.5) * costmap->getResolution();
  return Eigen::Vector2d(world_x, world_y);
}

}  // namespace smac_planner

PLUGINLIB_EXPORT_CLASS(smac_planner::SmacPlanner2D, nav_core::BaseGlobalPlanner)
