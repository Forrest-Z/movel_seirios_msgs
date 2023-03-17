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
  , recovery_loader_("nav_core", "nav_core::RecoveryBehavior")
  , obstructed_(false)
{
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
    if (!setupRecoveryBehaviors())
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

  param_loader.get_required("points_distance", p_point_gen_dist_);
  param_loader.get_required("look_ahead_distance", p_look_ahead_dist_);
  param_loader.get_required("goal_tolerance", p_goal_tolerance_);
  param_loader.get_required("angular_tolerance", p_angular_tolerance_);
  param_loader.get_required("spline_enable", p_spline_enable_);
  param_loader.get_required("max_spline_bypass_degree", p_bypass_degree_);
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
}

void MultiPointNavigationHandlerBase::setupDerivedValues()
{
  obstacle_check_interval_ = 1.0 / p_obstacle_check_rate_;
  look_ahead_points_ = std::max(int(p_look_ahead_dist_ / p_point_gen_dist_), min_lookahead_points_);
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

bool MultiPointNavigationHandlerBase::setupRecoveryBehaviors()
{
  XmlRpc::XmlRpcValue behavior_list;
  if (loadRecoveryBehaviors(behavior_list))
  {
    if (validateRecoveryBehaviorList(behavior_list))
    {
      return createRecoveryBehaviors(behavior_list);
    }
  }
  return false;
}

bool MultiPointNavigationHandlerBase::loadRecoveryBehaviors(XmlRpc::XmlRpcValue& behavior_list_out)
{
  if (!nh_handler_.getParam("recovery_behaviors", behavior_list_out))
  {
    ROS_WARN("[%s] No recovery behaviors specified", name_.c_str());
    return false;
  }
  return true;
}

bool MultiPointNavigationHandlerBase::validateRecoveryBehaviorList(const XmlRpc::XmlRpcValue& behavior_list)
{
  if (!behavior_list.getType() == XmlRpc::XmlRpcValue::TypeArray)
  {
    ROS_FATAL("[%s] Expected list of recovery behavior to be XmlRpcValue Type %d, instead received XmlRpcValue Type %d",
              name_.c_str(), XmlRpc::XmlRpcValue::TypeArray, behavior_list.getType());
    return false;
  }

  std::vector<std::string> behavior_names;
  for (int i = 0; i < behavior_list.size(); ++i)
  {
    if (!behavior_list[i].getType() == XmlRpc::XmlRpcValue::TypeStruct)
    {
      ROS_FATAL("[%s] Expected recovery behavior to be map, instead received XmlRpcValue Type %d", name_.c_str(),
                behavior_list[i].getType());
      return false;
    }

    if (!behavior_list[i].hasMember("name") || !behavior_list[i].hasMember("type"))
    {
      ROS_FATAL("[%s] Recovery behaviors must have a name and a type but this one does not");
      return false;
    }

    std::string behavior_name = behavior_list[i]["name"];
    if (std::find(behavior_names.begin(), behavior_names.end(), behavior_name) != behavior_names.end())
    {
      ROS_FATAL("[%s] A recovery behavior with the name %s already exists, this is not allowed", name_.c_str(),
                behavior_name.c_str());
      return false;
    }
    else
    {
      behavior_names.push_back(behavior_name);
    }
  }

  return true;
}

bool MultiPointNavigationHandlerBase::createRecoveryBehaviors(const XmlRpc::XmlRpcValue& behavior_list)
{
  for (int i = 0; i < behavior_list.size(); ++i)
  {
    std::string behavior_name = behavior_list[i]["name"];
    std::string behavior_type = behavior_list[i]["type"];
    try
    {
      // check existence of plugin class/type
      if (!recovery_loader_.isClassAvailable(behavior_type))
      {
        // cannot find plugin class name, check if a non-fully-qualified name has potentially been passed instead
        std::vector<std::string> plugin_classes = recovery_loader_.getDeclaredClasses();
        for (unsigned int j = 0; j < plugin_classes.size(); ++j)
        {
          if (behavior_type == recovery_loader_.getName(plugin_classes[i]))
          {
            // found the supposed plugin class
            ROS_WARN(
                "[%s] Recovery behavior specifications should now include the package name. You are using a deprecated "
                "API. Please switch from %s to %s in your yaml file",
                name_.c_str(), behavior_type.c_str(), plugin_classes[i].c_str());
            behavior_type = plugin_classes[i];
            break;
          }
        }
      }

      boost::shared_ptr<nav_core::RecoveryBehavior> behavior(recovery_loader_.createInstance(behavior_type));

      if (behavior.get() == NULL)
      {
        ROS_FATAL("[%s] The ClassLoader returned a null pointer without throwing an exception. This should not happen",
                  name_.c_str());
        return false;
      }

      behavior->initialize(behavior_name, &tf_buffer_, costmap_ptr_.get(), costmap_ptr_.get());
      recovery_behaviors_.push_back(behavior);
    }
    catch (pluginlib::PluginlibException& e)
    {
      ROS_FATAL("[%s] Failed to load recovery behavior plugin %s: %s", name_.c_str(), behavior_name.c_str(), e.what());
      return false;
    }
  }

  return true;
}

bool MultiPointNavigationHandlerBase::generatePathForNavigation(std::vector<std::vector<float>> major_pts,
                                                                std::vector<std::vector<float>>& complete_path)
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
    std::vector<float> robot_pose_vec = { float(current_pose.position.x), float(current_pose.position.y) };
    major_pts.insert(major_pts.begin(), robot_pose_vec);
  }

  if (!generatePathFromMajorPoints(major_pts, complete_path, major_indices_))
  {
    ROS_ERROR("[%s] Failed to generate path from major points for multipoint navigation", name_.c_str());
    return false;
  }

  ROS_INFO(
      "[%s] Info :\n Major points - %ld,"
      "\n Total nav points - %ld,"
      "\n Spline enable - %d,"
      "\n Obstacle check interval - %0.2f s,"
      "\n Obstacle timeout - %0.2f s,"
      "\n Look ahead points - %d",
      name_.c_str(), major_pts.size(), complete_path.size(), p_spline_enable_, obstacle_check_interval_,
      p_obstruction_timeout_, look_ahead_points_);

  // Print generated nav points
  printGeneratedPath(major_pts);

  return true;
}

bool MultiPointNavigationHandlerBase::generatePathFromMajorPoints(std::vector<std::vector<float>> major_pts,
                                                                  std::vector<std::vector<float>>& complete_path,
                                                                  std::vector<int>& major_pts_idxs)
{
  std::vector<std::vector<float>> coords_for_spline;
  std::vector<std::vector<int>> points_to_spline;
  int bypass_degree = p_bypass_degree_;

  std::vector<int> tmp_major_pts_idxs = { 0 };

  for (int i = 0; i < major_pts.size() - 1; i++)
  {
    std::vector<float> start_pt = major_pts[i];
    std::vector<float> end_pt = major_pts[i + 1];

    if (start_pt[0] == end_pt[0] && start_pt[1] == end_pt[1])
    {
      ROS_ERROR("[%s] Having 2 major points with same coordinates (%.2f, %.2f). This is not allowed", name_.c_str(), start_pt[0], start_pt[1]);
      return false;
    }

    tmp_major_pts_idxs.push_back(tmp_major_pts_idxs.back());

    int major_idx_increment = generateMinorPointsFromLineSegment(start_pt, end_pt, coords_for_spline);

    tmp_major_pts_idxs.back() = tmp_major_pts_idxs.back() + major_idx_increment;
  }
  coords_for_spline.push_back(major_pts.back());

  // Check if spline/smoothening enabled
  if (p_spline_enable_)
  {
    if (major_pts.size() > 2 && coords_for_spline.size() > 0 &&
        getPointsToSpline(major_pts, tmp_major_pts_idxs, points_to_spline))
    {
      // Spline/smoothen points
      splinePoints(coords_for_spline, points_to_spline, complete_path);
    }
    else
    {
      // No spline
      complete_path = coords_for_spline;
    }
  }
  else
  {
    // No spline
    complete_path = coords_for_spline;
  }

  major_pts_idxs = tmp_major_pts_idxs;

  return true;
}

int MultiPointNavigationHandlerBase::generateMinorPointsFromLineSegment(const std::vector<float>& start_point, const std::vector<float>& end_point, std::vector<std::vector<float>>& minor_pt_container)
{
  float major_pts_dx = end_point[0] - start_point[0];
  float major_pts_dy = end_point[1] - start_point[1];
  float major_pts_dist = std::sqrt(pow(major_pts_dx, 2) + pow(major_pts_dy, 2));
  float n_generated_minor_pts = major_pts_dist / p_point_gen_dist_;
  float min_dist_between_pts = 0.1;
  if ((n_generated_minor_pts - int(n_generated_minor_pts)) * p_point_gen_dist_ < min_dist_between_pts)
    n_generated_minor_pts--;
  
  int total_generated_points = 0;

  if (major_pts_dx != 0)  // slope of line segment from start_point to end_point is not inf
  {
    float slope = major_pts_dy / major_pts_dx;

    for (int j = 0; j <= int(n_generated_minor_pts); j++)
    {
      float minor_pt_x = start_point[0] + ((p_point_gen_dist_ * j) * (major_pts_dx / major_pts_dist));
      float minor_pt_y = slope * (minor_pt_x - start_point[0]) + start_point[1];

      std::vector<float> minor_pt = { minor_pt_x, minor_pt_y };
      minor_pt_container.push_back(minor_pt);
      total_generated_points = total_generated_points + 1;
    }
  }
  else  // slope of line segment from start_point to end_point is inf (line is perfectly vertical)
  {
    for (int j = 0; j < int(n_generated_minor_pts); j++)
    {
      // c++ shorthand to get sign of major_pts_dy (returns 1 if >0, -1 if <0, 0 if ==0)
      int signum_dy = (float(0) < major_pts_dy) - (major_pts_dy < float(0));

      float minor_pt_x = start_point[0];
      float minor_pt_y = start_point[1] + (signum_dy * (p_point_gen_dist_ * j));

      std::vector<float> minor_pt = { minor_pt_x, minor_pt_y };
      minor_pt_container.push_back(minor_pt);
      total_generated_points = total_generated_points + 1;
    }
  }

  return total_generated_points;
}

}  // namespace task_supervisor