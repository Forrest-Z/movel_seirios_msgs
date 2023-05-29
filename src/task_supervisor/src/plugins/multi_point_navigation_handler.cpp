#include <task_supervisor/plugins/multi_point_navigation_handler.h>
#include <movel_common_libs/json.hpp>
#include <pluginlib/class_list_macros.h>
#include <actionlib_msgs/GoalID.h>
#include <movel_seirios_msgs/GetReachableSubplan.h>
#include <type_traits>
#include <movel_fms_utils/path_dist_utils.hpp>

PLUGINLIB_EXPORT_CLASS(task_supervisor::MultiPointNavigationHandler, task_supervisor::TaskHandler);

using json = nlohmann::json;

class ObstacleRaytracer
{
private:
  const costmap_2d::Costmap2D& costmap_;
  bool* is_line_obstructed_;

public:
  ObstacleRaytracer(const costmap_2d::Costmap2D& costmap, bool* is_line_obstructed) :
    costmap_(costmap),
    is_line_obstructed_(is_line_obstructed)
  {
  }

  inline void operator()(unsigned int offset)
  {
    unsigned int mx, my;
    costmap_.indexToCells(offset, mx, my);

    unsigned char cost = costmap_.getCost(mx, my);

    if (cost == costmap_2d::INSCRIBED_INFLATED_OBSTACLE ||
        cost == costmap_2d::LETHAL_OBSTACLE || 
        cost == costmap_2d::NO_INFORMATION)
    {
      *is_line_obstructed_ = true;
    }
  }
};

class LineObstructionChecker : public costmap_2d::Costmap2D
{
public:
  LineObstructionChecker(const costmap_2d::Costmap2D& map) :
    costmap_2d::Costmap2D(map)
  {
  }

  bool isLineObstructed(std::vector<float> p1, std::vector<float> p2)
  {
    unsigned int m1x, m1y, m2x, m2y;

    if (!worldToMap(p1[0], p1[1], m1x, m1y) || 
        !worldToMap(p2[0], p2[1], m2x, m2y))
    {
      ROS_INFO("[LineObstructionChecker] isLineObstructed: Failed to raytrace, out of bounds");
      return true;
    }

    bool is_line_obstructed;
    ObstacleRaytracer obstacle_raytracer(*this, &is_line_obstructed);
    raytraceLine(obstacle_raytracer, m1x, m1y, m2x, m2y);

    return is_line_obstructed;
  }
};

double calcPoseDistances(geometry_msgs::Pose a, geometry_msgs::Pose b)
{
  double dx, dy, dee;
  dx = a.position.x - b.position.x;
  dy = a.position.y - b.position.y;
  dee = sqrt(dx * dx + dy * dy);
  return dee;
}

double calcPoseStampedDistances(geometry_msgs::PoseStamped a, geometry_msgs::PoseStamped b)
{
  double dx, dy, dee;
  dx = a.pose.position.x - b.pose.position.x;
  dy = a.pose.position.y - b.pose.position.y;
  dee = sqrt(dx * dx + dy * dy);
  return dee;
}

namespace task_supervisor
{
MultiPointNavigationHandler::MultiPointNavigationHandler()
  : task_cancelled_(false)
  , isHealthy_(true)
  , tf_ear_(tf_buffer_)
  , recovery_loader_("nav_core", "nav_core::RecoveryBehavior")
{
}

MultiPointNavigationHandler::~MultiPointNavigationHandler()
{
  recovery_behaviors_.clear();
}

bool MultiPointNavigationHandler::setupHandler()
{
  /* TODO :
  -> documentation (including bypass degree calculation for turning radius)
  -> fix costmap_common_params overwriting
  -> confirm format for current goal publish
  -> if name changes, change name of srv file too
  */

  // Dynamic Reconfigure
  ros::NodeHandle nl("~"+name_);
  dynamic_reconf_server_.reset(new dynamic_reconfigure::Server<multi_point::MultipointConfig>(nl));
  dynamic_reconfigure_callback_ = boost::bind(&MultiPointNavigationHandler::reconfCB, this, _1, _2);
  dynamic_reconf_server_->setCallback(dynamic_reconfigure_callback_);

  if (!loadParams())
  {
    ROS_FATAL("[%s] Error during parameter loading. Shutting down.", name_.c_str());
    return false;
  }

  // Check minimum obstacle timeout
  if (p_obstruction_timeout_ < min_obst_timeout_)
  {
    ROS_WARN("[%s] Obstruction timeout too low, resetting to minimum %f sec", name_.c_str(), min_obst_timeout_);
    p_obstruction_timeout_ = min_obst_timeout_;
  }
  // Obstacle checking frequency
  if (p_obst_check_freq_ > 0.5 && p_obst_check_freq_ < 10.0)
  {
    obst_check_interval_ = 1 / p_obst_check_freq_;
  }
  // Obstacle look ahead points from distance
  if (int(p_look_ahead_dist_ / p_point_gen_dist_) > look_ahead_points_)
  {
    look_ahead_points_ = int(p_look_ahead_dist_ / p_point_gen_dist_);
  }
  // Angular tolerance
  if (p_angular_tolerance_ > angular_tolerance_)
  {
    angular_tolerance_ = p_angular_tolerance_;
  }

  path_visualize_pub_ = nh_handler_.advertise<visualization_msgs::MarkerArray>("path", 10);
  robot_pose_sub_ = nh_handler_.subscribe("/pose", 1, &MultiPointNavigationHandler::robotPoseCB, this);
  cmd_vel_pub_ = nh_handler_.advertise<geometry_msgs::Twist>("/cmd_vel_mux/autonomous", 1);
  current_goal_pub_ = nh_handler_.advertise<movel_seirios_msgs::MultipointProgress>("current_goal", 1);
  obstacle_path_pub_ = nh_handler_.advertise<nav_msgs::Path>("/obstacle_avoidance_path", 1);
  path_srv_ = nh_handler_.advertiseService("generate_path", &MultiPointNavigationHandler::pathServiceCb, this);
  clear_costmap_srv_ =
      nh_handler_.advertiseService("clear_costmap", &MultiPointNavigationHandler::clearCostmapCb, this);
  coverage_percentage_pub_ = nh_handler_.advertise<std_msgs::Float64>("coverage_percentage", 1);
  make_plan_client_ = nh_handler_.serviceClient<nav_msgs::GetPlan>("/move_base/GlobalPlanner/make_plan");
  make_reachable_plan_client_ = nh_handler_.serviceClient<nav_msgs::GetPlan>("/planner_utils/make_reachable_plan");
  stop_at_obstacle_enabled_client_ = nh_handler_.serviceClient<std_srvs::Trigger>("/stop_obstacle_check");

  obstructed_ = true;
  return true;
}

template <typename param_type>
bool MultiPointNavigationHandler::load_param_util(std::string param_name, param_type& output)
{
  if (!nh_handler_.getParam(param_name, output))
  {
    ROS_ERROR("[%s] Failed to load parameter: %s", name_.c_str(), param_name.c_str());
    return false;
  }
  else
  {
    if (std::is_same<param_type, bool>::value)
    {  // bool
      ROS_INFO_STREAM("[" << name_ << "] " << param_name << ": " << output ? "true" : "false");
    }
    else
    {  // all others
      ROS_INFO_STREAM("[" << name_ << "] " << param_name << ": " << output);
    }
    return true;
  }
}

bool MultiPointNavigationHandler::loadParams()
{
  ROS_WARN(
      "[%s] Loading of plugin parameters by ros_utils has not been implemented. Loading directly from Parameter "
      "Server instead.",
      name_.c_str());

  if (!load_param_util("points_distance", p_point_gen_dist_)){return false;}
  if (!load_param_util("look_ahead_distance", p_look_ahead_dist_)){return false;}
  if (!load_param_util("obst_check_freq", p_obst_check_freq_)){return false;}
  if (!load_param_util("goal_tolerance", p_goal_tolerance_)){return false;}
  if (!load_param_util("angular_tolerance", p_angular_tolerance_)){return false;}
  if (!load_param_util("spline_enable", p_spline_enable_)){return false;}
  if (!load_param_util("obstacle_timeout", p_obstruction_timeout_)){return false;}
  if (!load_param_util("kp", p_kp_)){return false;}
  if (!load_param_util("ki", p_ki_)){return false;}
  if (!load_param_util("kd", p_kd_)){return false;}
  if (!load_param_util("forward_only", p_forward_only_)){return false;}
  if (!load_param_util("max_linear_acc", p_linear_acc_)){return false;}
  if (!load_param_util("max_angular_acc", p_angular_acc_)){return false;}
  if (!load_param_util("max_spline_bypass_degree", p_bypass_degree_)){return false;} 
  if (!load_param_util("slow_curve_vel", p_curve_vel_)){return false;}
  if (!load_param_util("recovery_behavior_enabled_", p_recovery_behavior_enabled_)){return false;}
  // New params, do nothing if fail for now
  if (!load_param_util("slow_curve_scale", p_curve_scale_)){}
  if (!load_param_util("slow_at_points_enable", p_slow_points_enable_)){}
  if (!load_param_util("slow_at_curve_enable", p_slow_curve_enable_)){}
  if (!load_param_util("max_linear_dacc", p_linear_dacc_)){}

  // stop at obstacle
  if (!load_param_util("/move_base/stop_at_obstacle", p_stop_at_obstacle_)){p_stop_at_obstacle_ = false;}

  return true;
}

ReturnCode MultiPointNavigationHandler::runTask(movel_seirios_msgs::Task& task, std::string& error_message)
{
  task_cancelled_ = false;
  task_active_ = true;
  task_parsed_ = false;
  isHealthy_ = true;
  start_ = ros::Time::now();
  recovery_index_ = 0;

  ROS_INFO("[%s] Task payload %s", name_.c_str(), task.payload.c_str());
  json payload = json::parse(task.payload);

  if (payload.find("path") != payload.end())
  {
    // Get received coordinates
    std::vector<std::vector<float>> rcvd_coords;
    for (auto& elem : payload["path"])
    {
      std::vector<float> coord_instance;
      coord_instance.push_back(elem["position"]["x"].get<float>());
      coord_instance.push_back(elem["position"]["y"].get<float>());
      rcvd_coords.push_back(coord_instance);
    }

    // Set task velocities
    if (task.angular_velocity > min_angular_vel_ && task.angular_velocity < max_angular_vel_)
    {
      angular_vel_ = task.angular_velocity;
    }
    else
    {
      angular_vel_ = min_angular_vel_;
      ROS_WARN("[%s] Angular velocity out of bounds, setting default %f", name_.c_str(), angular_vel_);
    }
    if (task.linear_velocity > min_linear_vel_ && task.linear_velocity < max_linear_vel_)
    {
      linear_vel_ = task.linear_velocity;
    }
    else
    {
      linear_vel_ = min_linear_vel_;
      ROS_WARN("[%s] Linear velocity out of bounds, setting default %f", name_.c_str(), linear_vel_);
    }
    // If robot is already near starting major point
    at_start_point_ = false;
    if (payload.find("at_start_point") != payload.end())
    {
      at_start_point_ = payload["at_start_point"].get<bool>();
    }

    coords_for_nav_.clear();
    is_original_coords_for_nav_.clear();

    // Local vector (made to accomodate path generation service)
    std::vector<std::vector<float>> coords_for_nav;

    costmap_ptr_ = std::make_shared<costmap_2d::Costmap2DROS>("multi_point_map", tf_buffer_);

    if (p_recovery_behavior_enabled_)
      loadRecoveryBehaviors(nh_handler_);

    start_at_nearest_point_ = false;
    if (payload.find("start_at_nearest_point") != payload.end())
    {
      start_at_nearest_point_ = payload["start_at_nearest_point"].get<bool>();
    }

    // Generate all minor points
    if (pointsGen(rcvd_coords, coords_for_nav, true))
    {
      // Store coords in global
      coords_for_nav_ = coords_for_nav;
      if (coords_for_nav_.size() > 0)
      {
        ROS_INFO("[%s] Starting Multi-point navigation across %ld generated points", name_.c_str(),
                 coords_for_nav_.size());
        // Check which index to start with
        int start_at_idx = 0;

        if (start_at_nearest_point_)
        {
          // Build waypoints variable for getting nearest index
          std::vector<geometry_msgs::Pose> waypoints_vector;
          for (int i = 0; i < coords_for_nav_.size(); i++)
          {
            geometry_msgs::Pose temp_pose_holder;
            temp_pose_holder.position.x = coords_for_nav_[i][0];
            temp_pose_holder.position.y = coords_for_nav_[i][1];
            waypoints_vector.push_back(temp_pose_holder);
          }

          ros::Duration(0.2).sleep();  // give /pose time to update
          start_at_idx = movel_fms_utils::getNearestWaypointIdx(waypoints_vector, robot_pose_,
                                                                movel_fms_utils::DistMetric::EUCLIDEAN);
          ROS_WARN("[%s] NEAREST IDX %d", name_.c_str(), start_at_idx);

          if (start_at_idx > 0)
          {
            for (int i = 0; i < rcvd_multi_coords_.size(); i++)
            {
              if (major_indices_[i] <= start_at_idx)
              {
                rcvd_coords.erase(rcvd_coords.begin());
              }
            }
            rcvd_coords.insert(rcvd_coords.begin(), coords_for_nav_[start_at_idx]);
          }

          // Set start_at_nearest_point_ to false so that robot pose is added properly in pointsGen
          start_at_nearest_point_ = false;
          at_start_point_ = false;
          coords_for_nav.clear();
          if (pointsGen(rcvd_coords, coords_for_nav, true))
          {
            // Store coords in global
            coords_for_nav_ = coords_for_nav;
            if (coords_for_nav_.size() == 0)
            {
              // Cancel
              setMessage("2nd points generation call for start-at-nearest-point returned empty");
              error_message = message_;
              setTaskResult(false);
              recovery_behaviors_.clear();
              return code_;
            }
          }
          else
          {
            // Cancel
            setMessage("2nd points generation call for start-at-nearest-point failed");
            error_message = message_;
            setTaskResult(false);
            recovery_behaviors_.clear();
            return code_;
          }
        }

        coveragePercentage(coords_for_nav_);
        printGeneratedPath(rcvd_multi_coords_);
        for (int i = 0; i < coords_for_nav_.size(); ++i)
          is_original_coords_for_nav_.push_back(true);

        // Loop through generated points
        for (int i = 0; i < coords_for_nav_.size(); i++)
        {
          // Visualize on rviz
          visualizePath(i, false);

          // Publish current goal (mainly for FMS)
          publishCurrentGoal(i);
          current_idx_ = i - 1;

          // Call nav for instance point
          if (!navToPoint(i))
          {
            // If navigation was unsuccessful, cancel
            setMessage("Navigation to point unsuccessful");
            error_message = message_;
            setTaskResult(false);
            recovery_behaviors_.clear();
            return code_;
          }

          // check for visited original points
          int visited_rcvd_coord_idx = -1;
          for (int j = 0; j < rcvd_multi_coords_.size(); j++)
          {
            if (coords_for_nav_[i][0] == rcvd_multi_coords_[j][0] && coords_for_nav_[i][1] == rcvd_multi_coords_[j][1])
            {
              visited_rcvd_coord_idx = j;
              break;
            }
          }
          if (visited_rcvd_coord_idx > -1)
          {
            json handler_feedback(visited_rcvd_coord_idx);
            publishHandlerFeedback(handler_feedback);
          }
        }
        // Successful navigation
        ROS_INFO("[%s] Multi-point nav successfully completed", name_.c_str());
        outputMissedPts();
        unvisited_coords_.clear();
        n_init_unvisited_coords_ = 0;
        setTaskResult(true);
      }
      else
      {
        setMessage("Navigational coordinates vector empty");
        error_message = message_;
        setTaskResult(false);
      }
    }
    else
    {
      setMessage("Major points too close");
      error_message = message_;
      setTaskResult(false);
    }
  }
  else
  {
    setMessage("Malformed payload");
    error_message = message_;
    setTaskResult(false);
  }
  // Clean rviz topic
  visualizePath(0, true);
  stopRobot();
  recovery_behaviors_.clear();
  return code_;
}

/// Generating Path

bool MultiPointNavigationHandler::pointsGen(std::vector<std::vector<float>> rcvd_multi_coords,
                                            std::vector<std::vector<float>>& coords_for_nav, bool for_nav)
{
  // Time
  ros::Time starttime = ros::Time::now();

  std::vector<std::vector<float>> coords_for_spline;
  std::vector<std::vector<int>> points_to_spline;
  // Bypass degree kept separate to make sure uniformity throughout generation process in case reconfigured in middle
  bypass_degree_ = p_bypass_degree_;

  // Only if generating points for task (not service)
  if (for_nav && !start_at_nearest_point_)
  {
    if (at_start_point_)
    {
      // Robot already near starting point, can remove starting point from navigation
      ROS_INFO("[%s] Robot already near starting point (%.2f,%.2f), ignoring it", name_.c_str(),
               rcvd_multi_coords[0][0], rcvd_multi_coords[0][1]);
      rcvd_multi_coords.erase(rcvd_multi_coords.begin());
    }

    // Add current robot pose to the front of the major points list
    boost::shared_ptr<geometry_msgs::Pose const> shared_current_pose;
    shared_current_pose = ros::topic::waitForMessage<geometry_msgs::Pose>("/pose", ros::Duration(2.0));
    if (shared_current_pose != NULL)
    {
      std::vector<float> robot_pose_vec = { float(shared_current_pose->position.x),
                                            float(shared_current_pose->position.y) };
      rcvd_multi_coords.insert(rcvd_multi_coords.begin(), robot_pose_vec);
    }
    else
    {
      ROS_ERROR("[%s] Robot pose unavailable", name_.c_str());
      return false;
    }
  }

  // Will track indices of major points in the collection of all coords
  std::vector<int> major_indices{ 0 };

  // Loop through major points to generate minor (breadcrumb) points
  for (int i = 0; i < rcvd_multi_coords.size() - 1; i++)
  {
    // Check if 2 major points are the same
    if (rcvd_multi_coords[i][0] == rcvd_multi_coords[i + 1][0] &&
        rcvd_multi_coords[i][1] == rcvd_multi_coords[i + 1][1])
    {
      ROS_ERROR("[%s] 2 Major points with same coordinates (%.2f, %.2f)", name_.c_str(), rcvd_multi_coords[i][0],
                rcvd_multi_coords[i][1]);
      return false;
    }

    float slope;
    float maj_point_distance = std::sqrt(pow((rcvd_multi_coords[i + 1][0] - rcvd_multi_coords[i][0]), 2) +
                                         pow((rcvd_multi_coords[i + 1][1] - rcvd_multi_coords[i][1]), 2));
    float num_of_points = maj_point_distance / p_point_gen_dist_;

    if ((num_of_points - int(num_of_points)) * p_point_gen_dist_ < 0.1)
    {
      num_of_points--;
    }

    // debug
    // std::cout << "\nRCVD MULTI COORDS IDX " << i << ": num_of_points " << num_of_points << " int(num_of_points) " <<
    // int(num_of_points);

    major_indices.push_back(major_indices.back());

    if ((rcvd_multi_coords[i + 1][0] - rcvd_multi_coords[i][0]) != 0)
    {
      slope = (rcvd_multi_coords[i + 1][1] - rcvd_multi_coords[i][1]) /
              (rcvd_multi_coords[i + 1][0] - rcvd_multi_coords[i][0]);

      for (int j = 0; j <= int(num_of_points); j++)
      {
        std::vector<float> generated_min_point;
        if (rcvd_multi_coords[i][0] > rcvd_multi_coords[i + 1][0])
        {
          generated_min_point.push_back(rcvd_multi_coords[i][0] -
                                        ((p_point_gen_dist_ * j) * (sqrt(1 / (1 + pow(slope, 2))))));
        }
        else
        {
          generated_min_point.push_back(rcvd_multi_coords[i][0] +
                                        ((p_point_gen_dist_ * j) * (sqrt(1 / (1 + pow(slope, 2))))));
        }
        if (((rcvd_multi_coords[i][1] > rcvd_multi_coords[i + 1][1]) && slope > 0) ||
            ((rcvd_multi_coords[i][1] < rcvd_multi_coords[i + 1][1]) && slope < 0))
        {
          generated_min_point.push_back(rcvd_multi_coords[i][1] -
                                        ((p_point_gen_dist_ * j * slope) * (sqrt(1 / (1 + pow(slope, 2))))));
        }
        else if (((rcvd_multi_coords[i][1] < rcvd_multi_coords[i + 1][1]) && slope > 0) ||
                 ((rcvd_multi_coords[i][1] > rcvd_multi_coords[i + 1][1]) && slope < 0))
        {
          generated_min_point.push_back(rcvd_multi_coords[i][1] +
                                        ((p_point_gen_dist_ * j * slope) * (sqrt(1 / (1 + pow(slope, 2))))));
        }
        else
        {
          generated_min_point.push_back(rcvd_multi_coords[i][1]);
        }

        coords_for_spline.push_back(generated_min_point);
        major_indices.back() = major_indices.back() + 1;
      }
    }
    else
    {
      for (int j = 0; j <= int(num_of_points); j++)
      {
        std::vector<float> generated_min_point;

        generated_min_point.push_back(rcvd_multi_coords[i][0]);

        if (rcvd_multi_coords[i][1] > rcvd_multi_coords[i + 1][1])
        {
          generated_min_point.push_back(rcvd_multi_coords[i][1] - (p_point_gen_dist_ * j));
        }
        else if (rcvd_multi_coords[i][1] < rcvd_multi_coords[i + 1][1])
        {
          generated_min_point.push_back(rcvd_multi_coords[i][1] + (p_point_gen_dist_ * j));
        }
        else
        {
          generated_min_point.push_back(rcvd_multi_coords[i][1]);
        }
        coords_for_spline.push_back(generated_min_point);
        major_indices.back() = major_indices.back() + 1;
      }
    }
  }
  coords_for_spline.push_back(rcvd_multi_coords.back());

  // Check if spline/smoothening enabled
  if (p_spline_enable_)
  {
    if (rcvd_multi_coords.size() > 2 && coords_for_spline.size() > 0 &&
        getPointsToSpline(rcvd_multi_coords, major_indices, points_to_spline))
    {
      // Spline/smoothen points
      splinePoints(coords_for_spline, points_to_spline, coords_for_nav);
    }
    else
    {
      // No spline
      coords_for_nav = coords_for_spline;
    }
  }
  else
  {
    // No spline
    coords_for_nav = coords_for_spline;
  }

  // Time
  ros::Time endtime = ros::Time::now();
  ros::Duration time_taken = endtime - starttime;

  if (for_nav)
  {
    rcvd_multi_coords_ = rcvd_multi_coords;
    major_indices_ = major_indices;

    if (!start_at_nearest_point_)
    {
      ROS_INFO(
          "[%s] Info :\n Major points - %ld,\n Total nav points - %ld,\n Spline enable - %d,\n Obstacle check interval "
          "- %0.2f s,\n Obstacle timeout - %0.2f s,\n Look ahead points - %d,\n Gen. time - %f",
          name_.c_str(), rcvd_multi_coords.size(), coords_for_nav.size(), p_spline_enable_, obst_check_interval_,
          p_obstruction_timeout_, look_ahead_points_, time_taken.toSec());

      // Print generated nav points
      // printGeneratedPath(rcvd_multi_coords);
    }
  }

  return true;
}

void MultiPointNavigationHandler::splinePoints(std::vector<std::vector<float>>& coords_for_spline,
                                               std::vector<std::vector<int>> points_to_spline,
                                               std::vector<std::vector<float>>& coords_for_nav)
{
  coords_for_nav.clear();

  // Check if points_to_spline is not empty
  if (points_to_spline.size() > 0)
  {
    int index_pointer = 0;
    for (int i = 0; i < points_to_spline.size(); i++)
    {
      // insert into coords_for_nav, upto the indice_to_smoothen - bypass_degree. These are the unaffected points just
      // before spline
      for (int j = index_pointer; j <= (points_to_spline[i][0] - points_to_spline[i][1]); j++)
      {
        coords_for_nav.push_back(coords_for_spline[j]);
        index_pointer++;
      }

      // Make coordinate pairs to find intersections for spline
      std::vector<co_ord_pair> conn_points;
      co_ord_pair pC =
          std::make_pair(coords_for_spline[points_to_spline[i][0]][0], coords_for_spline[points_to_spline[i][0]][1]);
      conn_points.push_back(pC);
      for (int j = 1; j <= points_to_spline[i][1]; j++)
      {
        co_ord_pair temp_pCminus = std::make_pair(coords_for_spline[points_to_spline[i][0] - j][0],
                                                  coords_for_spline[points_to_spline[i][0] - j][1]);
        co_ord_pair temp_pCplus = std::make_pair(coords_for_spline[points_to_spline[i][0] + j][0],
                                                 coords_for_spline[points_to_spline[i][0] + j][1]);
        conn_points.insert(conn_points.begin(), temp_pCminus);
        conn_points.push_back(temp_pCplus);
      }

      // Find intersections for spline and push into coords for navigation
      int center_conn_point = points_to_spline[i][1];
      int lc1 = 0, lc2 = center_conn_point, lc3 = center_conn_point + 1;
      int lc4 = center_conn_point + 1, lc5 = 0, lc6 = 1;

      for (int j = 0; j < ((2 * points_to_spline[i][1]) - 1); j++)
      {
        if (j != 0)
        {
          if (j % 2 == 1)
          {
            lc1++;
            lc2++;
            lc3++;
          }
          else
          {
            lc4++;
            lc5++;
            lc6++;
          }
        }
        coords_for_nav.push_back(intersectPoint(conn_points[lc1], midPoint(conn_points[lc2], conn_points[lc3]),
                                                conn_points[lc4], midPoint(conn_points[lc5], conn_points[lc6])));
      }

      index_pointer = index_pointer + (2 * points_to_spline[i][1]) - 1;
    }
    for (int i = index_pointer; i < coords_for_spline.size(); i++)
    {
      coords_for_nav.push_back(coords_for_spline[i]);
      index_pointer++;
    }
  }

  // If empty, do not spline
  else
  {
    ROS_WARN("[%s] Points to spline was empty, aborting spline", name_.c_str());
    coords_for_nav = coords_for_spline;
  }
}

bool MultiPointNavigationHandler::getPointsToSpline(std::vector<std::vector<float>> rcvd_multi_coords,
                                                    std::vector<int> major_indices,
                                                    std::vector<std::vector<int>>& points_to_spline)
{
  points_to_spline.clear();

  for (int i = 1; i < major_indices.size() - 1; i++)
  {
    // Check if straight line
    bool slope1exists = false, slope2exists = false;
    float slope1, slope2;
    if ((rcvd_multi_coords[i][0] - rcvd_multi_coords[i - 1][0]) != 0)
    {
      // Slope 1 exists
      slope1 = (rcvd_multi_coords[i][1] - rcvd_multi_coords[i - 1][1]) /
               (rcvd_multi_coords[i][0] - rcvd_multi_coords[i - 1][0]);
      slope1exists = true;
    }
    if ((rcvd_multi_coords[i + 1][0] - rcvd_multi_coords[i][0]) != 0)
    {
      // Slope 2 exists
      slope2 = (rcvd_multi_coords[i + 1][1] - rcvd_multi_coords[i][1]) /
               (rcvd_multi_coords[i + 1][0] - rcvd_multi_coords[i][0]);
      slope2exists = true;
    }

    if ((slope1exists != slope2exists) || (slope1exists && slope2exists && (slope1 != slope2)))
    {
      // Check if enough points are there between major points, to accomodate bypass_degree
      if (((major_indices[i] - major_indices[i - 1]) >= (bypass_degree_ * 2)) &&
          ((major_indices[i + 1] - major_indices[i]) >= (bypass_degree_ * 2)))
      {
        int point_bypass_degree = bypass_degree_;
        std::vector<int> temp_vector;
        temp_vector.push_back(major_indices[i]);
        temp_vector.push_back(point_bypass_degree);
        points_to_spline.push_back(temp_vector);
      }
      // If lesser points are available, make do with with respectively lesser bypass_degree
      // Check which side of the major_index is shorter and use that to calc possible bypass_degree
      else if (((major_indices[i] - major_indices[i - 1]) <= (major_indices[i + 1] - major_indices[i])) &&
               ((major_indices[i] - major_indices[i - 1]) >= 2))
      {
        int point_bypass_degree;
        if ((major_indices[i] - major_indices[i - 1]) < 4)
        {
          point_bypass_degree = 1;
        }
        else
        {
          point_bypass_degree = int((major_indices[i] - major_indices[i - 1]) / 2);
        }
        std::vector<int> temp_vector;
        temp_vector.push_back(major_indices[i]);
        temp_vector.push_back(point_bypass_degree);
        points_to_spline.push_back(temp_vector);
      }
      else if (((major_indices[i + 1] - major_indices[i]) < (major_indices[i] - major_indices[i - 1])) &&
               ((major_indices[i + 1] - major_indices[i]) >= 2))
      {
        int point_bypass_degree;
        if ((major_indices[i + 1] - major_indices[i]) < 4)
        {
          point_bypass_degree = 1;
        }
        else
        {
          point_bypass_degree = int((major_indices[i + 1] - major_indices[i]) / 2);
        }
        std::vector<int> temp_vector;
        temp_vector.push_back(major_indices[i]);
        temp_vector.push_back(point_bypass_degree);
        points_to_spline.push_back(temp_vector);
      }
    }
  }

  if (points_to_spline.size() == 0)
  {
    return false;
  }
  return true;
}

co_ord_pair MultiPointNavigationHandler::midPoint(co_ord_pair P1, co_ord_pair P2)
{
  co_ord_pair mid_point = std::make_pair((P1.first + P2.first) / 2, (P1.second + P2.second) / 2);
  return mid_point;
}

std::vector<float> MultiPointNavigationHandler::intersectPoint(co_ord_pair line1A, co_ord_pair line1B,
                                                               co_ord_pair line2C, co_ord_pair line2D)
{
  // Line 1 AB represented as a1x + b1y = c1
  float a1 = line1B.second - line1A.second;
  float b1 = line1A.first - line1B.first;
  float c1 = a1 * line1A.first + b1 * line1A.second;

  // Line 2 CD represented as a2x + b2y = c2
  float a2 = line2D.second - line2C.second;
  float b2 = line2C.first - line2D.first;
  float c2 = a2 * line2C.first + b2 * line2C.second;

  float determinant = a1 * b2 - a2 * b1;

  std::vector<float> return_vec;

  // Push X value of intersecting point
  return_vec.push_back((b2 * c1 - b1 * c2) / determinant);
  // Push Y value of intersecting point
  return_vec.push_back((a1 * c2 - a2 * c1) / determinant);

  return return_vec;
}

///////-----///////

/// Visualize, topics, service and config

void MultiPointNavigationHandler::visualizePath(int point_index, bool delete_all)
{
  int marker_action = 0;
  if (delete_all)
  {
    point_index = 0;
    marker_action = 2;
  }
  visualization_msgs::MarkerArray marker_array;

  // markers[0] is to visualize Major points - Sphere
  visualization_msgs::Marker major_marker;
  geometry_msgs::Vector3 sphere_scale;
  std_msgs::ColorRGBA sphere_color;
  sphere_color.r = 1;
  sphere_color.g = 0;
  sphere_color.b = 0;
  sphere_color.a = 1;
  sphere_scale.x = 0.07;
  sphere_scale.y = 0.07;
  sphere_scale.z = 0.07;
  major_marker.header.frame_id = "map";
  major_marker.header.stamp = ros::Time();
  major_marker.id = 0;
  major_marker.type = 7;
  major_marker.action = marker_action;
  major_marker.pose.orientation.w = 1.0;
  major_marker.scale = sphere_scale;
  major_marker.color = sphere_color;

  for (int i = 0; i < rcvd_multi_coords_.size(); i++)
  {
    if (point_index <= major_indices_[i])
    {
      geometry_msgs::Point mark_pose;
      mark_pose.x = rcvd_multi_coords_[i][0];
      mark_pose.y = rcvd_multi_coords_[i][1];
      major_marker.points.push_back(mark_pose);
    }
  }
  marker_array.markers.push_back(major_marker);
  
  // markers[1] is to visualize generated path - Sphere
  visualization_msgs::Marker original_path_marker;
  geometry_msgs::Vector3 line_scale;
  std_msgs::ColorRGBA line_color;
  line_color.r = 1;
  line_color.g = 1;
  line_color.b = 0;
  // line_color.a = 0.5;
  line_color.a = 1;
  // line_scale.x = 0.01;
  line_scale.x = 0.05;
  line_scale.y = 0.05;
  line_scale.z = 0.05;
  original_path_marker.header.frame_id = "map";
  original_path_marker.header.stamp = ros::Time();
  original_path_marker.id = 1;
  original_path_marker.type = 7;
  original_path_marker.action = marker_action;
  original_path_marker.pose.orientation.w = 1.0;
  original_path_marker.scale = line_scale;
  original_path_marker.color = line_color;

  for (int i = 0; i < coords_for_nav_.size(); i++)
  {
    if (is_original_coords_for_nav_[i])
    {
      geometry_msgs::Point mark_pose;
      mark_pose.x = coords_for_nav_[i][0];
      mark_pose.y = coords_for_nav_[i][1];
      original_path_marker.points.push_back(mark_pose);
    }
  }
  if (original_path_marker.points.size() > 1)
  {
    marker_array.markers.push_back(original_path_marker);
  }
  else
  {
    original_path_marker.action = 2;
    marker_array.markers.push_back(original_path_marker);
  }

  // markers[2] is to visualize adjusted path segment - Sphere
  visualization_msgs::Marker new_segment_marker;
  // geometry_msgs::Vector3 line_scale;
  // std_msgs::ColorRGBA line_color;
  line_color.r = 0;
  line_color.g = 0;
  line_color.b = 1;
  // line_color.a = 0.5;
  line_color.a = 1;
  // line_scale.x = 0.01;
  line_scale.x = 0.05;
  line_scale.y = 0.05;
  line_scale.z = 0.05;
  new_segment_marker.header.frame_id = "map";
  new_segment_marker.header.stamp = ros::Time();
  new_segment_marker.id = 2;
  new_segment_marker.type = 7;
  new_segment_marker.action = marker_action;
  new_segment_marker.pose.orientation.w = 1.0;
  new_segment_marker.scale = line_scale;
  new_segment_marker.color = line_color;

  for (int i = 0; i < coords_for_nav_.size(); i++)
  {
    if (!is_original_coords_for_nav_[i])
    {
      geometry_msgs::Point mark_pose;
      mark_pose.x = coords_for_nav_[i][0];
      mark_pose.y = coords_for_nav_[i][1];
      new_segment_marker.points.push_back(mark_pose);
    }
  }
  if (new_segment_marker.points.size() > 1)
  {
    marker_array.markers.push_back(new_segment_marker);
  }
  else
  {
    new_segment_marker.action = 2;
    marker_array.markers.push_back(new_segment_marker);
  }

  // markers[2] is to visualize current goal point - Sphere
  /*
  visualization_msgs::Marker current_marker;
  sphere_color.r = 0;
  sphere_color.g = 0;
  sphere_color.b = 1;
  sphere_color.a = 1;
  sphere_scale.x = 0.08;
  sphere_scale.y = 0.08;
  sphere_scale.z = 0.08;
  current_marker.header.frame_id = "map";
  current_marker.header.stamp = ros::Time();
  current_marker.id = 2;
  current_marker.type = 7;
  current_marker.action = marker_action;
  current_marker.pose.orientation.w = 1.0;
  current_marker.scale = sphere_scale;
  current_marker.color = sphere_color;

  geometry_msgs::Point mark_pose;
  mark_pose.x = coords_for_nav_[point_index][0];
  mark_pose.y = coords_for_nav_[point_index][1];
  current_marker.points.push_back(mark_pose);
  marker_array.markers.push_back(current_marker);
  */

  // markers[3] is to visualize obstacle check look ahead path - Line strip
  visualization_msgs::Marker look_ahead_marker;
  line_color.r = 0;
  line_color.g = 1;
  line_color.b = 0;
  line_color.a = 1;
  line_scale.x = 0.02;
  look_ahead_marker.header.frame_id = "map";
  look_ahead_marker.header.stamp = ros::Time();
  look_ahead_marker.id = 3;
  look_ahead_marker.type = 4;
  look_ahead_marker.action = marker_action;
  look_ahead_marker.pose.orientation.w = 1.0;
  look_ahead_marker.scale = line_scale;
  look_ahead_marker.color = line_color;

  int max_i_count = look_ahead_points_;

  if (point_index > coords_for_nav_.size() - 1)
  {
    max_i_count = 1;
  }
  else if (point_index > coords_for_nav_.size() - max_i_count)
  {
    max_i_count = coords_for_nav_.size() - point_index;
  }

  for (int i = 0; i < max_i_count; i++)
  {
    geometry_msgs::Point mark_pose;
    mark_pose.x = coords_for_nav_[point_index + i][0];
    mark_pose.y = coords_for_nav_[point_index + i][1];
    look_ahead_marker.points.push_back(mark_pose);
  }
  marker_array.markers.push_back(look_ahead_marker);

  // markers[4] is to visualize tolerance for current goal - Flat cylinder
  visualization_msgs::Marker goal_tolerance_marker;
  geometry_msgs::Vector3 cylinder_scale;
  std_msgs::ColorRGBA cylinder_color;
  cylinder_color.r = 0;
  cylinder_color.g = 0;
  cylinder_color.b = 1;
  cylinder_color.a = 0.3;
  cylinder_scale.x = p_goal_tolerance_;
  cylinder_scale.y = p_goal_tolerance_;
  cylinder_scale.z = 0.02;
  goal_tolerance_marker.header.frame_id = "map";
  goal_tolerance_marker.header.stamp = ros::Time();
  goal_tolerance_marker.id = 4;
  goal_tolerance_marker.type = 3;
  goal_tolerance_marker.action = marker_action;
  goal_tolerance_marker.pose.position.x = coords_for_nav_[point_index][0];
  goal_tolerance_marker.pose.position.y = coords_for_nav_[point_index][1];
  goal_tolerance_marker.pose.orientation.w = 1.0;
  goal_tolerance_marker.scale = cylinder_scale;
  goal_tolerance_marker.color = cylinder_color;

  marker_array.markers.push_back(goal_tolerance_marker);

  // markers[0] is to visualize Major points - Sphere
  visualization_msgs::Marker minor_marker;
  geometry_msgs::Vector3 marker_scale;
  std_msgs::ColorRGBA marker_color;
  marker_color.r = 1;
  marker_color.g = 1;
  marker_color.b = 0;
  marker_color.a = 1;
  marker_scale.x = 0.07;
  marker_scale.y = 0.07;
  marker_scale.z = 0.07;
  minor_marker.header.frame_id = "map";
  minor_marker.header.stamp = ros::Time();
  minor_marker.id = 0;
  minor_marker.type = 7;
  minor_marker.action = marker_action;
  minor_marker.pose.orientation.w = 1.0;
  minor_marker.scale = marker_scale;
  minor_marker.color = marker_color;

  for(int i = 0; i < coords_for_nav_.size(); i++){
    if(point_index <= major_indices_[i]){
      geometry_msgs::Point mark_pose;
      mark_pose.x = coords_for_nav_[i][0];
      mark_pose.y = coords_for_nav_[i][1];
      minor_marker.points.push_back(mark_pose);
    }
  }

  marker_array.markers.push_back(minor_marker);

  path_visualize_pub_.publish(marker_array);
}

void MultiPointNavigationHandler::printGeneratedPath(std::vector<std::vector<float>> rcvd_multi_coords)
{
  // Only for debugging
  std::cout << std::endl << "Major points : \n";
  for (int i = 0; i < rcvd_multi_coords.size(); i++)
  {
    std::cout << "⦿ [" << rcvd_multi_coords[i][0] << ", " << rcvd_multi_coords[i][1] << "]" << std::endl;
  }

  std::cout << std::endl
            << "Generated points : \n"
            << "COORDS FOR NAV SIZE: " << coords_for_nav_.size() << std::endl;
  for (int i = 0; i < coords_for_nav_.size(); i++)
  {
    std::cout << "• [" << coords_for_nav_[i][0] << ", " << coords_for_nav_[i][1] << "]" << std::endl;
  }
}

void MultiPointNavigationHandler::publishCurrentGoal(int nav_coords_index)
{
  if (nav_coords_index != 0)
  {
    for (int i = 0; i < major_indices_.size(); i++)
    {
      if (nav_coords_index <= major_indices_[i])
      {
        movel_seirios_msgs::MultipointProgress to_publish;

        to_publish.from_major_index = i - 1;
        to_publish.to_major_index = i;

        to_publish.from_minor_index = nav_coords_index - 1;
        to_publish.to_minor_index = nav_coords_index;

        to_publish.from_major_pose.position.x = rcvd_multi_coords_[i - 1][0];
        to_publish.from_major_pose.position.y = rcvd_multi_coords_[i - 1][1];

        to_publish.to_major_pose.position.x = rcvd_multi_coords_[i][0];
        to_publish.to_major_pose.position.y = rcvd_multi_coords_[i][1];

        for (int j = 0; j < rcvd_multi_coords_.size(); j++)
        {
          geometry_msgs::Pose instance_pose;
          instance_pose.position.x = rcvd_multi_coords_[j][0];
          instance_pose.position.y = rcvd_multi_coords_[j][1];

          to_publish.major_points_path.push_back(instance_pose);
        }
        current_goal_pub_.publish(to_publish);
        break;
      }
    }
  }
}

bool MultiPointNavigationHandler::pathServiceCb(movel_seirios_msgs::MultipointPath::Request& req,
                                                movel_seirios_msgs::MultipointPath::Response& res)
{
  if (req.major_points.size() > 1)
  {
    // Call points gen for visualization/UI
    std::vector<std::vector<float>> rcvd_srv_coords;
    for (auto& elem : req.major_points)
    {
      std::vector<float> coord_instance;
      coord_instance.push_back(elem.position.x);
      coord_instance.push_back(elem.position.y);
      rcvd_srv_coords.push_back(coord_instance);
    }

    // Will store generated path, passed reference
    std::vector<std::vector<float>> coords_for_nav;

    if (pointsGen(rcvd_srv_coords, coords_for_nav, false))
    {
      for (auto& elem : coords_for_nav)
      {
        geometry_msgs::Point coord_point;
        coord_point.x = elem[0];
        coord_point.y = elem[1];
        res.generated_path.push_back(coord_point);
      }
      res.result = "Successful";
    }
    else
    {
      res.result = "Failure in path generation";
    }
  }
  else
  {
    res.result = "Failure, not enough major points";
  }

  return true;
}

void MultiPointNavigationHandler::robotPoseCB(const geometry_msgs::Pose::ConstPtr& msg)
{
  if (task_active_)
  {
    robot_pose_ = *msg;
  }

  // Publish coverage percentage
  double x = msg->position.x;
  double y = msg->position.y;

  for (int i = 0; i < unvisited_coords_.size(); ++i)
  {
    double dx = x - unvisited_coords_[i][0];
    double dy = y - unvisited_coords_[i][1];
    if (hypot(dx, dy) <= p_goal_tolerance_)
    {
      unvisited_coords_.erase(unvisited_coords_.begin() + i);
      // debug
      // ROS_INFO("Point reached %i coordinates: (%f,%f)", i, unvisited_coords_[i][0],unvisited_coords_[i][1]);// 1
      // index
      break;
    }
  }

  if (n_init_unvisited_coords_ != 0)
  {
    area_percentage_.data = 1.0 - ((float)unvisited_coords_.size() / (float)n_init_unvisited_coords_);
    coverage_percentage_pub_.publish(area_percentage_);
  }
}

void MultiPointNavigationHandler::reconfCB(multi_point::MultipointConfig& config, uint32_t level)
{
  ROS_INFO("[%s] Reconfigure Request: %f %f %f %f %f %f %f %f %s %f %s %f %f %d %f %f %s %s",
            name_.c_str(), 
            config.points_distance, config.look_ahead_distance, 
            config.obst_check_freq, config.goal_tolerance,
            config.angular_tolerance, config.kp,
            config.ki, config.kd,
            config.spline_enable ? "True" : "False",
            config.obstacle_timeout,
            config.forward_only ? "True" : "False",
            config.max_linear_acc, config.max_angular_acc,
            config.max_spline_bypass_degree, config.slow_curve_vel, config.slow_curve_scale,
            config.slow_at_curve_enable ? "True" : "False",
            config.slow_at_points_enable ? "True" : "False");

  p_point_gen_dist_ = config.points_distance;
  p_look_ahead_dist_ = config.look_ahead_distance;
  p_obst_check_freq_ = config.obst_check_freq;
  p_goal_tolerance_ = config.goal_tolerance;
  p_angular_tolerance_ = config.angular_tolerance;
  p_kp_ = config.kp;
  p_ki_ = config.ki;
  p_kd_ = config.kd;
  p_spline_enable_ = config.spline_enable;
  p_obstruction_timeout_ = config.obstacle_timeout;
  p_forward_only_ = config.forward_only;
  p_linear_acc_ = config.max_linear_acc;
  p_linear_dacc_ = config.max_linear_dacc;
  p_angular_acc_ = config.max_angular_acc;
  p_bypass_degree_ = config.max_spline_bypass_degree;
  p_curve_vel_ = config.slow_curve_vel;
  p_curve_scale_ = config.slow_curve_scale;
  p_slow_curve_enable_ = config.slow_at_curve_enable;
  p_slow_points_enable_ = config.slow_at_points_enable;
}

///////-----///////

/// Navigation

bool MultiPointNavigationHandler::navToPoint(int instance_index)
{
  ros::Time obs_start_time;
  bool obs_timeout_started = false;
  obstructed_ = obstacleCheck(instance_index);
  ros::Time prev_check_time = ros::Time::now();
  end_in_horizon_ = isEndInHorizon(instance_index);

  // debug
  // ROS_WARN("Moving to : %i", instance_index);

  // If robot pose not within tolerance, point towards it
  while (std::sqrt(pow((robot_pose_.position.x - coords_for_nav_[instance_index][0]), 2) +
                   pow((robot_pose_.position.y - coords_for_nav_[instance_index][1]), 2)) > p_goal_tolerance_)
  {
    if (task_cancelled_)
    {
      stopRobot();
      ROS_ERROR("[%s] Published stop command", name_.c_str());
      return false;
    }

    // Obstruction timeout check
    if (!obstructed_ && obs_timeout_started)
    {
      obs_timeout_started = false;
    }
    if (obstructed_ && !obs_timeout_started)
    {
      obs_timeout_started = true;
      obs_start_time = ros::Time::now();
    }
    if (obstructed_ && obs_timeout_started &&
        (ros::Time::now() - obs_start_time > ros::Duration(p_obstruction_timeout_)) && stopAtObstacleEnabled())
    {
      if (p_recovery_behavior_enabled_ && recovery_index_ < recovery_behaviors_.size())
      {
        ROS_INFO("[%s] Executing recovery_behavior %d", name_.c_str(), recovery_index_);
        recovery_behaviors_[recovery_index_]->runBehavior();

        // we'll check if the recovery behavior actually worked
        obs_timeout_started = false;
        obstructed_ = obstacleCheck(instance_index);
        ros::Time prev_check_time = ros::Time::now();
        if (obstructed_)
        {
          obs_timeout_started = true;
          obs_start_time = ros::Time::now();
        }

        // update the index of the next recovery behavior that we'll try
        recovery_index_++;
      }
      else
      {
        ROS_ERROR("[%s] Obstruction time out reached. Cancelling task", name_.c_str());
        return false;
      }
    }
    // Get angle of robot with instance point goal
    float angle_to_point = std::atan2((coords_for_nav_[instance_index][1] - robot_pose_.position.y),
                                      (coords_for_nav_[instance_index][0] - robot_pose_.position.x));

    // Get robot orientation theta
    tf::Quaternion q(robot_pose_.orientation.x, robot_pose_.orientation.y, robot_pose_.orientation.z,
                     robot_pose_.orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, theta;
    m.getRPY(roll, pitch, theta);

    geometry_msgs::Twist to_cmd_vel;
    float dtheta = angle_to_point - theta;

    if (dtheta > M_PI)
    {
      dtheta = dtheta - (2 * M_PI);
    }
    if (dtheta < -M_PI)
    {
      dtheta = dtheta + (2 * M_PI);
    }

    obst_check_interval_ = 1 / p_obst_check_freq_;

    // Update obstruction status
    if (ros::Time::now() - prev_check_time >= ros::Duration(obst_check_interval_))
    {
      // clearCostmapFn();
      obstructed_ = obstacleCheck(instance_index);
      prev_check_time = ros::Time::now();
      if (obstructed_)
      {
        ROS_WARN("[%s] Robot Obstructed", name_.c_str());
      }
    }

    if (obstructed_ && !isTaskPaused() && !stopAtObstacleEnabled())
    {
      ROS_INFO("[%s] Obstacle blockage at: %i", name_.c_str(), instance_index);
      adjustPlanForObstacles(coords_for_nav_);
      
      // Update with the new path plan
      obstructed_ = obstacleCheck(instance_index);
      end_in_horizon_ = isEndInHorizon(instance_index);
      visualizePath(instance_index, false);
    }

    // To slow down X points before the reaching a major point
    float allowed_linear_vel = linear_vel_;
    if(p_slow_points_enable_)
    {
      for(int j = 0; j < major_indices_.size(); j++)
      {
        // If curve velocity is higher than task linear vel, keep task linear vel
        if(instance_index < major_indices_[j] && p_curve_vel_ < linear_vel_)
        {
          if(major_indices_[j] - instance_index <= 3)
          {
            allowed_linear_vel = p_curve_vel_;
            break;
          }
        }
      }
    }

    // Handle curve deceleration
    if(p_slow_curve_enable_){
      if (instance_index>0 && instance_index < coords_for_nav_.size()-2){
        static float prev_scaling_theta = 0;
        static float prev_instance_index = 0;
        float scaling_theta;
        float a_x = coords_for_nav_[instance_index-1][0];
        float b_x = coords_for_nav_[instance_index][0];
        float c_x = coords_for_nav_[instance_index+1][0];

        float a_y = coords_for_nav_[instance_index-1][1];
        float b_y = coords_for_nav_[instance_index][1];
        float c_y = coords_for_nav_[instance_index+1][1];

        float abc_theta = std::fabs(std::atan2(c_y-b_y, c_x-b_x) - std::atan2(a_y-b_y, a_x-b_x)) * 180 / M_PI;
        scaling_theta = std::fabs(180-abc_theta);
        scaling_theta = (scaling_theta / p_curve_scale_) + 1;

        //If the robot just curved, so the robot dont accelerate suddenly right after curving
        if (scaling_theta <= 1.5 && instance_index-prev_instance_index == 1){
          scaling_theta = prev_scaling_theta;
        }
        else{
          prev_scaling_theta = scaling_theta;
          prev_instance_index = instance_index;
        }
        
        if (scaling_theta !=0) allowed_linear_vel = allowed_linear_vel / scaling_theta;
        
        // ROS_INFO_THROTTLE(1, "Scaling theta : %f, allowed linear vel: %f", scaling_theta, allowed_linear_vel);
      }
      else if (instance_index >= coords_for_nav_.size() - 2 && p_curve_vel_ < linear_vel_){
        // Slow down to last point, only if curve velocity is lower than task linear vel
        // Slow down to last point
        allowed_linear_vel = p_curve_vel_;
      }
    }


    // Nav cmd velocity if:
    //  - not paused and not obstructed, or
    //  - not paused and obstructed but not stop at obstacle 
    if (!isTaskPaused() && (!obstructed_ || (obstructed_ && !stopAtObstacleEnabled())))
    {
      // std::cout << "I KEEP ON MOVING" << std::endl;
      if (std::abs(dtheta) > angular_tolerance_)
      {
        if ((std::abs(dtheta) > M_PI - angular_tolerance_) && (std::abs(dtheta) < M_PI + angular_tolerance_) &&
            !p_forward_only_)
        {
          to_cmd_vel.linear.x = linAccelerationCheck(-allowed_linear_vel);
          to_cmd_vel.angular.z = angAccelerationCheck(-pidFn(dtheta, 0));
        }
        else
        {
          to_cmd_vel.linear.x = linAccelerationCheck(0.0);
          to_cmd_vel.angular.z = angAccelerationCheck(pidFn(dtheta, 0));
        }
      }
      else
      {
        to_cmd_vel.linear.x = linAccelerationCheck(allowed_linear_vel);
        to_cmd_vel.angular.z = angAccelerationCheck(pidFn(dtheta, 0));
      }
    }
    else
    {
      // std::cout << "I'M STOPPING" << std::endl;
      stopRobot();
    }

    // Publish cmd_vel
    // std::cout << "Publishing cmd vel:\n" << to_cmd_vel << std::endl;
    cmd_vel_pub_.publish(to_cmd_vel);
  }

  return true;
}

float MultiPointNavigationHandler::pidFn(float dtheta, float set_point)
{
  static float prev_value = 0;
  static float i_err = 0;

  float error = set_point - dtheta;
  float pTerm = p_kp_ * error;

  static float iTerm = 0;
  iTerm += p_ki_ * error;

  float dTerm = p_kd_ * (dtheta - prev_value);
  prev_value = dtheta;

  float return_val = pTerm + dTerm;

  if (return_val > angular_vel_)
  {
    return_val = angular_vel_;
  }
  else if (return_val < -angular_vel_)
  {
    return_val = -angular_vel_;
  }

  return return_val;
}

bool MultiPointNavigationHandler::obstacleCheck(int nav_coords_index)
{
  // Check if planned points exist
  if (coords_for_nav_.size() == 0)
  {
    ROS_ERROR("[%s] Navigation Co-ords vector empty", name_.c_str());
    return true;
  }
  else if (nav_coords_index >= coords_for_nav_.size())
  {
    ROS_ERROR("[%s] Obstacle check failed, index invalid", name_.c_str());
    return true;
  }

  // debug
  // ROS_WARN("obstacle check at : %i", nav_coords_index);

  enum ObstructionType
  {
    LETHAL,
    INSCRIBED_INFLATED
  };
  ObstructionType obstruction_type = LETHAL;

  costmap_2d::Costmap2D* sync_costmap = costmap_ptr_->getCostmap();

  look_ahead_points_ = int(p_look_ahead_dist_ / p_point_gen_dist_);
  // Look ahead to atleast 2 points
  if (look_ahead_points_ < 2)
  {
    look_ahead_points_ = 2;
  }

  int max_i_count = look_ahead_points_;

  if (nav_coords_index > coords_for_nav_.size() - 1)
  {
    max_i_count = 1;
  }
  else if (nav_coords_index > coords_for_nav_.size() - max_i_count)
  {
    max_i_count = coords_for_nav_.size() - nav_coords_index;
  }

  // Check for obstacles on the points ahead
  for (int i = 0; i < max_i_count; i++)
  {
    unsigned int mx, my;
    double wx, wy;

    wx = coords_for_nav_[nav_coords_index + i][0];
    wy = coords_for_nav_[nav_coords_index + i][1];

    if (sync_costmap->worldToMap(wx, wy, mx, my))
    {
      unsigned char cost_i = sync_costmap->getCost(mx, my);
      if (cost_i == costmap_2d::INSCRIBED_INFLATED_OBSTACLE)
      {
        ROS_ERROR("[%s] Found obstruction on point %i - Inscribed inflation", name_.c_str(), nav_coords_index + i);
        blocked_idx_ = nav_coords_index;
        blocked_trigger_idx_ = nav_coords_index + i;
        obstruction_type = INSCRIBED_INFLATED;
        return true;
      }
      if (cost_i == costmap_2d::LETHAL_OBSTACLE)
      {
        ROS_ERROR("[%s] Found obstruction on point %i - Lethal obstacle", name_.c_str(), nav_coords_index + i);
        blocked_idx_ = nav_coords_index;
        blocked_trigger_idx_ = nav_coords_index + i;
        obstruction_type = LETHAL;
        return true;
      }
      if (cost_i == costmap_2d::NO_INFORMATION)
      {
        ROS_ERROR("[%s] Found obstruction on point %i - No Information", name_.c_str(), nav_coords_index + i);
        blocked_idx_ = nav_coords_index;
        blocked_trigger_idx_ = nav_coords_index + i;
        return true;
      }
    }
    // Out of bounds
    else
    {
      ROS_ERROR("[%s] Out of bounds", name_.c_str());
      obstruction_type = LETHAL;
      return true;
    }

    // Also check the middle co-ordinates between nav points
    if (!(nav_coords_index == 0 && i == 0))
    {
      unsigned int mx_mid, my_mid;
      double wx_mid, wy_mid;

      co_ord_pair nav_coord_1 =
          std::make_pair(coords_for_nav_[nav_coords_index + i - 1][0], coords_for_nav_[nav_coords_index + i - 1][1]);
      co_ord_pair nav_coord_2 =
          std::make_pair(coords_for_nav_[nav_coords_index + i][0], coords_for_nav_[nav_coords_index + i][1]);

      co_ord_pair mid_nav_coord = midPoint(nav_coord_1, nav_coord_2);

      wx_mid = mid_nav_coord.first;
      wy_mid = mid_nav_coord.second;

      if (sync_costmap->worldToMap(wx_mid, wy_mid, mx_mid, my_mid))
      {
        unsigned char cost_i = sync_costmap->getCost(mx_mid, my_mid);
        if (cost_i == costmap_2d::INSCRIBED_INFLATED_OBSTACLE)
        {
          ROS_ERROR("[%s] Found obstruction between points %i and %i - Inscribed inflation", name_.c_str(),
                    nav_coords_index + i - 1, nav_coords_index + i);
          blocked_idx_ = nav_coords_index;
          blocked_trigger_idx_ = nav_coords_index + i;
          obstruction_type = INSCRIBED_INFLATED;
          return true;
        }
        if (cost_i == costmap_2d::LETHAL_OBSTACLE)
        {
          ROS_ERROR("[%s] Found obstruction between points %i and %i - Lethal obstacle", name_.c_str(),
                    nav_coords_index + i - 1, nav_coords_index + i);
          blocked_idx_ = nav_coords_index;
          blocked_trigger_idx_ = nav_coords_index + i;
          obstruction_type = LETHAL;
          return true;
        }
        if (cost_i == costmap_2d::NO_INFORMATION)
        {
          ROS_ERROR("[%s] Found obstruction between points %i and %i - No Information", name_.c_str(),
                    nav_coords_index + i - 1, nav_coords_index + i);
          blocked_idx_ = nav_coords_index;
          blocked_trigger_idx_ = nav_coords_index + i;
          return true;
        }
      }
      // Out of bounds
      else
      {
        ROS_ERROR("[%s] Out of bounds", name_.c_str());
        obstruction_type = LETHAL;
        return true;
      }
    }
  }
  // Only for debugging
  // ROS_INFO("[%s] Path is clear for index %d(out of %ld)", name_.c_str(), nav_coords_index,coords_for_nav_.size());
  return false;
}

bool MultiPointNavigationHandler::obstacleCheckSinglePoint(int nav_coords_index)
{
  // Check if planned points exist
  if (coords_for_nav_.size() == 0)
  {
    ROS_ERROR("[%s] Navigation co-ords vector empty", name_.c_str());
    return true;
  }
  else if (nav_coords_index >= coords_for_nav_.size())
  {
    ROS_ERROR("[%s] Obstacle check failed, index invalid", name_.c_str());
    return true;
  }

  costmap_2d::Costmap2D* sync_costmap = costmap_ptr_->getCostmap();

  // Check for obstacles on the given point
  unsigned int mx, my;
  double wx, wy;

  wx = coords_for_nav_[nav_coords_index][0];
  wy = coords_for_nav_[nav_coords_index][1];

  if (sync_costmap->worldToMap(wx, wy, mx, my))
  {
    unsigned char cost_i = sync_costmap->getCost(mx, my);
    if (cost_i == costmap_2d::INSCRIBED_INFLATED_OBSTACLE)
    {
      ROS_ERROR("[%s] Found obstruction on point %i - Inscribed inflation", name_.c_str(), nav_coords_index);
      return true;
    }
    if (cost_i == costmap_2d::LETHAL_OBSTACLE)
    {
      ROS_ERROR("[%s] Found obstruction on point %i - Lethal obstacle", name_.c_str(), nav_coords_index);
      return true;
    }
    if (cost_i == costmap_2d::NO_INFORMATION)
    {
      ROS_ERROR("[%s] Found obstruction on point %i - No Information", name_.c_str(), nav_coords_index);
      return true;
    }
  }
  else
  {
    ROS_ERROR("[%s] Out of bounds", name_.c_str());
    return true;
  }
  
  return false;
}

bool MultiPointNavigationHandler::isEndInHorizon(int current_point_index)
{
  if (coords_for_nav_.size() - (current_point_index + 1) <= look_ahead_points_)
  {
    return true;
  }
  else
  {
    return false;
  }
}

float MultiPointNavigationHandler::linAccelerationCheck(float req_lin_vel)
{
  static float prev_lin_vel = 0.0;
  static ros::Time prev_time = ros::Time::now();
  float allowed_lin_vel;

  // Experimental deceleration feature, only enabled if forward_only is enabled
  if (p_forward_only_){
    if( req_lin_vel - prev_lin_vel > (p_linear_acc_)*((ros::Time::now()-prev_time).toSec()) || req_lin_vel - prev_lin_vel < (-p_linear_dacc_)*((ros::Time::now()-prev_time).toSec()) ){
      if(req_lin_vel - prev_lin_vel < 0.0){
        allowed_lin_vel = prev_lin_vel - (p_linear_dacc_)*((ros::Time::now()-prev_time).toSec());
        // ROS_INFO_THROTTLE(1,"Decelerating!");
      }
      else{
        allowed_lin_vel = prev_lin_vel + (p_linear_acc_)*((ros::Time::now()-prev_time).toSec());
        // ROS_INFO_THROTTLE(1,"Accelerating!");
      }
    }
    else{
      allowed_lin_vel = req_lin_vel;
    }
  }
  else{
    // If the target velocity is not reached yet, increase/decrease according to acceleration
    if (std::abs(prev_lin_vel) < std::abs(req_lin_vel)){
      if (req_lin_vel < 0 && prev_lin_vel < 0){ //Robot is moving with neg vel, and target vel is neg, then accelerate with neg
        allowed_lin_vel = prev_lin_vel - (p_linear_acc_)*((ros::Time::now()-prev_time).toSec());
        // ROS_INFO_THROTTLE(1,"Accelerating in negative vel!");
      }
      else if (req_lin_vel < 0 && prev_lin_vel >=0){ //Robot is moving with pos vel, and target vel is neg, then decelerate with neg
        allowed_lin_vel = prev_lin_vel - (p_linear_dacc_)*((ros::Time::now()-prev_time).toSec());
        // ROS_INFO_THROTTLE(1,"Decelerating in positive vel!");
      }
      else if (req_lin_vel > 0 && prev_lin_vel < 0){ //Robot is moving with neg vel, and target vel is pos, then decelerate with pos
        allowed_lin_vel = prev_lin_vel + (p_linear_dacc_)*((ros::Time::now()-prev_time).toSec());
        // ROS_INFO_THROTTLE(1,"Decelerating in negative vel!");
      }
      else if (req_lin_vel > 0 && prev_lin_vel >= 0){ //Robot is moving with pos vel, and target vel is pos, then accelerate with pos
        allowed_lin_vel = prev_lin_vel + (p_linear_acc_)*((ros::Time::now()-prev_time).toSec());
        // ROS_INFO_THROTTLE(1,"Accelerating in positive vel!");
      }
    }
    
    // Original part
    // if(std::abs(req_lin_vel - prev_lin_vel) > (p_linear_acc_)*((ros::Time::now()-prev_time).toSec())){
    //   if(req_lin_vel - prev_lin_vel < 0.0){
    //     allowed_lin_vel = prev_lin_vel - (p_linear_acc_)*((ros::Time::now()-prev_time).toSec());
    //   }
    //   else{
    //     allowed_lin_vel = prev_lin_vel + (p_linear_acc_)*((ros::Time::now()-prev_time).toSec());
    //   }
    // }
    // else{
    //   allowed_lin_vel = req_lin_vel;
    // }

    else{ // Target vel is 0
      if( std::abs(req_lin_vel - prev_lin_vel) < (p_linear_dacc_)*((ros::Time::now()-prev_time).toSec()) ){
        if (prev_lin_vel<0){
          allowed_lin_vel = prev_lin_vel + (p_linear_dacc_)*((ros::Time::now()-prev_time).toSec());
          // ROS_INFO_THROTTLE(1,"Decelerating to 0 (-)");
        }
        else{
          allowed_lin_vel = prev_lin_vel - (p_linear_dacc_)*((ros::Time::now()-prev_time).toSec());
          // ROS_INFO_THROTTLE(1,"Decelerating to 0 (+)");
        }
      }
      else{
        allowed_lin_vel = req_lin_vel;
      }
    }
  }

  prev_lin_vel = allowed_lin_vel;
  prev_time = ros::Time::now();

  return allowed_lin_vel;
}

float MultiPointNavigationHandler::angAccelerationCheck(float req_ang_vel)
{
  static float prev_ang_vel = 0.0;
  static ros::Time prev_time = ros::Time::now();
  float allowed_ang_vel;

  if (std::abs(req_ang_vel - prev_ang_vel) > (p_angular_acc_) * ((ros::Time::now() - prev_time).toSec()))
  {
    if (req_ang_vel - prev_ang_vel < 0.0)
    {
      allowed_ang_vel = prev_ang_vel - (p_angular_acc_) * ((ros::Time::now() - prev_time).toSec());
    }
    else
    {
      allowed_ang_vel = prev_ang_vel + (p_angular_acc_) * ((ros::Time::now() - prev_time).toSec());
    }
  }
  else
  {
    allowed_ang_vel = req_ang_vel;
  }

  prev_ang_vel = allowed_ang_vel;
  prev_time = ros::Time::now();

  return allowed_ang_vel;
}

void MultiPointNavigationHandler::stopRobot()
{
  geometry_msgs::Twist to_cmd_vel;
  float current_lin_vel_cmd = linAccelerationCheck(0.0), current_ang_vel_cmd = angAccelerationCheck(0.0);

  if (current_lin_vel_cmd != 0.0 || current_ang_vel_cmd != 0.0)
  {
    while (current_lin_vel_cmd != 0.0 || current_ang_vel_cmd != 0.0)
    {
      to_cmd_vel.linear.x = current_lin_vel_cmd;
      to_cmd_vel.angular.z = current_ang_vel_cmd;
      cmd_vel_pub_.publish(to_cmd_vel);
      current_lin_vel_cmd = linAccelerationCheck(0.0);
      current_ang_vel_cmd = angAccelerationCheck(0.0);
    }
  }
  geometry_msgs::Twist stop;
  cmd_vel_pub_.publish(stop);
}

///////-----///////

void MultiPointNavigationHandler::cancelTask()
{
  stopRobot();
  setTaskResult(false);
  visualizePath(0, true);

  task_cancelled_ = true;
  task_parsed_ = true;
  task_active_ = false;
  task_paused_ = false;
}

bool MultiPointNavigationHandler::clearCostmapCb(std_srvs::Empty::Request&, std_srvs::Empty::Response&)
{
  if (costmap_ptr_.get() != nullptr)
  {
    ROS_INFO("[%s] Clearing costmap", name_.c_str());
    boost::unique_lock<costmap_2d::Costmap2D::mutex_t> lock_planner(*(costmap_ptr_.get()->getCostmap()->getMutex()));
    costmap_ptr_.get()->resetLayers();
  }
  else
    ROS_WARN("[%s] Costmap not initialized, cannot clear", name_.c_str());
  return true;
}

bool MultiPointNavigationHandler::loadRecoveryBehaviors(ros::NodeHandle node)
{
  XmlRpc::XmlRpcValue behavior_list;
  if (node.getParam("recovery_behaviors", behavior_list))
  {
    if (behavior_list.getType() == XmlRpc::XmlRpcValue::TypeArray)
    {
      for (int i = 0; i < behavior_list.size(); ++i)
      {
        if (behavior_list[i].getType() == XmlRpc::XmlRpcValue::TypeStruct)
        {
          if (behavior_list[i].hasMember("name") && behavior_list[i].hasMember("type"))
          {
            // check for recovery behaviors with the same name
            for (int j = i + 1; j < behavior_list.size(); j++)
            {
              if (behavior_list[j].getType() == XmlRpc::XmlRpcValue::TypeStruct)
              {
                if (behavior_list[j].hasMember("name") && behavior_list[j].hasMember("type"))
                {
                  std::string name_i = behavior_list[i]["name"];
                  std::string name_j = behavior_list[j]["name"];
                  if (name_i == name_j)
                  {
                    ROS_ERROR(
                        "A recovery behavior with the name %s already exists, this is not allowed. Using the default "
                        "recovery behaviors instead.",
                        name_i.c_str());
                    return false;
                  }
                }
              }
            }
          }
          else
          {
            ROS_ERROR(
                "Recovery behaviors must have a name and a type and this does not. Using the default recovery "
                "behaviors instead.");
            return false;
          }
        }
        else
        {
          ROS_ERROR(
              "Recovery behaviors must be specified as maps, but they are XmlRpcType %d. We'll use the default "
              "recovery behaviors instead.",
              behavior_list[i].getType());
          return false;
        }
      }

      // if we've made it to this point, we know that the list is legal so we'll create all the recovery behaviors
      for (int i = 0; i < behavior_list.size(); ++i)
      {
        try
        {
          // check if a non fully qualified name has potentially been passed in
          if (!recovery_loader_.isClassAvailable(behavior_list[i]["type"]))
          {
            std::vector<std::string> classes = recovery_loader_.getDeclaredClasses();
            for (unsigned int i = 0; i < classes.size(); ++i)
            {
              if (behavior_list[i]["type"] == recovery_loader_.getName(classes[i]))
              {
                // if we've found a match... we'll get the fully qualified name and break out of the loop
                ROS_WARN(
                    "Recovery behavior specifications should now include the package name. You are using a deprecated "
                    "API. Please switch from %s to %s in your yaml file.",
                    std::string(behavior_list[i]["type"]).c_str(), classes[i].c_str());
                behavior_list[i]["type"] = classes[i];
                break;
              }
            }
          }

          boost::shared_ptr<nav_core::RecoveryBehavior> behavior(
              recovery_loader_.createInstance(behavior_list[i]["type"]));

          // shouldn't be possible, but it won't hurt to check
          if (behavior.get() == NULL)
          {
            ROS_ERROR("The ClassLoader returned a null pointer without throwing an exception. This should not happen");
            return false;
          }

          // initialize the recovery behavior with its name
          behavior->initialize(behavior_list[i]["name"], &tf_buffer_, costmap_ptr_.get(), costmap_ptr_.get());
          recovery_behaviors_.push_back(behavior);
        }
        catch (pluginlib::PluginlibException& ex)
        {
          ROS_ERROR("Failed to load a plugin. Using default recovery behaviors. Error: %s", ex.what());
          return false;
        }
      }
    }
    else
    {
      ROS_ERROR(
          "The recovery behavior specification must be a list, but is of XmlRpcType %d. We'll use the default recovery "
          "behaviors instead.",
          behavior_list.getType());
      return false;
    }
  }
  else
  {
    // if no recovery_behaviors are specified, we'll just load the defaults
    return false;
  }

  // if we've made it here... we've constructed a recovery behavior list successfully
  return true;
}

void MultiPointNavigationHandler::coveragePercentage(std::vector<std::vector<float>> path)
{
  for (int i = 0; i < path.size(); ++i)
    unvisited_coords_.push_back(path[i]);
  n_init_unvisited_coords_ = unvisited_coords_.size();
  // debug
  // ROS_INFO("unvisited coords size: %i", unvisited_coords_.size());
}

void MultiPointNavigationHandler::outputMissedPts()
{
  ROS_INFO("[%s] Points not yet cleaned size: %i", name_.c_str(), unvisited_coords_.size());

  for (int i = 0; i < unvisited_coords_.size(); i++)
  {
    ROS_INFO("[%s] Coords: (%f,%f)", name_.c_str(), unvisited_coords_[i][0], unvisited_coords_[i][1]);
  }
}

void MultiPointNavigationHandler::adjustPlanForObstacles(std::vector<std::vector<float>>& path)
{
  ROS_INFO("[%s] Adjusting plan", name_.c_str());
  std::vector<geometry_msgs::PoseStamped> interplan;
  std::vector<geometry_msgs::PoseStamped> decimated_plan;
  nav_msgs::GetPlan srv;
  int start_segment_idx, end_segment_idx;
  int free_backward_offset = 2;
  int free_forward_offset = 2;
  int obstacle_check_window = 10;

  if (blocked_idx_ <= 1)
    return;

  start_segment_idx = blocked_idx_ - free_backward_offset;
  if (start_segment_idx < 0)
    return;
  
  srv.request.start.header.frame_id = "map";
  srv.request.start.pose.position.x = coords_for_nav_[start_segment_idx][0];
  srv.request.start.pose.position.y = coords_for_nav_[start_segment_idx][1];

  // Check for end goal obstruction if the end goal is in horizon
  if (end_in_horizon_)
  {
    if (obstacleCheckSinglePoint(coords_for_nav_.size() - 1))
    {
      ROS_INFO("[%s] End goal is obstructed, adjusting end goal", name_.c_str());

      // Erase points from the farthest free point to the end goal
      int farthest_free_idx = coords_for_nav_.size() - 2;
      while (farthest_free_idx > start_segment_idx && obstacleCheckSinglePoint(farthest_free_idx))
      {
        --farthest_free_idx;
      }

      coords_for_nav_.erase(coords_for_nav_.begin() + farthest_free_idx + 1, coords_for_nav_.end());
      is_original_coords_for_nav_.erase(is_original_coords_for_nav_.begin() + farthest_free_idx + 1,
                                        is_original_coords_for_nav_.end());
      
      // Check whether we still need to replan around the obstacle
      if (blocked_trigger_idx_ >= coords_for_nav_.size() - 1)
      {
        return;
      }
    }
    else
    {
      // Check whether there is a path to the end goal
      // If we don't get a path plan, it means that the end goal is blocked off
      srv.request.goal.header.frame_id = "map";
      srv.request.goal.pose.position.x = coords_for_nav_.back()[0];
      srv.request.goal.pose.position.y = coords_for_nav_.back()[1];
      srv.request.goal.pose.orientation.w = 1;

      if (!(make_plan_client_.call(srv) && srv.response.plan.poses.size() > 0))
      {
        ROS_INFO("[%s] End goal is blocked off, adjusting end goal", name_.c_str());

        // Erase points from the farthest free point before the obstacle to the end goal
        LineObstructionChecker line_obstruction_checker(*(costmap_ptr_->getCostmap()));
        
        int farthest_obst_idx = coords_for_nav_.size() - 1;
        for (farthest_obst_idx; farthest_obst_idx > start_segment_idx; --farthest_obst_idx)
        {
          if (line_obstruction_checker.isLineObstructed(
                coords_for_nav_[farthest_obst_idx], coords_for_nav_[farthest_obst_idx - 1]))
          {
            break;
          }
        }
        
        int farthest_free_idx = std::max(farthest_obst_idx - 1, start_segment_idx);
        while (farthest_free_idx > start_segment_idx && obstacleCheckSinglePoint(farthest_free_idx))
        {
          --farthest_free_idx;
        }

        coords_for_nav_.erase(coords_for_nav_.begin() + farthest_free_idx + 1, coords_for_nav_.end());
        is_original_coords_for_nav_.erase(is_original_coords_for_nav_.begin() + farthest_free_idx + 1,
                                          is_original_coords_for_nav_.end());
      
        // Check whether we still need to replan around the obstacle
        if (blocked_trigger_idx_ >= coords_for_nav_.size() - 1)
        {
          return;
        }
      }

      srv.response.plan = nav_msgs::Path();
    }
  }

  // looking for free index
  int coords_checked_for_obstacle = 0;
  int farthest_obst_idx = blocked_idx_ + 1;
  for (int i = blocked_idx_ + 1; i < coords_for_nav_.size(); i++)
  {
    if (coords_checked_for_obstacle > obstacle_check_window)
      break;

    if (obstacleCheckSinglePoint(i))
      farthest_obst_idx = i;
    coords_checked_for_obstacle += 1;
  }

  ROS_INFO("[%s] Start segment idx: %i", name_.c_str(), start_segment_idx);
  end_segment_idx = farthest_obst_idx + free_forward_offset;
  if (end_segment_idx >= coords_for_nav_.size())
  {
    end_segment_idx = coords_for_nav_.size() - 1;
  }
  ROS_INFO("[%s] End segment idx: %i", name_.c_str(), end_segment_idx);
  
  srv.request.goal.header.frame_id = "map";
  srv.request.goal.pose.position.x = coords_for_nav_[end_segment_idx][0];
  srv.request.goal.pose.position.y = coords_for_nav_[end_segment_idx][1];
  srv.request.goal.pose.orientation.w = 1;

  ROS_INFO("[%s] Making plan", name_.c_str());
  if (make_plan_client_.call(srv) && srv.response.plan.poses.size() > 0)
  {
    interplan = srv.response.plan.poses;
    obstacle_path_pub_.publish(srv.response.plan);

    ROS_INFO("[%s] Finish plan", name_.c_str());
  }
  else
  {
    ROS_ERROR("[%s] Failed to get path plan from move_base", name_.c_str());

    ROS_INFO("[%s] Retrying with best effort", name_.c_str());
    if (make_reachable_plan_client_.call(srv) && srv.response.plan.poses.size() > 0)
    {
      interplan = srv.response.plan.poses;
      obstacle_path_pub_.publish(srv.response.plan);

      ROS_INFO("[%s] Finish plan", name_.c_str());
    }
    else
    {
      ROS_ERROR("[%s] Failed to find best effort path plan, aborting", name_.c_str());
      return;
    }
  }

  if (interplan.size() > 0)
  {
    ROS_WARN("[%s] Adding to interplan", name_.c_str());
    ROS_INFO("[%s] Interplan size: %i", name_.c_str(), interplan.size());
    std::vector<std::vector<float>> interplan_segment;
    std::vector<bool> false_v;
    decimatePlan(interplan, decimated_plan);
    ROS_INFO("[%s] Decimated_plan size: %i", name_.c_str(), decimated_plan.size());
    for (int i = 0; i < decimated_plan.size(); i++)
    {
      interplan_segment.push_back(
          { (float)decimated_plan[i].pose.position.x, (float)decimated_plan[i].pose.position.y });
      false_v.push_back(false);
    }
    pushed_idx_ = interplan_segment.size();
    coords_for_nav_.erase(coords_for_nav_.begin() + start_segment_idx, coords_for_nav_.begin() + end_segment_idx);
    coords_for_nav_.insert(coords_for_nav_.begin() + start_segment_idx, interplan_segment.begin(),
                           interplan_segment.end());
    is_original_coords_for_nav_.erase(is_original_coords_for_nav_.begin() + start_segment_idx,
                                      is_original_coords_for_nav_.begin() + end_segment_idx);
    is_original_coords_for_nav_.insert(is_original_coords_for_nav_.begin() + start_segment_idx, false_v.begin(),
                                       false_v.end());
  }
}

void MultiPointNavigationHandler::decimatePlan(const std::vector<geometry_msgs::PoseStamped>& plan_in,
                                               std::vector<geometry_msgs::PoseStamped>& plan_out)
{
  ROS_INFO("[%s] Decimating plan", name_.c_str());
  plan_out.clear();
  plan_out.push_back(plan_in[0]);
  geometry_msgs::PoseStamped last_pose = plan_in[0];

  for (int i = 1; i < plan_in.size() - 1; i++)
  {
    double dee = calcPoseStampedDistances(last_pose, plan_in[i]);
    if (dee > 0.15)
    {
      plan_out.push_back(plan_in[i]);
      last_pose = plan_in[i];
      ROS_INFO("[%s] p1: %f p2: %f", name_.c_str(), last_pose.pose.position.x, last_pose.pose.position.y);
    }
  }
  plan_out.push_back(plan_in[plan_in.size() - 1]);
}

bool MultiPointNavigationHandler::stopAtObstacleEnabled()
{
  std_srvs::Trigger srv;

  // If we get a response then update p_stop_at_obstacle
  // If we don't, just return whatever the previous p_stop_at_obstacle is
  if (stop_at_obstacle_enabled_client_.call(srv))
  {
    if (srv.response.success)
      p_stop_at_obstacle_ = true;
    else
      p_stop_at_obstacle_ = false;
  }

  return p_stop_at_obstacle_;
}

}  // namespace task_supervisor
