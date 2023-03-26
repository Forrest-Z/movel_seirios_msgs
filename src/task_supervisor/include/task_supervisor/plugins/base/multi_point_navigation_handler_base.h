#ifndef TASK_SUPERVISOR_MULTI_POINT_NAVIGATION_BASE_H
#define TASK_SUPERVISOR_MULTI_POINT_NAVIGATION_BASE_H

#include <cmath>
#include <ros/ros.h>

#include <costmap_2d/costmap_2d_ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <nav_core/recovery_behavior.h>
#include <std_msgs/Float64.h>
#include <std_srvs/Empty.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <movel_seirios_msgs/MultipointPath.h>
#include <movel_seirios_msgs/MultipointProgress.h>
#include <multi_point/MultipointConfig.h>
#include <multi_point_navigation/path_generator.h>
#include <multi_point_navigation/recovery_behavior_loader.h>
#include <task_supervisor/common.h>
#include <task_supervisor/plugins/task_handler.h>

namespace task_supervisor
{
class MultiPointNavigationHandlerBase : public TaskHandler
{
public:
  // default values
  const float min_obstacle_timeout_ = 4.0;
  const float min_obstacle_check_rate_ = 0.5;
  const float max_obstacle_check_rate_ = 10.0;
  const float min_angular_vel_ = 0.05;
  const float max_angular_vel_ = 1.0;
  const float min_linear_vel_ = 0.05;
  const float max_linear_vel_ = 1.0;
  const int min_lookahead_points_ = 2;
  const float min_dist_between_points_ = 0.1;

  // ROS params
  float p_look_ahead_dist_;
  float p_obstacle_check_rate_;
  float p_goal_tolerance_ = 0.1;
  float p_angular_tolerance_;
  bool p_spline_enable_;
  float p_obstruction_timeout_;
  bool p_recovery_behavior_enabled_;

  // values derived from params ("second-level params")
  float obstacle_check_interval_ = 2.0;
  int look_ahead_points_ = 2;

  // bookkeeping
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  bool task_cancelled_;
  bool robot_pose_available_;
  bool is_healthy_;
  geometry_msgs::Pose robot_pose_;
  std::vector<std::vector<float>> rcvd_multi_coords_;
  std::vector<std::vector<float>> coords_for_nav_;
  bool obstructed_;
  int bypass_degree_;
  std::shared_ptr<costmap_2d::Costmap2DROS> costmap_ptr_;
  float angular_vel_;
  float linear_vel_;
  bool at_start_point_ = false;
  bool start_at_nearest_point_ = false;

  // variables for coverage percentage
  double total_path_size_ = 0;
  std::vector<int> pending_path_;
  std::vector<int> completed_path_;
  std_msgs::Float64 area_percentage_;

  std::shared_ptr<multi_point_navigation::PathGeneratorConfig> path_generator_config_ptr_;
  multi_point_navigation::PathGenerator path_generator;

  multi_point_navigation::RecoveryBehaviorLoader recovery_behavior_loader_;
  std::vector<boost::shared_ptr<nav_core::RecoveryBehavior>> recovery_behaviors_;
  unsigned int recovery_index_;

  std::shared_ptr<dynamic_reconfigure::Server<multi_point::MultipointConfig>> dynamic_reconfigure_srv_;
  dynamic_reconfigure::Server<multi_point::MultipointConfig>::CallbackType dynamic_reconfigure_cb_;

  // topics/services
  ros::Subscriber robot_pose_sub_;
  ros::Subscriber pose_coverage_subscriber_;
  ros::Publisher path_visualize_pub_;
  ros::Publisher cmd_vel_pub_;
  ros::Publisher current_goal_pub_;
  ros::Publisher coverage_percentage_pub_;
  ros::ServiceServer path_srv_;
  ros::ServiceServer clear_costmap_srv_;

public:
  MultiPointNavigationHandlerBase();
  ~MultiPointNavigationHandlerBase();

  bool setupHandler();

protected:
  bool loadParams();

  void setupDerivedValues();

  void setupTopicsAndServices();

  void setupDynamicReconfigure();

  void cancelTask();

  /********************
   * Generating Paths *
   ********************/

  bool generatePathForNavigation(std::vector<multi_point_navigation::Point> major_pts, multi_point_navigation::Path& path);

  bool prepareMajorPointsForPathGeneration(std::vector<multi_point_navigation::Point> major_pts);

  /**************************************
   * View, topics, services, and config *
   **************************************/

  /**
   * @brief Method for rviz visualization of the path and progress
   */
  void visualizePath(int, bool);

  /**
   * @brief Method to print generated path for debugging
   */
  void printGeneratedPath(std::vector<multi_point_navigation::Point>);

  /**
   * @brief Publish current progress of the navigation on topic
   */
  void publishCurrentGoal(int);

  /**
   * @brief Service callback to generate path for given coordinates without navigation
   */
  bool pathServiceCb(movel_seirios_msgs::MultipointPath::Request&, movel_seirios_msgs::MultipointPath::Response&);

  /**
   * @brief Subscribing to robot's current pose
   */
  void robotPoseCb(const geometry_msgs::Pose::ConstPtr&);

  /**
   * @brief Subscribing to dynamic reconfigure
   */
  void reconfigureCb(multi_point::MultipointConfig&, uint32_t);

  /**
   * @brief Service callback to clear costmap
   */
  bool clearCostmapCb(std_srvs::Empty::Request&, std_srvs::Empty::Response&);

  void poseCoverageCb(const geometry_msgs::Pose::ConstPtr& msg);

  void coveragePercentage(std::vector<multi_point_navigation::Point> path);

  void outputMissedPts();
};
}  // namespace task_supervisor

#endif