#ifndef TASK_SUPERVISOR_MULTI_POINT_NAVIGATION_HANDLER_H
#define TASK_SUPERVISOR_MULTI_POINT_NAVIGATION_HANDLER_H

#include <ros/ros.h>

#include <task_supervisor/common.h>
#include <task_supervisor/plugins/task_handler.h>

#include <movel_seirios_msgs/Reports.h>
#include <movel_seirios_msgs/ObstructionStatus.h>
#include <movel_seirios_msgs/MultipointPath.h>
#include <movel_seirios_msgs/MultipointProgress.h>

#include <boost/thread/mutex.hpp>
#include <std_srvs/Empty.h>
#include <std_srvs/Trigger.h>

#include <algorithm>
#include <cmath>
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/ColorRGBA.h"
#include <tf/transform_listener.h>
#include <tf2_ros/transform_listener.h>
#include "std_msgs/Bool.h"
#include <costmap_2d/costmap_2d_ros.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <nav_msgs/GetPlan.h>
#include <nav_msgs/Path.h>

#include <dynamic_reconfigure/server.h>
#include <multi_point/MultipointConfig.h>

#include <pluginlib/class_loader.hpp>
#include <nav_core/recovery_behavior.h>
#include <nav_core/base_global_planner.h>

#include <std_msgs/Float64.h>
#include <geometry_msgs/Pose.h>

#define co_ord_pair std::pair<float, float>

namespace task_supervisor
{
class MultiPointNavigationHandler : public TaskHandler
{
// private:
public:
  // ROS params
  float p_point_gen_dist_;
  float p_look_ahead_dist_;
  float p_obst_check_freq_;
  float p_goal_tolerance_ = 0.1;
  float p_angular_tolerance_;
  bool p_spline_enable_;
  float p_obstruction_timeout_;
  float p_kp_, p_ki_, p_kd_;
  bool p_forward_only_ = true;
  float p_angular_acc_, p_linear_acc_, p_linear_dacc_;
  int p_bypass_degree_ = 3;
  float p_curve_vel_ = 0.1;
  float p_curve_scale_ = 30.0;
  bool p_recovery_behavior_enabled_;
  bool p_stop_at_obstacle_;
  bool p_slow_curve_enable_ = true;
  bool p_slow_points_enable_ = false;

  // variables
  boost::mutex mtx_;
  bool task_cancelled_;
  geometry_msgs::Pose robot_pose_;
  bool isHealthy_;
  std::vector<int> major_indices_;
  std::vector<std::vector<float>> rcvd_multi_coords_;
  std::vector<std::vector<float>> coords_for_nav_;
  std::vector<bool> is_original_coords_for_nav_;
  bool obstructed_;
  int bypass_degree_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_ear_;
  std::shared_ptr<costmap_2d::Costmap2DROS> costmap_ptr_;
  std::shared_ptr<costmap_2d::Costmap2DROS> local_costmap_ptr_;
  float min_obst_timeout_ = 4.0;
  float obst_check_interval_ = 2.0;
  float angular_tolerance_ = 0.1;
  const float min_angular_vel_ = 0.05, min_linear_vel_ = 0.05;
  const float max_angular_vel_ = 1.0, max_linear_vel_ = 1.0;
  float angular_vel_;
  float linear_vel_;
  int look_ahead_points_ = 2;
  bool at_start_point_ = false;
  bool start_at_nearest_point_ = false;
  std::string global_planner_;
  int current_idx_;
  double obs_x_;
  double obs_y_;
  int blocked_idx_;
  int pushed_idx_ = 0;

  // variables for coverage percentage
  int n_init_unvisited_coords_ = 0;
  std::vector<std::vector<float>> unvisited_coords_;
  std_msgs::Float64 area_percentage_;

  std::vector<boost::shared_ptr<nav_core::RecoveryBehavior>> recovery_behaviors_;
  unsigned int recovery_index_;
  pluginlib::ClassLoader<nav_core::RecoveryBehavior> recovery_loader_;
  boost::shared_ptr<nav_core::BaseGlobalPlanner> planner_ptr_;
  pluginlib::ClassLoader<nav_core::BaseGlobalPlanner> bgp_loader_{ "nav_core", "nav_core::BaseGlobalPlanner" };
  ros::ServiceClient make_reachable_plan_client_;

  std::shared_ptr< dynamic_reconfigure::Server<multi_point::MultipointConfig> > dynamic_reconf_server_;
  dynamic_reconfigure::Server<multi_point::MultipointConfig>::CallbackType dynamic_reconfigure_callback_;

  // topics/services
  ros::Subscriber robot_pose_sub_;
  ros::Subscriber pose_coverage_subscriber_;
  ros::Publisher path_visualize_pub_;
  ros::Publisher cmd_vel_pub_;
  ros::Publisher current_goal_pub_;
  ros::Publisher coverage_percentage_pub_;
  ros::Publisher obstacle_path_pub_;
  ros::ServiceServer path_srv_;
  ros::ServiceServer clear_costmap_srv_;
  ros::ServiceClient stop_at_obstacle_enabled_client_;

  template <typename param_type>
  bool load_param_util(std::string param_name, param_type& output);
  bool loadParams();

  // Generating Path -------------------------------------------------------

  /**
   * @brief Method which handles all of points/path generation inside it when major points received
   * @param rcvd_multi_coords The points received from task or service through which we have to plan the path
   * @param coords_for_nav Passed by reference, will hold final generated coordinates
   * @param for_nav To tell the method if the generation is for task or service
   * @return bool which reports success/failure of the method
   */
  bool pointsGen(std::vector<std::vector<float>>, std::vector<std::vector<float>>&, bool);

  /**
   * @brief Method which splines the desired corners of the generated path
   * @param coords_for_spline Passed by reference, The basic generated points which have to be splined
   * @param points_to_spline The indices of the coords_for_spline vector which represent the corners to be splined
   * @param coords_for_nav Passed by reference, will hold final generated coordinates
   */
  void splinePoints(std::vector<std::vector<float>>&, std::vector<std::vector<int>>, std::vector<std::vector<float>>&);

  /**
   * @brief Method which filters which corners/points are eligible to be splined
   * @param rcvd_multi_coords The points received from task or service through which we have to plan the path
   * @param major_indices Indices of the original received points in the basic generated points vector
   * @param points_to_spline Passed by reference, will hold the indices of corners/points to be splined
   */
  bool getPointsToSpline(std::vector<std::vector<float>>, std::vector<int>, std::vector<std::vector<int>>&);

  /**
   * @brief Method which calculates the mid-point between 2 given points
   */
  co_ord_pair midPoint(co_ord_pair, co_ord_pair);

  /**
   * @brief Method which calculates the intersecting point of 2 lines
   */
  std::vector<float> intersectPoint(co_ord_pair, co_ord_pair, co_ord_pair, co_ord_pair);

  //------------------------------------------------------------------------

  // Visualize, topics, service and config ---------------------------------

  /**
   * @brief Method for rviz visualization of the path and progress
   */
  void visualizePath(int, bool);

  /**
   * @brief Method to print generated path for debugging
   */
  void printGeneratedPath(std::vector<std::vector<float>>);

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
  void robotPoseCB(const geometry_msgs::Pose::ConstPtr&);

  /**
   * @brief Subscribing to dynamic reconfigure
   */
  void reconfCB(multi_point::MultipointConfig&, uint32_t);
  bool clearCostmapCb(std_srvs::Empty::Request&, std_srvs::Empty::Response&);

  // -----------------------------------------------------------------------

  // Navigation ------------------------------------------------------------

  /**
   * @brief Method to handle navigation to the current point goal
   * @param instance_index Index of the point in coords_for_nav_ that the robot has to navigate to
   * @return bool Success/failure in navigation
   */
  bool navToPoint(int);

  /**
   * @brief Method to handle PID for angular movement
   */
  float pidFn(float, float);

  /**
   * @brief Method to check for obstacle on the current goal and the look-ahead distance
   */
  bool obstacleCheck(int);

  /**
   * @brief Method to limit linear velocity change (acc.) of the robot
   */
  float linAccelerationCheck(float);

  /**
   * @brief Method to limit angular velocity change (acc.) of the robot
   */
  float angAccelerationCheck(float);

  /**
   * @brief Method to stop robot smoothly within deceleration limits
   */
  void stopRobot();

  // -----------------------------------------------------------------------

  bool loadRecoveryBehaviors(ros::NodeHandle node);

  void poseCoverageCB(const geometry_msgs::Pose::ConstPtr& msg);

  void coveragePercentage(std::vector<std::vector<float>> path);

  void outputMissedPts();

  bool loadPlanner(const std::string& planner, costmap_2d::Costmap2DROS* costmap_ros);

  void adjustPlanForObstacles(std::vector<std::vector<float>>& path);

  void decimatePlan(const std::vector<geometry_msgs::PoseStamped>& plan_in,
                    std::vector<geometry_msgs::PoseStamped>& plan_out);
  
  bool stopAtObstacleEnabled();

public:
  MultiPointNavigationHandler();
  ~MultiPointNavigationHandler();

  /**
   * @brief Method called by task_supervisor when a navigation task is received
   * @param task Relevant task passed to handler by task supervisor
   * @param error_message Error message returned by this handler if execution fails
   * @return ReturnCode which indicates failure, cancellation or success
   */
  virtual ReturnCode runTask(movel_seirios_msgs::Task& task, std::string& error_message);

  bool setupHandler();
  void cancelTask();
};

}  // namespace task_supervisor

#endif