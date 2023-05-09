#ifndef TASK_SUPERVISOR_MULTI_POINT_NAVIGATION_BASE_H
#define TASK_SUPERVISOR_MULTI_POINT_NAVIGATION_BASE_H

#include <cmath>
#include <ros/ros.h>

#include <costmap_2d/costmap_2d_ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_core/recovery_behavior.h>
#include <std_msgs/Float64.h>
#include <std_srvs/Empty.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <movel_seirios_msgs/MultipointPath.h>
#include <movel_seirios_msgs/MultipointProgress.h>
#include <multi_point_navigation/path_generator.h>
#include <multi_point_navigation/recovery_behavior_loader.h>
#include <task_supervisor/common.h>
#include <task_supervisor/plugins/task_handler.h>

namespace task_supervisor
{
enum ObstructionType
{
  LETHAL,
  INSCRIBED_INFLATED,
  NO_INFORMATION,
  FREE
};

class MultiPointNavigationHandlerBase : public TaskHandler
{
public:
  // default values
  const float c_min_obstacle_timeout_ = 4.0;
  const float c_min_obstacle_check_rate_ = 0.5;
  const float c_max_obstacle_check_rate_ = 10.0;
  const float c_min_angular_vel_ = 0.05;
  const float c_max_angular_vel_ = 1.0;
  const float c_min_linear_vel_ = 0.05;
  const float c_max_linear_vel_ = 1.0;
  const int c_min_lookahead_points_ = 2;
  const float c_min_dist_between_points_ = 0.1;

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
  bool obstructed_;
  std::shared_ptr<costmap_2d::Costmap2DROS> costmap_ptr_;
  bool start_at_nearest_point_ = false;

  std::shared_ptr<multi_point_navigation::PathGeneratorConfig> path_generator_config_ptr_;
  multi_point_navigation::PathGenerator path_generator;

  multi_point_navigation::RecoveryBehaviorLoader recovery_behavior_loader_;
  std::vector<boost::shared_ptr<nav_core::RecoveryBehavior>> recovery_behaviors_;
  unsigned int recovery_index_;

  // topics/services
  ros::Subscriber robot_pose_sub_;
  ros::Subscriber pose_coverage_subscriber_;
  ros::Publisher path_visualize_pub_;
  ros::Publisher cmd_vel_pub_;
  ros::Publisher current_goal_pub_;
  ros::Publisher coverage_percentage_pub_;
  ros::ServiceServer path_srv_;
  ros::ServiceServer clear_costmap_srv_;

  double task_linear_vel_;
  double task_angular_vel_;

public:
  MultiPointNavigationHandlerBase();
  ~MultiPointNavigationHandlerBase();

  bool setupHandler();

  /**
   * @brief Method called by task_supervisor when a navigation task is received
   * @param task Relevant task passed to handler by task supervisor
   * @param error_message Error message returned by this handler if execution fails
   * @return ReturnCode which indicates failure, cancellation or success
   */
  virtual ReturnCode runTask(movel_seirios_msgs::Task& task, std::string& error_message);

  void cancelTask();

protected:
  bool loadParams();

  void setupDerivedValues();

  void setupTopicsAndServices();

  bool parseTask(const movel_seirios_msgs::Task& task, std::vector<multi_point_navigation::Point>& major_pts,
                 double& linear_veloctiy, double& angular_velocity, bool& start_at_nearest_point,
                 std::string& error_msg);

  bool parseTaskPayload(std::string payload_string, std::vector<multi_point_navigation::Point>& major_pts,
                        std::string& error_msg);

  void validateTaskVelocity(double& linear_veloctiy, double& angular_velocity);

  virtual bool navigateToPoint(const multi_point_navigation::Path& path, int goal_index) 
  {
    ROS_INFO("[%s] Called base class' navigateToPoint()", name_.c_str());
    return true;
  };

  virtual void stopNavigation()
  {
    ROS_INFO("[%s] Called base class' stopNavigation()", name_.c_str());
    return;
  };

  /********************
   * Generating Paths *
   ********************/

  bool generatePathForNavigation(std::vector<multi_point_navigation::Point> major_pts, bool start_at_nearest_point,
                                 multi_point_navigation::Path& path);

  bool prepareMajorPointsForPathGeneration(std::vector<multi_point_navigation::Point> major_pts);

  /************************
   * Obstruction Checking *
   ************************/

  bool checkForObstacle(const std::vector<multi_point_navigation::Point>& path_points, int path_index,
                        ObstructionType& obstruction_type);

  void getObstructionTypeAtCoordinate(double world_x, double world_y, ObstructionType& obstruction_type);

  /**************************************
   * View, topics, services, and config *
   **************************************/

  /**
   * @brief Method for rviz visualization of the path and progress
   */
  void visualizePath(multi_point_navigation::Path& path, int current_index, int marker_action);

  /**
   * @brief Publish current progress of the navigation on topic
   */
  void publishCurrentGoal(const multi_point_navigation::Point& point);

  /**
   * @brief Service callback to generate path for given coordinates without navigation
   */
  bool pathServiceCb(movel_seirios_msgs::MultipointPath::Request& req,
                     movel_seirios_msgs::MultipointPath::Response& res);

  /**
   * @brief Subscribing to robot's current pose
   */
  void robotPoseCb(const geometry_msgs::Pose::ConstPtr& msg);

  /**
   * @brief Service callback to clear costmap
   */
  bool clearCostmapCb(std_srvs::Empty::Request&, std_srvs::Empty::Response&);
};
}  // namespace task_supervisor

#endif