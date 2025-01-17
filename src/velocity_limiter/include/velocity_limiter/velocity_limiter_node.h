#ifndef VELOCITY_LIMITER_NODE_H
#define VELOCITY_LIMITER_NODE_H

#include <sstream>

#include <ros_utils/ros_utils.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <std_srvs/SetBool.h>
#include <std_srvs/Trigger.h>
#include <movel_seirios_msgs/Uint8.h>
#include <movel_seirios_msgs/StringTrigger.h>
#include <tf/transform_listener.h>
#include <actionlib_msgs/GoalStatusArray.h>

#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/crop_box.h>

#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/publisher.h>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <diagnostic_msgs/DiagnosticStatus.h>
#include <diagnostic_msgs/KeyValue.h>

#include <velocity_limiter/common.h>
#include <velocity_limiter/yaml.h>
#include <velocity_limiter/velocity_grid.h>
#include <velocity_limiter/velocity_limiter.h>

#include <sw/redis++/redis++.h>

/**
 * Limit the velocity given the point cloud data representing the obstacles
 */
class VelocityLimiterNode
{
public:
  VelocityLimiterNode();
  ~VelocityLimiterNode();

  VelocityLimit autonomous_velocity_limit_;
  VelocityLimit safe_teleop_velocity_limit_;

private:
  bool loadParams();
  void setupTopics();

  void onAutonomousVelocity(const geometry_msgs::Twist::ConstPtr& velocity);
  void onTeleopVelocity(const geometry_msgs::Twist::ConstPtr& velocity);
  void onCostmap(const nav_msgs::OccupancyGrid::ConstPtr& costmap);
  void onClickedPoint(const geometry_msgs::PointStamped::ConstPtr& point);

  bool onEnableTeleopSafety(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& resp);
  bool onCheckTeleopSafety(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& resp);
  bool onCheckAutonomousSafety(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& resp);
  bool onEnableAutonomousSafety(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& resp);
  bool onSwitchLimitSet(movel_seirios_msgs::StringTrigger::Request& req,
                        movel_seirios_msgs::StringTrigger::Response& resp);

  bool loadLimitSetMap(std::map<std::string, Set>& limit_set_map);
  bool switchLimitSet(std::string new_profile);
  bool isValidLimitSetMap(std::map<std::string, Set>& limit_set_map);
  bool isValidZoneList(const std::vector<Zone>& zone_list);
  bool computeVelocityGrids();
  void updateVelocityLimits(VelocityLimit& velocity_limit, VelocityGrid& velocity_grid, const pcl::PointCloud<pcl::PointXYZ>& cloud);
  bool onPublishZones(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& resp);
  bool onPublishGrid(movel_seirios_msgs::Uint8::Request& req, movel_seirios_msgs::Uint8::Response& resp);
  void onActionStatus(actionlib_msgs::GoalStatusArray msg);
  bool isCloudOutdated(const sensor_msgs::PointCloud2& cloud);
  void updateCloudBuffer(sensor_msgs::PointCloud2 new_cloud);

  void nodeState(diagnostic_updater::DiagnosticStatusWrapper& stat);
  void limitEnabled(diagnostic_updater::DiagnosticStatusWrapper& stat);
  void safeTeleopEnabled(diagnostic_updater::DiagnosticStatusWrapper& stat);
  void profile(diagnostic_updater::DiagnosticStatusWrapper& stat);
  void timeLastCloud(diagnostic_updater::DiagnosticStatusWrapper& stat);
  void timeLastVel(diagnostic_updater::DiagnosticStatusWrapper& stat);

  void safeAutonomyTimerCb(const ros::TimerEvent& event);

  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  sw::redis::ConnectionOptions opts_;
  std::string redis_host_;
  std::string redis_port_;
  int socket_timeout_;
  std::ostringstream redis_conn_;
  // redis keys
  std::string redis_autonomous_velo_lim_key_;
  std::string redis_teleop_velo_lim_key_;

  ros::Subscriber autonomous_velocity_sub_;
  ros::Subscriber teleop_velocity_sub_;
  ros::Subscriber costmap_sub_;
  ros::Subscriber clicked_point_sub_;
  ros::Publisher autonomous_velocity_limited_pub_;
  ros::Publisher teleop_velocity_limited_pub_;
  ros::Publisher is_teleop_velocity_overridden_pub_;
  ros::Publisher velocity_grid_pub_;
  ros::Publisher velocity_frontiers_pub_;
  ros::Publisher merged_cloud_pub_;
  ros::Publisher task_pause_pub_;
  ros::Publisher stopped_time_pub_;

  ros::ServiceServer enable_srv_;
  ros::ServiceServer enable_safe_teleop_srv_;
  ros::ServiceServer switch_limit_set_srv_;
  ros::ServiceServer publish_zones_srv_;
  ros::ServiceServer publish_grid_srv_;

  ros::ServiceServer safe_teleop_checker;
  ros::ServiceServer safe_autonomy_checker;

  ros::Timer autonomous_safety_timer_;

  /**
   * The threshold costmap value to determine obstruction
   */
  int p_obstruction_threshold_;
  /**
   * The robot base frame.
   */
  std::string p_base_frame_;
  /**
   * The frame that the point cloud data will be merged in.
   */
  std::string p_merging_frame_;
  /**
   * Whether to publish the topic ```cloud/persisted```.
   */
  bool p_publish_pcl_;
  /**
   * Resolution of the velocity grid, defining the size of each grid cell.
   */
  double p_grid_resolution_;
  /**
   * Not in use for now.
   */
  double p_low_pass_gain_;
  /**
   * If the point loud data appears for less that this period, the data will be discard to avoid disturbance.
   */
  double p_cloud_persistence_;
  /**
   * The initial set of velocity limit regions.
   */
  std::string p_initial_limit_set_;
  /**
   * The initial set of velocity limit regions for safe teleop.
   */
  std::string p_initial_safe_teleop_limit_set_;
  /**
   * The set of velocity limit regions for teleop guard mode.
   */
  std::string p_safe_teleop_limit_set_;
  /**
   * A map of limit set name and the set.
   */
  std::map<std::string, Set> p_limit_set_map_;
  /**
   * A map of velocity grid name and the grid.
   */
  std::map<std::string, VelocityGrid> p_velocity_grid_map_;

  double p_stop_timeout_;
  bool p_start_enabled_;
  bool p_start_teleop_enabled_;

  /**
   * Whether autonomous velocity limit is enabled.
   */
  bool is_autonomous_safety_enabled_;
  /**
   * Whether teleop emergency brake is enabled
   */
  bool is_teleop_safety_enabled_;
  /**
   * Whether teleop velocity is overridden by velo limiter
   */
  bool is_teleop_velocity_overridden_;
  /**
   * Name of the current limit set.
   */
  std::string current_profile_;
  /**
   * Whether the first cloud message is received.
   */
  bool first_cloud_received_;
  /**
   * Time of previous receipt of cloud data.
   */
  ros::Time last_cloud_time_;
  /**
   * Whether the first velocity message is received.
   */
  bool first_vel_received_;
  /**
   * Time of previous receipt of velocity data.
   */
  ros::Time last_vel_time_;
  /**
   * Stores a limit set.
   */
  Set autonomous_limit_set_;
  /**
   * Stores a limit set for safe teleop mode.
   */
  Set safe_teleop_limit_set_;
  /**
   * Stores the buffer of point cloud data. To determine whther the cloud data is out of date.
   */
  std::vector<sensor_msgs::PointCloud2> cloud_buffer_;
  /**
   * Stores the calculated velocity grid.
   */
  VelocityGrid autonomous_velocity_grid_;
  /**
   * Stores the calculated velocity grid.
   */
  VelocityGrid safe_teleop_velocity_grid_;
  /**
   * The limiter object.
   */
  VelocityLimiter limiter_;
  /**
   * The diagnostic updater object.
   */
  diagnostic_updater::Updater updater_;

  tf::TransformListener tf_listener_;

  bool is_stopped_;
  bool is_stopped_during_autonomy_;
  ros::Time t_stopped_;
};

#endif
