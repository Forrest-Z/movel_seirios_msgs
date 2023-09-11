#ifndef DIFF_DRIVE_DOCKING_H
#define DIFF_DRIVE_DOCKING_H

#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <math.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
#include <tf/tf.h>
#include <std_srvs/SetBool.h>
#include <actionlib_msgs/GoalID.h>

class DiffDriveDocking
{
private:
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  // ROS interfaces
  ros::Subscriber stop_sub_;
  ros::Subscriber cancel_sub_;
  ros::Publisher success_pub_;
  ros::Publisher vel_pub_;
  ros::Timer run_timer_;
  tf2_ros::Buffer tfBuffer_;
  tf2_ros::TransformListener tfListener_;
  tf2_ros::TransformBroadcaster br_;
  ros::Subscriber odom_sub_;
  ros::ServiceServer pause_service_;
  ros::Publisher pose_pub_;

  // Bookkeeping
  bool task_cancelled_;
  bool run_;
  bool start_;
  bool turn_loop_;
  bool approach_loop_;
  std::string action_state_;
  double linear_vel_;
  bool log_printed_;
  bool odom_received_;
  bool start_final_approach_;
  bool parallel_approach_;
  nav_msgs::Odometry init_odom_;
  nav_msgs::Odometry current_odom_;
  geometry_msgs::TransformStamped current_goal_;
  bool goal_published_;

  // For recording xy coordinates of docking goal
  int history_index_;
  std::vector<double> x_history_;
  std::vector<double> y_history_;
  std::vector<double> yaw_sin_history_;
  std::vector<double> yaw_cos_history_;

  // ROS params
  bool p_two_phase_;
  int p_frames_tracked_;
  double p_init_xy_tolerance_;
  double p_final_xy_tolerance_;
  double p_yaw_tolerance_;
  double p_final_yaw_tolerance_;
  double p_max_linear_vel_;
  double p_min_linear_vel_;
  double p_max_turn_vel_;
  double p_min_turn_vel_;
  double p_reverse_;
  double p_linear_acc_;
  double p_final_approach_dist_;
  double p_max_yaw_diff_;
  double p_y_bounds_;
  std::string p_reference_frame_;
  double p_xy_update_tolerance_;
  double p_yaw_update_tolerance_;
  double p_transform_tolerance_;
  double p_loop_rate_;
  double p_action_delay_;
  double p_move_away_distance_;

  void initialize();
  bool loadParams();
  void setupTopics();
  
  // ROS callbacks
  void odomCb(const nav_msgs::Odometry odom);
  void stopCallback(const std_msgs::Empty::ConstPtr& msg);
  void startCallback(const std_msgs::Empty::ConstPtr& msg);
  void goalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
  bool pauseService(std_srvs::SetBool::Request& request, std_srvs::SetBool::Response& response);

  // Docking loop
  void runDocking(const ros::TimerEvent& event);

  // Cancel task
  void cancelSrvCb(const actionlib_msgs::GoalID::ConstPtr& msg);

  // Find average xy coordinates of docking goal based on past transforms
  bool historyAveraging(geometry_msgs::TransformStamped& goal);

  // Run when docking ended
  void cleanupTask(bool success);

  // Convert transform to pose
  void tfToPose(geometry_msgs::TransformStamped tf, geometry_msgs::PoseStamped& pose);

  // Go towards docking position
  void approachDock(double dtheta, double distance);//geometry_msgs::TransformStamped& goal, geometry_msgs::TransformStamped& init);

  // Go towards docking position with some y offset
  void parallelApproach(double dtheta, double distance);

  // Go away from docking position for pose correction
  void backAway(double dtheta);

  // On spot rotation
  void correctYaw(double dtheta);

  // Forward/backward motion
  void linearMotion(double distance, bool backwards);

public:
  DiffDriveDocking();
};

#endif
