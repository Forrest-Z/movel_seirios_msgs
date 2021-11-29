#ifndef PALLET_DOCKING_H
#define PALLET_DOCKING_H

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Bool.h>
#include <ros_utils/ros_utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf/tf.h>
#include <move_base_msgs/MoveBaseActionResult.h>

class PalletDocking
{
private:
  bool init_;
  bool run_;
  bool odom_received_;
  bool status_received_;

  nav_msgs::Odometry current_odom_;
  nav_msgs::Odometry initial_odom_;
  geometry_msgs::PoseStamped current_goal_;
  geometry_msgs::PoseStamped final_goal_; 
  ros::Time start_time_;
  double distance_;

  // ROS params
  double loop_rate_;
  double docking_speed_;
  double docking_distance_;
  double xy_tolerance_;
  double yaw_tolerance_;
  bool backward_;
  bool undock_;
  bool use_move_base_;
  double retry_undocking_distance_;
  bool retry_;

  // ROS interfaces
  ros::Publisher status_pub_;
  ros::Publisher vel_pub_;
  ros::Publisher success_pub_;
  ros::Subscriber odom_sub_;
  ros::Subscriber current_goal_sub_;
  ros::Subscriber goal_sub_;
  ros::Subscriber start_sub_;
  ros::Subscriber mb_sub_;
  ros::Timer timer_;
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  /**
   *  @brief Initialize node
   */
  void initialize();

  /**
   *  @brief Load ROS params
   */
  bool loadParams();

  /**
   *  @brief Setup ROS callbacks
   */
  void setupTopics();

  void odomCb(nav_msgs::Odometry odom);
  void goalCb(geometry_msgs::PoseStamped goal);
  void currentGoalCb(geometry_msgs::PoseStamped goal);
  void startCb(std_msgs::Bool start);
  void mbCallback(const move_base_msgs::MoveBaseActionResult::ConstPtr &msg);

  /**
   *  @brief Main loop
   */
  void timerCallback(const ros::TimerEvent& e);
  
  double calcDistance(geometry_msgs::Pose pose1, geometry_msgs::Pose pose2);
  double calcAngleDiff(geometry_msgs::Pose init_pose, geometry_msgs::Pose target_pose);

public:
  PalletDocking();
};

#endif
