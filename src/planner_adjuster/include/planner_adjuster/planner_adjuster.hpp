#ifndef PLANNER_ADJUSTER_HPP
#define PLANNER_ADJUSTER_HPP

#include <iostream>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <planner_adjuster/simple_pid.hpp>

class PlannerAdjuster
{
public:
  PlannerAdjuster();
  ~PlannerAdjuster(){}

private:
  ros::NodeHandle nh_;
  SimplePID angle_PID_init;
  SimplePID angle_PID_final;
  SimplePID dist_PID;
  double dist_tolerance_;
  double angle_tolerance_;
  geometry_msgs::PoseStamped current_goal_;
  geometry_msgs::Pose latest_pose_;
  nav_msgs::Odometry latest_odom_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_ear_;

  ros::Publisher cmd_vel_pub_;
  ros::Publisher goal_status;
  ros::Publisher reached_pub_;
  ros::Subscriber goal_sub_;
  ros::Subscriber odom_sub_;
  ros::Subscriber stop_now_sub_;

  bool stop_check;
  bool dist_feasible;
  bool has_goal_;
  ros::Time t_prev_;
  int controller_stage_;

  bool getParams();
  bool setupTopics();
  void odometryCb(const nav_msgs::Odometry msg);
  void goalCb(const geometry_msgs::PoseStamped msg);
  void stopNowCb(const std_msgs::Bool msg);

  double calcDist(geometry_msgs::Pose a, geometry_msgs::Pose b);
  void doControl(geometry_msgs::Pose current_pose);
};

#endif
