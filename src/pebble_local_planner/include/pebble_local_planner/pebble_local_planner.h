#ifndef pebble_local_planner_h
#define pebble_local_planner_h

#include <nav_core/base_local_planner.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include "pebble_local_planner/pebble_pid.h"

struct Pt
{
  double x;
  double y;
};

namespace pebble_local_planner
{

class PebbleLocalPlanner : public nav_core::BaseLocalPlanner
{
public:
  PebbleLocalPlanner();
  ~PebbleLocalPlanner(){}

  bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel);
  bool isGoalReached();
  bool setPlan(const std::vector<geometry_msgs::PoseStamped>& plan);
  void initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros);

  bool loadParams();
  int decimatePlan(const std::vector<geometry_msgs::PoseStamped> &plan_in, std::vector<geometry_msgs::PoseStamped> &plan_out);
  bool getRobotPose(geometry_msgs::PoseStamped &robot_pose);
  int findIdxAlongPlan(geometry_msgs::PoseStamped &robot_pose, std::vector<geometry_msgs::PoseStamped> &plan, int start_idx=0);

  void calcVeloSimple(double xref, double yref, double thref, double dt, double &vx, double &wz);

private:
  // bookkeeping
  std::vector<geometry_msgs::PoseStamped> global_plan_;
  std::vector<geometry_msgs::PoseStamped> decimated_global_plan_;
  int idx_plan_;
  std::string name_;
  tf2_ros::Buffer* tf_buffer_;
  PID2D pid_;
  ros::Time prev_t_;
  double prev_vx_ = 0.;
  double prev_wz_ = 0.;
  bool goal_reached_;

  // params
  double d_min_;
  std::string robot_frame_;
  std::string map_frame_;
  double xy_tolerance_;
  double th_tolerance_;
  double th_turn_, max_vx_, max_wz_, max_ax_, max_alphaz_;
  bool allow_reverse_;

  // publishers
  ros::Publisher decimated_path_pub_;
  ros::Publisher waypoint_pub_;

};

}

#endif