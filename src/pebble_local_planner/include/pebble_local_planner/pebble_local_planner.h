#ifndef pebble_local_planner_h
#define pebble_local_planner_h

#include <dynamic_reconfigure/server.h>
#include <global_planner/planner_core.h>
#include <nav_core/base_local_planner.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include "pebble_local_planner/pebble_pid.h"
#include "pebble_local_planner/pebble_local_plannerConfig.h"

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
  int decimatePlan(const std::vector<geometry_msgs::PoseStamped> &plan_in, std::vector<geometry_msgs::PoseStamped> &plan_out, std::vector<size_t> &idx_map);
  bool getRobotPose(geometry_msgs::PoseStamped &robot_pose);
  int findIdxAlongPlan(geometry_msgs::PoseStamped &robot_pose, std::vector<geometry_msgs::PoseStamped> &plan, int start_idx=0);

  void calcVeloSimple(double xref, double yref, double thref, double dt, double &vx, double &wz);
  bool adjustPlanForObstacles();

  void dynConfigCb(pebble_local_planner::pebble_local_plannerConfig &config, uint32_t level);

private:
  // bookkeeping
  std::vector<geometry_msgs::PoseStamped> global_plan_;
  std::vector<geometry_msgs::PoseStamped> decimated_global_plan_;
  std::vector<size_t> idx_map_; // index mapping between global plan and decimated global plan
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
  double th_reverse_;
  bool local_obsav_;

  // utility objects
  std::shared_ptr<global_planner::GlobalPlanner> planner_ptr_;
  std::shared_ptr<costmap_2d::Costmap2DROS> costmap_ptr_;

  // publishers
  ros::Publisher decimated_path_pub_;
  ros::Publisher waypoint_pub_;

  // dynamic reconfigure
  std::shared_ptr< dynamic_reconfigure::Server<pebble_local_plannerConfig> > dyn_config_srv;
  dynamic_reconfigure::Server<pebble_local_plannerConfig>::CallbackType dyn_config_cb;

};

}

#endif