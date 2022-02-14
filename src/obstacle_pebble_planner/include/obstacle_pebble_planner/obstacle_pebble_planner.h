#ifndef obstacle_pebble_planner_h
#define obstacle_pebble_planner_h

#include <dynamic_reconfigure/server.h>
#include <pluginlib/class_loader.hpp>
#include <nav_core/base_global_planner.h>
#include <nav_core/base_local_planner.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <std_srvs/SetBool.h>
// #include "obstacle_pebble_planner/pebble_pid.h"
#include "obstacle_pebble_planner/obstacle_pebble_plannerConfig.h"
#include <nav_core/base_local_planner.h>
#include <base_local_planner/local_planner_util.h>
#include <base_local_planner/local_planner_limits.h>

#include <base_local_planner/Position2DInt.h>
#include <base_local_planner/world_model.h>
#include <base_local_planner/costmap_model.h>
#include <movel_seirios_msgs/StopReconfig.h>

struct Pt
{
  double x;
  double y;
};

namespace obstacle_pebble_planner
{

class PebbleLocalPlanner : public nav_core::BaseLocalPlanner
{
public:
  PebbleLocalPlanner();
  ~PebbleLocalPlanner();

  bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel);
  bool isGoalReached();
  bool setPlan(const std::vector<geometry_msgs::PoseStamped>& plan);
  void initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros);
  bool reconfigureParams(std::string op);
  void saveParams();
  bool loadParams();
  int decimatePlan(const std::vector<geometry_msgs::PoseStamped> &plan_in, std::vector<geometry_msgs::PoseStamped> &plan_out, std::vector<size_t> &idx_map);
  bool getRobotPose(geometry_msgs::PoseStamped &robot_pose);
  int findIdxAlongPlan(geometry_msgs::PoseStamped &robot_pose, std::vector<geometry_msgs::PoseStamped> &plan, int start_idx=0);

  void calcVeloSimple(double xref, double yref, double thref, double dt, double &vx, double &wz);
  bool adjustPlanForObstacles();

  bool planAheadForObstacles(int N); // plan for obstacles N pebbles ahead
  double compute_max_line_cost(const tf2::Vector3& world_pos_0,const tf2::Vector3& world_pos_1);
  bool enableCb(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res);

  void dynConfigCb(obstacle_pebble_planner::obstacle_pebble_plannerConfig &config, uint32_t level);

private:
  // bookkeeping
  std::vector<geometry_msgs::PoseStamped> global_plan_;
  std::vector<geometry_msgs::PoseStamped> decimated_global_plan_;
  std::vector<size_t> idx_map_; // index mapping between global plan and decimated global plan
  int idx_plan_;
  std::string name_;
  bool close_enough_;
  tf2_ros::Buffer* tf_buffer_;
  // PID2D pid_;
  ros::Time prev_t_;
  double prev_vx_ = 0.;
  double prev_wz_ = 0.;
  bool goal_reached_;

  
  double frequency_temp_,osc_timeout_;
  bool low_obstacle_layer,obstacle_layer,reconfigureParams_state,revert_state;
  ros::ServiceClient set_common_params_,set_global_costmap, set_global_costmap_low,set_global_costmap_range;
  ros::ServiceClient reconfig_client;
  ros::ServiceServer enable_sub_;
  
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
  int N_lookahead_;
  float m_min_stop_dist;
  bool enable_obsctacle_check,re_plan;
  int count;
   ros::Time stoped_timer;
   double m_waiting_time;

  // utility objects
  boost::shared_ptr<nav_core::BaseGlobalPlanner> planner_ptr_;
  std::shared_ptr<costmap_2d::Costmap2DROS> costmap_ptr_;
  pluginlib::ClassLoader<nav_core::BaseGlobalPlanner> bgp_loader_{"nav_core", "nav_core::BaseGlobalPlanner"};

  // publishers
  ros::Publisher decimated_path_pub_;
  ros::Publisher waypoint_pub_;

  // dynamic reconfigure
  std::shared_ptr< dynamic_reconfigure::Server<obstacle_pebble_plannerConfig> > dyn_config_srv;
  dynamic_reconfigure::Server<obstacle_pebble_plannerConfig>::CallbackType dyn_config_cb;

};

}

#endif