#ifndef pebble_local_planner_h
#define pebble_local_planner_h

#include <dynamic_reconfigure/server.h>
#include <pluginlib/class_loader.hpp>
#include <nav_core/base_global_planner.h>
#include <nav_core/base_local_planner.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <std_srvs/SetBool.h>
// #include "pebble_local_planner/pebble_pid.h"
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
  ~PebbleLocalPlanner();

  bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel);
  bool isGoalReached();
  bool setPlan(const std::vector<geometry_msgs::PoseStamped>& plan);
  void initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros);

  bool loadParams();
  bool loadPlanner(const std::string& planner, costmap_2d::Costmap2DROS* costmap_ros);
  int decimatePlan(const std::vector<geometry_msgs::PoseStamped> &plan_in, std::vector<geometry_msgs::PoseStamped> &plan_out, std::vector<size_t> &idx_map, std::vector<std::vector<int>> curve_idx);
  bool getRobotPose(geometry_msgs::PoseStamped &robot_pose);
  int findIdxAlongPlan(geometry_msgs::PoseStamped &robot_pose, std::vector<geometry_msgs::PoseStamped> &plan, int start_idx=0);

  void calcVeloSimple(double xref, double yref, double thref, double dt, double &vx, double &wz, double distance_to_goal);
  bool adjustPlanForObstacles();

  bool planAheadForObstacles(int N); // plan for obstacles N pebbles ahead

  bool checkPebbleObstructed(int idx_pebble);

  void dynConfigCb(pebble_local_planner::pebble_local_plannerConfig &config, uint32_t level);

  bool lastwaypointCB(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res);

  void findCurve(std::vector<geometry_msgs::PoseStamped> plan_in);

  bool curveCheck();

private:
  // bookkeeping
  std::vector<geometry_msgs::PoseStamped> global_plan_;
  std::vector<geometry_msgs::PoseStamped> decimated_global_plan_;
  std::vector<size_t> idx_map_; // index mapping between global plan and decimated global plan
  std::vector<std::vector<int>> curve_global_idx_;
  std::vector<std::vector<int>> curve_decimate_idx_;
  int idx_plan_;
  std::string name_;
  bool close_enough_;
  tf2_ros::Buffer* tf_buffer_;
  // PID2D pid_;
  ros::Time prev_t_;
  double prev_vx_ = 0.;
  double prev_wz_ = 0.;
  bool goal_reached_;

  // params
  std::string inner_planner_;
  double d_min_;
  double curve_d_min_;
  std::string robot_frame_;
  std::string map_frame_;
  double xy_tolerance_;
  double th_tolerance_;
  double th_turn_, max_vx_, max_wz_, max_ax_, max_alphaz_;
  bool allow_reverse_;
  bool decelerate_goal_;
  bool at_last_goal_;
  double decelerate_distance_;
  double decelerate_factor_;
  bool decelerate_each_waypoint_;
  double th_reverse_;
  double curve_angle_tolerance_;
  double curve_vel_;
  bool at_curve_;
  bool local_obsav_;
  int N_lookahead_;
  double kpl_;
  double kil_;
  double kdl_;
  double kpa_;
  double kia_;
  double kda_;

  //Services
  ros::ServiceServer at_last_waypoint_srv_;

  // utility objects
  boost::shared_ptr<nav_core::BaseGlobalPlanner> planner_ptr_;
  std::shared_ptr<costmap_2d::Costmap2DROS> costmap_ptr_;
  pluginlib::ClassLoader<nav_core::BaseGlobalPlanner> bgp_loader_{"nav_core", "nav_core::BaseGlobalPlanner"};
  
  // publishers
  ros::Publisher decimated_path_pub_;
  ros::Publisher waypoint_pub_;

  // dynamic reconfigure
  std::string reconfg_inner_planner_{""};
  std::shared_ptr< dynamic_reconfigure::Server<pebble_local_plannerConfig> > dyn_config_srv;
  dynamic_reconfigure::Server<pebble_local_plannerConfig>::CallbackType dyn_config_cb;

};

}

#endif