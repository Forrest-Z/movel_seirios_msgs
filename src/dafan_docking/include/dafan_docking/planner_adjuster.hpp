#ifndef PLANNER_ADJUSTER_HPP
#define PLANNER_ADJUSTER_HPP

#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>

#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <nav_msgs/Odometry.h>
#include <dafan_msgs/BatteryStatus.h>
#include <dafan_msgs/ToggleDevices.h>

#include <std_msgs/Int8.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <dafan_docking/simple_pid.hpp>
#include <dafan_docking/planner_adjusterConfig.h>

class PlannerAdjuster
{
public:
  PlannerAdjuster();
  ~PlannerAdjuster(){}
  
private:
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private;
  SimplePID angle_PID_init; //Stage 0: error being tracsked is the angular error
  SimplePID angle_PID_final;
  SimplePID dist_PID;
  //stored msgs
  geometry_msgs::PoseStamped current_goal_;
  geometry_msgs::Pose latest_pose_;
  nav_msgs::Odometry latest_odom_;
  geometry_msgs::Pose pose_phase3_;
  double prev_pose_th_;
  //Transform
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_ear_;

  //////////////////////////////////////
  //Publishers, subscribers and services
  //////////////////////////////////////
  ros::Publisher cmd_vel_pub_;
  ros::Publisher reached_pub_;
  ros::Publisher dock_dist_pub_;
  ros::Publisher docking_status_pub_;
  // ros::Publisher goal1_received_pub_;
  ros::Publisher goal2_received_pub_;
  ros::Subscriber goal_sub_;
  ros::Subscriber goal_sub2_;
  ros::Subscriber odom_sub_;
  ros::Subscriber stop_now_sub_;
  ros::Subscriber batt_sub_;
  ros::ServiceClient toggle_device_client_;

  //////////////////////
  //Flags for Controller
  //////////////////////
  bool stop_check;
  bool dist_feasible;
  bool has_goal_;
  ros::Time t_prev_;
  int controller_stage_;
  bool odom_received;
  bool started_phase3_;
  // bool orientate_yaw_;
  bool phase1;
  bool phase2;
  bool phase3;
  
  //params
  float dist_tol_;
  float angle_tol_;
  float final_angle_tol_;
  float reverse_speed_;
  bool disable_phase_2_;
  bool disable_phase_3_;

  ////////////////////////////////////////////////
  //  INITIALIZATION
  ////////////////////////////////////////////////
  bool getParams();
  bool setupTopics();

  ////////////////////////////////////////////////
  //  CALLBACKS
  ////////////////////////////////////////////////
  void odometryCb(const nav_msgs::Odometry msg);
  void goalCb(const geometry_msgs::PoseStamped msg);
  void goalCb2(const geometry_msgs::PoseStamped msg);
  void stopNowCb(const std_msgs::Bool msg);
  void battCB(const dafan_msgs::BatteryStatus& msg);  

  ////////////////////////////////////////////////
  //  PID Controller
  ////////////////////////////////////////////////
  void doControl(geometry_msgs::Pose current_pose);
  void doControl2(geometry_msgs::Pose current_pose);

  ////////////////////////////////////////////////
  //  Helper Functions
  ////////////////////////////////////////////////
  double calcDist(geometry_msgs::Pose a, geometry_msgs::Pose b);
  void stopNow();
  void toggleCameraLED(const int& toggle_);

  ////////////////////////////////////////////////
  //  Dynamic Reconfigure
  ////////////////////////////////////////////////
  std::shared_ptr<dynamic_reconfigure::Server<dafan_docking::planner_adjusterConfig>> dyn_server_;
  boost::recursive_mutex own_mutex_;
  dafan_docking::planner_adjusterConfig param_config;
  void reconfigureCB(dafan_docking::planner_adjusterConfig &config, uint32_t level);

};

#endif
