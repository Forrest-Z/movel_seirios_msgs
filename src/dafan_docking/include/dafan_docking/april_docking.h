#ifndef APRIL_DOCKING_H
#define APRIL_skip_phase_0_H

#include <math.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <dafan_msgs/BatteryStatus.h>
#include <dafan_msgs/ToggleDevices.h>
#include <dafan_docking/ToggleDocking.h>

#include <std_msgs/Int8.h>

#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <apriltag_ros/AprilTagDetectionArray.h>
#include <apriltag_ros/AprilTagDetection.h>

#include <dynamic_reconfigure/server.h>
#include <dafan_docking/april_dockingConfig.h>

class AprilDocking
{
public:
  AprilDocking();

private:
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;
  //////////////////////////////////////
  //Publishers, subscribers and services
  //////////////////////////////////////
  ros::Subscriber transform_april_sub_;
  ros::Subscriber dock_dist_sub_;
  ros::Subscriber reached_sub_;
  ros::Subscriber batt_sub_;
  ros::Publisher pose_pub_;
  ros::Publisher stop_pub_;
  ros::Publisher cmd_vel_pub_;
  ros::Publisher docking_status_pub_;
  ros::ServiceClient toggle_device_client_;
  ros::ServiceServer toggle_docking_server;

  //Transforms
  tf2_ros::TransformBroadcaster br_;
  tf2_ros::Buffer tfBuffer_;
  tf2_ros::TransformListener tfListener_;
  
  // Parameters : april markers
  std::string id_array1_;
  std::vector<int32_t> id_array_vec_;
  void genIDVectors(std::string& id_array, std::vector<int32_t>& id_array_vec);
  void transformDockGoal(const geometry_msgs::Pose& april_pose, 
                          const std::string& msg_frame, 
                          std::string april_pose_frame, std::string dock_frame,
                          double& goal_x_offset, double& goal_y_offset,
                          double& goal_yaw_offset,
                          geometry_msgs::PoseStamped& pose_saved);

  geometry_msgs::PoseStamped pose_;
  double goal_x_offset_now_, goal_y_offset_now_, goal_yaw_offset_now_;
  double goal_x_offset1_, goal_y_offset1_, goal_yaw_offset1_;

  // Parameters: dock distance
  double dist_to_dock_;
  double checkpoint_thresh1_;
  bool stage1a_, stage1b_;
  bool start_pub_;
  bool enable_docking_;
  bool april_detected_;
  bool skip_phase_0_;

  // Bookkeeping
  ros::Time t_start_;

  /**
   * INITIALIZATION
   */
  void initialize();
  bool loadParams();
  void setupTopics();

  /**
   * TIMED FUNCTIONS
   */
  ros::Timer pubGoalTimer;
  void publishDockGoal(const ros::TimerEvent& event);

  /** 
   * CALLBACKS
   */
  void transformAprilBundleCB(const apriltag_ros::AprilTagDetectionArray::ConstPtr& msg);
  void transformAprilBundleCB_Align(const apriltag_ros::AprilTagDetectionArray::ConstPtr& msg);
  void distDockCB(const std_msgs::Float32::ConstPtr& dock_dist);
  void goalReachedCB(const std_msgs::Bool& goal_status_);
  void reconfigureCB(dafan_docking::april_dockingConfig &config, uint32_t level);
  void battCB(const dafan_msgs::BatteryStatus& msg);
  bool start_dockingSRV(dafan_docking::ToggleDocking::Request &req,
                              dafan_docking::ToggleDocking::Response &res);

  /**
   * HELPER FUNCTIONS
   */
  void toggleCameraLED(const int& toggle_);

  void poseToTransform(const geometry_msgs::Pose& pose, 
                        geometry_msgs::TransformStamped& tf);
  void rotateQuaternionMsgRPY(geometry_msgs::Quaternion& q_msg, const float& r, const float& p, const float& y);
  void parseVVF(const std::string& input, std::string& error_return, std::vector<int>& result);
  void calcRunningAvg(const geometry_msgs::PoseStamped& pose_);

  //Dynamic reconfigure server
  std::shared_ptr<dynamic_reconfigure::Server<dafan_docking::april_dockingConfig>> dyn_server_;
  boost::recursive_mutex own_mutex_;
  dafan_docking::april_dockingConfig param_config;

};

#endif
