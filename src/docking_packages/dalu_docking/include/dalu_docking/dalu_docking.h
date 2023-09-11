#ifndef DALU_DOCKING_H
#define DALU_DOCKING_H

#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <math.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
#include <tf/tf.h>
#include <std_srvs/SetBool.h>

class DaluDocking
{
private:
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  // ROS interfaces
  ros::Subscriber stop_sub_;
  ros::Subscriber start_sub_;
  ros::Publisher success_pub_;
  ros::Publisher vel_pub_;
  ros::Timer run_timer_;
  tf2_ros::Buffer tfBuffer_;
  tf2_ros::TransformListener tfListener_;
  ros::Subscriber pose_sub_;
  ros::ServiceServer run_service_;

  bool goal_received_;
  bool run_;
  bool x_loop_;
  bool y_loop_;
  bool yaw_loop_;
  geometry_msgs::PoseStamped goal_pose_;

  // For recording xy coordinates of docking goal
  int history_index_;
  std::vector<double> x_history_;
  std::vector<double> y_history_;

  // ROS params
  int p_frames_tracked_;
  double p_xy_tolerance_;
  double p_yaw_tolerance_;
  double p_max_linear_vel_;
  double p_min_linear_vel_;
  double p_max_turn_vel_;
  double p_min_turn_vel_;

  void initialize();
  bool loadParams();
  void setupTopics();
  
  // ROS callbacks
  void stopCallback(const std_msgs::Empty::ConstPtr& msg);
  void startCallback(const std_msgs::Empty::ConstPtr& msg);
  void goalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
  bool runService(std_srvs::SetBool::Request& request, std_srvs::SetBool::Response& response);

  // Docking loop
  void runDocking(const ros::TimerEvent& event);

  // Find average xy coordinates of docking goal based on past transforms
  bool historyAveraging(geometry_msgs::TransformStamped& goal);

  // Run when docking ended
  void cleanupTask(bool success);

  // Convert transform to pose
  void tfToPose(geometry_msgs::TransformStamped tf, geometry_msgs::PoseStamped& pose);

  // Get yaw from quaternion
  double getYaw(geometry_msgs::PoseStamped pose);

public:
  DaluDocking();
};

#endif
