#ifndef PALLET_DOCKING_HANDLER_H
#define PALLET_DOCKING_HANDLER_H

#include <ros/ros.h>
#include <task_supervisor/plugins/task_handler.h>
#include <task_supervisor/common.h>
#include <movel_seirios_msgs/Reports.h>
#include <movel_seirios_msgs/Task.h>
#include <ros_utils/ros_utils.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_srvs/Trigger.h>
#include <std_srvs/SetBool.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf/tf.h>
#include <actionlib_msgs/GoalID.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <std_msgs/Empty.h>
#include <apriltag_ros/AprilTagDetectionArray.h>

namespace pallet_docking_handler
{

class PalletDockingHandler: public task_supervisor::TaskHandler
{
private:
  ros::Subscriber tag_sub_;
  ros::Subscriber retry_sub_;
  ros::Subscriber pose_sub_;
  ros::Publisher health_check_pub_;
  ros::Publisher vel_pub_;
  ros::Subscriber success_sub_;
  ros::Subscriber success_sub2_;
  ros::Subscriber pallet_sub_;
  ros::Publisher cancel_pub_;
  ros::ServiceClient plan_inspector_client_;
  ros::ServiceClient teb_client_;

  // ROS params
  std::string launch_pkg_;
  std::string launch_file_;;
  double loop_rate_;
  bool dock_;
  bool use_move_base_;
  double spot_turn_vel_;
  double detection_timeout_;
  double xy_tolerance_;
  double yaw_tolerance_;
  std::string camera_name_;

  int docking_launch_id_;
  bool docking_success_;
  bool planner_fail_;
  bool healthy_;
  bool pose_received_;
  bool pallet_detected_;
  bool status_received_;
  bool task_cancelled_;
  bool task_done_;
  bool task2_done_;
  bool retry_;
  std::vector<double> default_values_;
  geometry_msgs::Pose pose_;
  bool paused_;

  // Setup parameters and topics
  bool setupHandler();

  // Load parameters
  bool loadParams();

  // Start dock/undock task
  std::string startTask();

  // Function to launch docking launch file
  std::string startDock();

  // Undock task handling
  std::string startUndock(bool retry);

  void spotTurn();
  
  // Calculate distance between 2 poses
  double calcDistance(geometry_msgs::Pose pose1, geometry_msgs::Pose pose2);

  double calcAngleDiff(geometry_msgs::Pose init_pose, geometry_msgs::Pose target_pose);

  void setConfigs(bool set);

  // Callbacks
  void retryCb(std_msgs::Empty msg);
  void poseCb(geometry_msgs::Pose pose);
  void successCb(std_msgs::Bool success);
  void successCb2(std_msgs::Bool success);
  void palletCb(visualization_msgs::Marker pallets);
  void tagCb(const apriltag_ros::AprilTagDetectionArray::ConstPtr& msg);

  void healthCheck(std::string error_message);

public:
  PalletDockingHandler();

  virtual task_supervisor::ReturnCode runTask(movel_seirios_msgs::Task& task,
                                              std::string& error_message);

  void cancelTask();
};

}

#endif
