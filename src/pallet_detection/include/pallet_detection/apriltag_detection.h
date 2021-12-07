/*
 *  Apriltag detection node takes in takes in apriltag tf and outputs docking pose.
 */

#ifndef APRILTAG_DETECTION_H
#define APRILTAG_DETECTION_H

#include <math.h>

// ROS
#include <ros/ros.h>
#include <tf/tf.h>
#include <tf2_ros/buffer.h>
#include <tf2/transform_datatypes.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Bool.h>
#include <move_base_msgs/MoveBaseActionResult.h>
#include <apriltag_ros/AprilTagDetectionArray.h>

class ApriltagDetection
{
private:
  //! For recording xy coordinates of docking goal
  int history_index_;
  std::vector<double> x_history_;
  std::vector<double> y_history_;

  bool goal_published_;
  bool status_received_;
  bool stages_complete_;
  bool next_stage_;
  geometry_msgs::PoseStamped recorded_goal_;

  //! Calculate euclidean distance
  double calcDistance(Eigen::Vector4f c1, Eigen::Vector4f c2);

  //! Calculate angle difference between 2 poses
  double calcAngleDiff(geometry_msgs::Pose init_pose, geometry_msgs::Pose target_pose);

  //! Convert tf to pose
  void tfToPose(geometry_msgs::TransformStamped tf, geometry_msgs::PoseStamped& pose);

  //! Find average dock position from all tracked frames
  void historyAveraging();

public:
  ApriltagDetection();

  //! Parameters to be loaded (refer to config file for details)
  int32_t tag_id_;
  double x_offset_;
  double y_offset_;
  double yaw_offset_;
  double goal_xy_tolerance_;
  double goal_yaw_tolerance_;
  int frames_tracked_;
  double inlier_dist_;
  double inlier_yaw_;
  bool use_move_base_;
  std::string apriltag_frame_;

  //! ROS interfaces
  ros::Publisher goal_pub_;
  ros::Publisher stop_pub_;
  ros::Publisher current_goal_pub_;
  tf2_ros::TransformBroadcaster br_;
  tf2_ros::Buffer tfBuffer_;
  tf2_ros::TransformListener tfListener_;
  
  //! Get apriltag detection input to output docking pose
  void tagCallback(const apriltag_ros::AprilTagDetectionArray::ConstPtr& msg);

  //! Get status from planner_adjuster
  void statusCallback(std_msgs::Bool success);

  //! move_base interface
  void mbCallback(const move_base_msgs::MoveBaseActionResult::ConstPtr &msg);

  //! dynamic reconfigure
  //void reconfigureCB(human_detection::human_detectionConfig &config, uint32_t level);
  
  bool loadParams();

  //dynamic_reconfigure::Server<human_detection::human_detectionConfig> configServer;
  //dynamic_reconfigure::Server<human_detection::human_detectionConfig>::CallbackType callbackType;

};

#endif
