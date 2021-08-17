#ifndef TAG_OFFSET_H
#define TAG_OFFSET_H

#include <ros/ros.h>
#include <apriltag_ros/AprilTagDetectionArray.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

class TagOffset
{
private:
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;
  
  // ROS interfaces
  ros::Subscriber tag_sub_;
  tf2_ros::TransformBroadcaster br_;

  // ROS params
  int32_t p_tag_id_;
  double p_apriltag_x_offset_;
  double p_apriltag_y_offset_;

  void initialize();
  bool loadParams();
  void setupTopics();
  void tagCallback(const apriltag_ros::AprilTagDetectionArray::ConstPtr& msg);

public:
  TagOffset();
};

#endif
