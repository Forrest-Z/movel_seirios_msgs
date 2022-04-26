#ifndef POSE_OFFSET_H
#define POSE_OFFSET_H

#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

class PoseOffset
{
private:
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;
  
  // ROS interfaces
  ros::Timer pub_timer_;
  tf2_ros::TransformBroadcaster br_;

  // ROS params
  double p_offset_;
  bool p_reverse_;
  double p_x_;
  double p_y_;
  double p_z_;
  double p_w_;
  
  void initialize();
  bool loadParams();
  void setupTopics();
  void publishTF(const ros::TimerEvent& event);

public:
  PoseOffset();
};

#endif
