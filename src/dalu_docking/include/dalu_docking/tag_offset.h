#ifndef TAG_OFFSET_H
#define TAG_OFFSET_H

#include <ros/ros.h>
#include <apriltag_ros/AprilTagDetectionArray.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <math.h>

#include <tf/tf.h>
#include <tf2_ros/buffer.h>
#include <tf2/transform_datatypes.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <string> 

#include <std_msgs/Empty.h>

class TagOffset
{
private:
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;
  
  // ROS interfaces
  ros::Subscriber start_sub_;
  ros::Subscriber tag_sub_;
  tf2_ros::TransformBroadcaster br_;
  tf2_ros::Buffer tfBuffer_;
  tf2_ros::TransformListener tfListener_;

  // ROS params
  int32_t p_tag_id_;
  double p_apriltag_x_offset_;
  double p_apriltag_y_offset_;
  bool p_reverse_;
  double p_apriltag_yaw_tolerance_;
  double p_transform_tolerance_;
  int p_tag_quantity_;

  // Bookkeeping
  bool start_;
  int selected_id_;
  
  void initialize();
  bool loadParams();
  void setupTopics();
  void tagCallback(const apriltag_ros::AprilTagDetectionArray::ConstPtr& msg);
  void startCallback(std_msgs::Empty msg);

public:
  TagOffset();
};

#endif
