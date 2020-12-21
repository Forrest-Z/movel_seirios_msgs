#ifndef map_transform_monitor_hpp
#define map_transform_monitor_hpp

#include <fstream>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <iostream>
#include <ros/ros.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

using std::string;
using std::cout;
using std::endl;

class MapTransformMonitor
{
public:
  MapTransformMonitor();
  ~MapTransformMonitor(){}

  bool setupTopics();
  bool setupParams();

private:
  // ROS infra
  ros::NodeHandle nh_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_ear_;
  tf2_ros::TransformBroadcaster tf_mouth_;

  // Parameters
  string orb_map_frame_;
  string orb_camera_frame_;
  string map_frame_;
  string camera_frame_;
  bool log_to_csv_;
  string log_file_path_;

  // Subscribers, Publishers
  ros::Subscriber orb_pose_sub_;

  // Bookkeeping
  tf2::Transform latest_map_transform_;
  bool have_map_transform_;

  // Callbacks
  void orbPoseCb(geometry_msgs::PoseStamped msg);

  // Helpers, Abstractions
}; 

#endif