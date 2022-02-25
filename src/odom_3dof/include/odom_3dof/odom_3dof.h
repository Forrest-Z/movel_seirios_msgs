#ifndef ODOM_3DOF_H
#define ODOM_3DOF_H

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <ros_utils/ros_utils.h>

#include <tf/tf.h>
#include <tf2/transform_datatypes.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

class Odom3dof
{
private:

  // ROS params
  std::string input_topic_;
  std::string output_topic_;
  std::string child_frame_id_;

  // ROS interfaces
  ros::Publisher odom_pub_;
  ros::Subscriber odom_sub_;
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;
  tf2_ros::TransformBroadcaster br_;

  /**
   *  @brief Initialize node
   */
  void initialize();

  /**
   *  @brief Load ROS params
   */
  bool loadParams();

  /**
   *  @brief Setup ROS callbacks
   */
  void setupTopics();

  /**
   *  @brief Process raw odom
   */
  void odomCallback(const nav_msgs::Odometry::ConstPtr& odom);

public:
  Odom3dof();
};

#endif