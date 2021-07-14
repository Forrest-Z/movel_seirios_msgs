#ifndef POINTCLOUD_MUX_H
#define POINTCLOUD_MUX_H

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Twist.h>

enum View
{
  FRONT,
  REAR,
  LEFT,
  RIGHT
};

class PointcloudMux
{
private:
  // ROS params
  std::string front_view_pointcloud_;
  std::string rear_view_pointcloud_;
  std::string left_view_pointcloud_;
  std::string right_view_pointcloud_;
  double angular_speed_threshold_;

  // ROS interfaces
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;
  ros::Publisher pointcloud_pub_;
  ros::Subscriber vel_sub_;
  ros::Subscriber front_sub_;
  ros::Subscriber rear_sub_;
  ros::Subscriber left_sub_;
  ros::Subscriber right_sub_;

  sensor_msgs::PointCloud2 front_pc_;
  sensor_msgs::PointCloud2 rear_pc_;
  sensor_msgs::PointCloud2 left_pc_;
  sensor_msgs::PointCloud2 right_pc_;
  View current_view_;

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
   *  @brief Subscribe to velocity commands for direction of motion
   */
  void velCallback(const geometry_msgs::Twist::ConstPtr& vel);
  
  /**
   *  @brief Subscribe to multiple camera pointclouds for front, rear, left and right views
   */
  void frontCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud);
  void rearCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud);
  void leftCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud);
  void rightCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud);
  
public:
  PointcloudMux();
};

#endif
