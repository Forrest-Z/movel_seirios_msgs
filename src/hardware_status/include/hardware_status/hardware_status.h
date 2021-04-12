#ifndef HARDWARE_STATUS_H
#define HARDWARE_STATUS_H

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Image.h>
#include <ros_utils/ros_utils.h>
#include <ros/master.h>
#include <algorithm>
#include <movel_seirios_msgs/HardwareStates.h>
#include <movel_seirios_msgs/HardwareState.h>

// Hardware states
namespace States
{
enum State
{
  INACTIVE,
  ACTIVE
};
}
typedef States::State State;

class HardwareStatus
{
private:
  // Handle multi-lidar and multi-camera system
  std::map<std::string, ros::Subscriber> lidar_sub_map_;  
  std::map<std::string, ros::Subscriber> camera_sub_map_;

  // Keep track of hardware states
  State motor_state_;
  ros::Time motor_state_time_;  
  std::map<std::string, State> lidar_states_;
  std::map<std::string, ros::Time> lidar_states_time_;
  std::map<std::string, State> camera_states_;
  std::map<std::string, ros::Time> camera_states_time_;
  std::map<std::string, State> other_hardware_states_;

  // ROS params
  double loop_rate_;
  double reset_timeout_;
  std::string odom_topic_;
  std::vector<std::string> lidar_topics_;
  std::vector<std::string> camera_topics_;
  std::vector<std::string> other_hardware_;

  // ROS interfaces
  ros::Subscriber odom_sub_;
  ros::Subscriber lidar_sub_;
  ros::Subscriber camera_sub_;
  ros::Timer timer_;
  ros::ServiceServer get_srv_;
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  /**
   *  @brief Initialize node
   */
  void initialize();

  /**
   *  @brief Load ROS params
   */
  bool loadParams();

  /**
   *  @brief Check existence of nodes
   */
  void checkNodes();

  /**
   *  @brief Setup ROS callbacks
   */
  void setupTopics();
  void subscribeLidarTopics();
  void subscribeCameraTopics();

  void odomCallback(const nav_msgs::Odometry::ConstPtr& odom);
  void lidarCallback(const sensor_msgs::LaserScan::ConstPtr& scan, std::string topic_name);
  void cameraCallback(const sensor_msgs::Image::ConstPtr& image, std::string topic_name);
  bool getStatus(movel_seirios_msgs::HardwareStates::Request &req, movel_seirios_msgs::HardwareStates::Response &res);
  
  /**
   *  @brief Periodically reset state if necessary
   */
  void timerCallback(const ros::TimerEvent& e);

public:
  HardwareStatus();
};

#endif
