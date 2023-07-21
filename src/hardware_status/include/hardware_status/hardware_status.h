#ifndef HARDWARE_STATUS_H
#define HARDWARE_STATUS_H

#include <ros/ros.h>
#include <ros_utils/ros_utils.h>
#include <ros/master.h>
#include <algorithm>
#include <string>
#include <map>
#include <vector>
#include <topic_tools/shape_shifter.h>
#include <movel_seirios_msgs/HardwareStates.h>
#include <movel_seirios_msgs/HardwareState.h>

namespace TopicStates
{
enum State
{
  INACTIVE,
  ACTIVE
};

struct TopicState {
  std::string topic_name;
  std::string topic_type;
  ros::Subscriber subscriber;
  ros::Time last_timestamp;
  State state;
};
}

typedef TopicStates::State State;
typedef TopicStates::TopicState TopicState;

class HardwareStatus
{
public:
  HardwareStatus(std::string name);
  ~HardwareStatus();

private:
  // The node's name for logging
  std::string name_;

  // Hardware states
  TopicState odom_state_;
  std::map<std::string, TopicState> lidar_states_;
  std::map<std::string, TopicState> camera_states_;
  std::map<std::string, TopicState> other_hardware_states_;

  // ROS params
  double loop_rate_;
  double reset_timeout_;
  std::string odom_topic_;
  std::vector<std::string> lidar_2d_topics_;
  std::vector<std::string> lidar_3d_topics_;
  std::vector<std::string> camera_topics_;
  std::vector<std::string> other_hardware_topics_;

  // ROS interfaces
  ros::Timer timer_;
  ros::ServiceServer get_srv_;
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  /**
   *  @brief Initialize node
   */
  void initAndRun();

  /**
   *  @brief Load ROS params
   */
  bool loadParams();

  /**
   *  @brief Setup ROS callbacks
   */
  void setupRosInterfaces();
  
  /**
   *  @brief Initialize a TopicState, subscribe to the given ROS topic name
   */
  TopicState initTopicState(std::string topic_name);
  
  /**
   *  @brief Initialize a map of TopicStates, subscribe to the given ROS topic names
   */
  void initTopicStateMap(std::vector<std::string> topic_names, std::map<std::string, TopicState>& topic_states);

  /**
   *  @brief Initialize a map of TopicStates, subscribe to the given ROS topic names
   */
  bool getStatusCb(movel_seirios_msgs::HardwareStates::Request &req, movel_seirios_msgs::HardwareStates::Response &res);

  /**
   *  @brief Update the state of a map of TopicStates according to the last timestamp and the set timeout
   */
  void updateTimeoutStatus(std::map<std::string, TopicState>& states);

  /**
   *  @brief Receive a message and update the corresponding TopicState
   */
  void topicCb(const topic_tools::ShapeShifter::ConstPtr& msg, const std::string& cb_topic_name);

  /**
   *  @brief Periodically reset state if necessary
   */
  void timerCb(const ros::TimerEvent& e);
};

#endif
