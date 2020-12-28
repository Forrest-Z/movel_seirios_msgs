#ifndef MOVE_BASE_STATE_H
#define MOVE_BASE_STATE_H

#include <ros/ros.h>
#include <rosgraph_msgs/Log.h>
#include <std_msgs/UInt8.h>
#include <ros_utils/ros_utils.h>

// move_base states
namespace States
{
enum State
{
  NORMAL,
  FAILED,
  RECOVERY,
  ABORTED
};
}
typedef States::State State;

class MoveBaseState
{
private:
  // For state tracking
  State prev_state_;
  State current_state_;
  bool init_;

  // For state reset
  ros::Time start_;

  // ROS params
  double loop_rate_;
  double failed_state_timeout_;
  double recovery_state_timeout_;
  double aborted_state_timeout_;
  std::string failed_msg_;
  std::string recovery_msg_;
  std::string aborted_msg_;

  // ROS interfaces
  ros::Publisher state_pub_;
  ros::Subscriber log_sub_;
  ros::Timer timer_;
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
   *  @brief Setup ROS callbacks
   */
  void setupTopics();

  /**
   *  @brief Get log messages from /rosout
   */
  void logCallback(const rosgraph_msgs::Log::ConstPtr& log);

  /**
   *  @brief Periodically publish state or reset state if necessary
   */
  void timerCallback(const ros::TimerEvent& e);

public:
  MoveBaseState();
};

#endif
