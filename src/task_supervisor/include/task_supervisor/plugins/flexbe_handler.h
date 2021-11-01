#ifndef TASK_SUPERVISOR_FLEXBE_HANDLER_H
#define TASK_SUPERVISOR_FLEXBE_HANDLER_H

#include <ros/ros.h>

#include <task_supervisor/plugins/task_handler.h>
#include <task_supervisor/common.h>

#include <actionlib/client/simple_action_client.h>
#include <actionlib_msgs/GoalID.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
#include <std_msgs/String.h>
#include <std_srvs/Trigger.h>
#include <flexbe_msgs/BehaviorExecutionAction.h>
#include <flexbe_msgs/BehaviorLog.h>
#include <flexbe_msgs/BEStatus.h>

namespace task_supervisor
{
class FlexbeHandler : public TaskHandler
{
public:
  FlexbeHandler();
  ~FlexbeHandler(){};

  /**
   * @brief Method called by task_supervisor when a navigation task is received
   * @param task Relevant task passed to handler by task supervisor
   * @param error_message Error message returned by this handler if execution fails
   * @return ReturnCode which indicates failure, cancellation or success
   */
  virtual ReturnCode runTask(movel_seirios_msgs::Task& task, std::string& error_message);

  /**
   * @brief Mark this handler as inactive and cancelled
   */
  void cancelTask();

  /**
   * @brief Pause behavior execution
   */
  void pauseTask();
  
  /**
   * @brief Resume behavior execution
   */
  void resumeTask();

private:
  /**
   * @brief Load parameters on setup of handler
   * @return Returns a boolean indicating success
   */
  bool loadParams();

  /**
   * @brief Setup handler method called by task_supervisor during initialization of plugin
   * @return Returns a boolean indicating success
   */
  bool setupHandler();

  /**
   * @brief Callback method for FlexBE log messages
   */
  void flexbeLogsCb(const flexbe_msgs::BehaviorLog::ConstPtr& msg);

  /**
   * @brief Callback method for FlexBE engine status
   */
  void flexbeStatusCb(const flexbe_msgs::BEStatus::ConstPtr& msg);

  /**
   * @brief Callback method for getting FLexBE current state's name
   */
  void flexbeCurrentStateCb(const std_msgs::String::ConstPtr& msg);

  /**
   * @brief Callback method for triggering REPEAT command on FlexBE (repeating current state from the beginning)
   * @return Returns a boolean indicating success
   */
  bool flexbeRepeatCommandCb(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);

  /**
   * @brief Initialise an action client to communicate with FlexBE actionserver
   * @return Returns a boolean indicating success
   */
  bool startActionClient();

  /**
   * @brief Send a behavior goal to FlexBE actionserver in order to start the behavior
   * @param behavior_goal The behavior that will be started
   * @return Returns a boolean indicating success
   */
  bool startBehavior(const flexbe_msgs::BehaviorExecutionGoal& behavior_goal);

  /**
   * @brief Stop the currently-running behavior if there is one
   */
  void stopBehavior();

  /**
   * @brief Main loop during behavior execution to check for pause/resume and cancel
   */
  bool behaviorLoop();

  // variables
  std::string current_behavior_;
  std::string current_active_state_;
  uint8_t behavior_status_;
  bool is_healthy_;

  // ROS params
  bool p_enable_flexbe_logs_;
  std::string p_flexbe_topics_prefix_;
  std::string p_flexbe_server_;
  double p_flexbe_server_timeout_;
  double p_flexbe_start_behavior_timeout_;

  // ROS comms
  std::shared_ptr<actionlib::SimpleActionClient<flexbe_msgs::BehaviorExecutionAction>> flexbe_ac_ptr_;
  ros::ServiceServer flexbe_repeat_srv_;
  ros::Subscriber flexbe_logs_sub_;
  ros::Subscriber flexbe_status_sub_;
  ros::Subscriber flexbe_current_state_sub_;
  ros::Publisher flexbe_pause_pub_;
  ros::Publisher flexbe_repeat_pub_;
};

} // namespace task_supervisor

#endif