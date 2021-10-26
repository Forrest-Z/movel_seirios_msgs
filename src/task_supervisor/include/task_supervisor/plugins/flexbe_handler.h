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
  ~FlexbeHandler();

  virtual ReturnCode runTask(movel_seirios_msgs::Task& task, std::string& error_message);

  void cancelTask();

  void pauseTask();
  
  void resumeTask();

private:
  bool loadParams();

  bool setupHandler();

  void flexbeLogsCb(const flexbe_msgs::BehaviorLog::ConstPtr& msg);

  void flexbeStatusCb(const flexbe_msgs::BEStatus::ConstPtr& msg);

  void flexbeCurrentStateCb(const std_msgs::String::ConstPtr& msg);

  bool flexbeRepeatCommandCb(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);

  bool startActionClient();

  bool behaviorLoop(const flexbe_msgs::BehaviorExecutionGoal& behavior_goal);

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