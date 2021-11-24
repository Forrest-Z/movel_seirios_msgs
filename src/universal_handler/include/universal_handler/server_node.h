#ifndef UNIVERSAL_HANDLER_SERVER_NODE_H_
#define UNIVERSAL_HANDLER_SERVER_NODE_H_

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/simple_client_goal_state.h>
#include <flexbe_msgs/BehaviorExecutionAction.h>
#include <movel_seirios_msgs/RunTaskListAction.h>
#include <movel_seirios_msgs/TaskList.h>
#include <movel_seirios_msgs/UnifiedTaskAction.h>
#include <std_msgs/Bool.h>

class UniversalHandlerNode
{
public:
  UniversalHandlerNode(std::string name);
  ~UniversalHandlerNode(){};

private:
  bool loadParams();

  void executeCb(const movel_seirios_msgs::UnifiedTaskGoalConstPtr& goal);

  bool isGoalDone();
  
  bool isGoalCancelled();

  void resultReturnPreempted(std::string cancellation_message);

  void resultReturnFailure(std::string failure_message);

  void resultReturnSuccess(std::string success_message);

  void pauseCb(const std_msgs::Bool::ConstPtr& msg);

  void publishPauseStatus();

  void heartbeat(const ros::TimerEvent& e);

  // ac callbacks
  void tsDoneCb(const actionlib::SimpleClientGoalState& state,
                const movel_seirios_msgs::RunTaskListResultConstPtr& result);
  
  void tsActiveCb();

  void tsFeedbackCb(const movel_seirios_msgs::RunTaskListFeedbackConstPtr& feedback);

  void flexbeDoneCb(const actionlib::SimpleClientGoalState& state,
                    const flexbe_msgs::BehaviorExecutionResultConstPtr& result);
  
  void flexbeActiveCb();

  void flexbeFeedbackCb(const flexbe_msgs::BehaviorExecutionFeedbackConstPtr& feedback);


private:
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  actionlib::SimpleActionServer<movel_seirios_msgs::UnifiedTaskAction> as_;
  
  movel_seirios_msgs::UnifiedTaskFeedback feedback_;
  movel_seirios_msgs::UnifiedTaskResult result_;

  ros::Timer heartbeat_timer_;
  ros::Publisher heartbeat_pub_;
  ros::Publisher pause_status_pub_;
  ros::Subscriber pause_sub_;

  // service topics
  ros::Publisher ts_pause_pub_;
  ros::Publisher flexbe_pause_pub_;

  std::shared_ptr<actionlib::SimpleActionClient<movel_seirios_msgs::RunTaskListAction>> ts_ac_ptr_;
  std::shared_ptr<actionlib::SimpleActionClient<flexbe_msgs::BehaviorExecutionAction>> flexbe_ac_ptr_;

  int unified_task_id_;
  uint8_t unified_task_target_;
  bool paused_;
  bool cancelled_;
  std::string server_name_;
  actionlib::SimpleClientGoalState::StateEnum current_goal_state_;

  double p_loop_rate_;
  std::string p_ts_server_;
  std::string p_flexbe_server_;
  double p_ts_server_timeout_;
  double p_flexbe_server_timeout_;

};

#endif // UNIVERSAL_HANDLER_SERVER_NODE_H_