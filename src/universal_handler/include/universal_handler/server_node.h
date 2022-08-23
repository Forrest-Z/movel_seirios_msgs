#ifndef UNIVERSAL_HANDLER_SERVER_NODE_H_
#define UNIVERSAL_HANDLER_SERVER_NODE_H_

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/simple_client_goal_state.h>
#include <actionlib/server/simple_action_server.h>
#include <flexbe_msgs/BehaviorExecutionAction.h>
#include <movel_seirios_msgs/RunTaskListAction.h>
#include <movel_seirios_msgs/TaskList.h>
#include <movel_seirios_msgs/UnifiedTaskAction.h>
#include <movel_seirios_msgs/StringTrigger.h>
#include <std_msgs/Bool.h>

class UniversalHandlerNode
{
public:
  UniversalHandlerNode(std::string name);
  ~UniversalHandlerNode(){};

private:
  /**
   * @brief Load parameters from yaml file
   * @return Returns a boolean indicating success
   */
  bool loadParams();

  /**
   * @brief Callback method for when Universal Handler receives a goal
   */
  void executeCb(const movel_seirios_msgs::UnifiedTaskGoalConstPtr& goal);

  /**
   * @brief Whether latest active goal is in a terminal state (SUCCEEDED, PREEMPTED, ABORTED, RECALLED, REJECTED, LOST)
   * @return Returns true if latest active goal is in a terminal state, false otherwise
   */
  bool isGoalDone();

  /**
   * @brief Whether latest active goal is cancelled
   * @return Returns true if latest active goal is cancelled, false otherwise
   */
  bool isGoalCancelled();

  /**
   * @brief Cancel latest active goal (set status to PREEMPTED) then return result
   * @param cancellation_message Message detailing the context of cancellation
   */
  void resultReturnPreempted(std::string cancellation_message);

  /**
   * @brief Fail latest active goal (set status to ABORTED) then return result
   * @param failure_message Message detailing the context of failure
   */
  void resultReturnFailure(std::string failure_message);

  /**
   * @brief Set latest active goal status to SUCCESS then return result
   * @param success_message Optional message
   */
  void resultReturnSuccess(std::string success_message);

  /**
   * @brief Callback method for when Universal Handler receives a PAUSE command
   */
  void pauseCb(const std_msgs::Bool::ConstPtr& msg);

  /**
   * @brief Publish latest active goal state of pausing
   */
  void publishPauseStatus();

  /**
   * @brief Timer function to notify that the node is still alive
   */
  void heartbeat(const ros::TimerEvent& e);

  // ACTION CLIENT CALLBACKS
  // -- TASK SUPERVISOR --
  /**
   * @brief Callback method for task supervisor action client when latest active goal enters a terminal state
   */
  void tsDoneCb(const actionlib::SimpleClientGoalState& state,
                const movel_seirios_msgs::RunTaskListResultConstPtr& result);
  
  /**
   * @brief Callback method for task supervisor action client when latest active goal enters ACTIVE state
   */
  void tsActiveCb();

  /**
   * @brief Callback method for task supervisor action client when it receives a feedback from server
   */
  void tsFeedbackCb(const movel_seirios_msgs::RunTaskListFeedbackConstPtr& feedback);

  // -- FLEXBE --
  /**
   * @brief Callback method for flexbe action client when latest active goal enters a terminal state
   */
  void flexbeDoneCb(const actionlib::SimpleClientGoalState& state,
                    const flexbe_msgs::BehaviorExecutionResultConstPtr& result);
  
  /**
   * @brief Callback method for flexbe action client when latest active goal enters ACTIVE state
   */
  void flexbeActiveCb();

  /**
   * @brief Callback method for flexbe action client when it receives a feedback from server
   */
  void flexbeFeedbackCb(const flexbe_msgs::BehaviorExecutionFeedbackConstPtr& feedback);

  void cancelCb(const actionlib_msgs::GoalID::ConstPtr& msg);


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
  ros::Subscriber cancel_sub_;

  ros::ServiceClient redis_client_;

  // service topics
  ros::Publisher ts_pause_pub_;
  ros::Publisher flexbe_pause_pub_;
  ros::Publisher ts_cancel_pub_;
  ros::Publisher flexbe_cancel_pub_;

  std::shared_ptr<actionlib::SimpleActionClient<movel_seirios_msgs::RunTaskListAction>> ts_ac_ptr_;
  std::shared_ptr<actionlib::SimpleActionClient<flexbe_msgs::BehaviorExecutionAction>> flexbe_ac_ptr_;

  uint8_t unified_task_target_;
  bool paused_;
  bool cancelled_;
  std::string server_name_;
  actionlib::SimpleClientGoalState::StateEnum current_goal_state_;
  std::string result_message_;
  int completed_task_id_;

  std::string navigation_state_;

  double p_loop_rate_;
  std::string p_ts_server_;
  std::string p_flexbe_server_;
  std::string p_flexbe_cmd_ns_;
  double p_ts_server_timeout_;
  double p_flexbe_server_timeout_;
  bool p_use_flexbe_;

};

#endif // UNIVERSAL_HANDLER_SERVER_NODE_H_