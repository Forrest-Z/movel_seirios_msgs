#ifndef TASK_SUPERVISOR_SERVER_NODE_H
#define TASK_SUPERVISOR_SERVER_NODE_H

#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <task_supervisor/server.h>
#include <movel_seirios_msgs/GetTaskType.h>

class TaskSupervisorNode
{
private:
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  actionlib::SimpleActionServer<movel_seirios_msgs::RunTaskListAction> as_;
  std::shared_ptr<task_supervisor::TaskSupervisor> task_supervisor_ptr_;

  movel_seirios_msgs::RunTaskListFeedback feedback_;
  movel_seirios_msgs::RunTaskListResult result_;
  task_supervisor::TaskSupervisor::Params task_supervisor_params_;

  int action_id_;
  bool stopped_;
  bool paused_;
  std::string server_name_;
  ros::Publisher pub_heartbeat_;

  ros::Subscriber sub_state_;
  ros::Timer heartbeat_timer_;
  ros::ServiceServer get_task_type_service_server_;
  ros::Subscriber pause_sub_;
  ros::Subscriber abort_action_sub_;
  ros::Publisher pause_pub_;

  /**
   * @brief Load parameters from yaml file
   * @return Returns a boolean indicating success
   */
  bool loadParams();

  /**
   * @brief Callback for when Task Supervisor receives a goal
   */
  void executeCB(const movel_seirios_msgs::RunTaskListGoalConstPtr& goal);

  /**
   * @brief Checks if any an Action Goal has been preempted or other conditions that might have a similar consequence
   * @return Returns a boolean indicating if an Action cancellation-like event has occurred.
   */
  bool isActionCancelled();

  /**
   * @brief Cancels latest active Action Goal
   * @param cancellation_message Message detailing context of cancellation
   */
  void cancelAction(std::string cancellation_message);

  /**
   * @brief Fails latest active Action Goal
   * @param failure_message Message detailing context of failure
   */
  void failAction(std::string failure_message);

  /**
   * @brief Gives latest active Action Goal a Success status
   */
  void completeAction();

  void HeartBeat(const ros::TimerEvent& e);

  bool getTaskTypeServiceCb(movel_seirios_msgs::GetTaskType::Request &req, movel_seirios_msgs::GetTaskType::Response &res)
  {
    res.task_type = task_supervisor_ptr_->getActiveTaskType();
    return true;
  }

  void pauseCb(const std_msgs::Bool::ConstPtr& msg);

  void abortActionCb(const std_msgs::String::ConstPtr& msg);

public:
  TaskSupervisorNode(std::string name);
  ~TaskSupervisorNode();
};
#endif
