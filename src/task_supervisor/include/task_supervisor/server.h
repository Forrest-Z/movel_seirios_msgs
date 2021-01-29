#ifndef TASK_SUPERVISOR_SERVER_H
#define TASK_SUPERVISOR_SERVER_H

#include <queue>
#include <atomic>

#include <ros/ros.h>

#include <task_supervisor/common.h>
#include <task_supervisor/plugins/task_handler.h>
#include <task_supervisor/plugins/task_expander.h>

#include <actionlib/server/simple_action_server.h>
#include <pluginlib/class_loader.h>

#include <movel_seirios_msgs/RunTaskListAction.h>
#include <movel_seirios_msgs/SetSpeed.h>

namespace task_supervisor
{
enum RUN_TASK_RETURN
{
  RUN_OK,
  VEHICLE_NOT_STOPPED,
  NO_PLUGIN_FOUND,
  RUN_STOPPED,
  RUN_ERROR,
  RUN_RESULT_ERROR
};
enum EXPAND_TASK_RETURN
{
  EXPAND_OK,
  EXPAND_ERROR
};
enum LOADER_RETURN
{
  LOAD_OK,
  INITIALIZE_FAIL,
  ALREADY_EXISTS,
  LOAD_FAIL
};
enum SUPERVISOR_STATE
{
  RUNNING,
  STOPPED,
  STOPPING,
  COMPLETED,
  PAUSED
};

class TaskSupervisor
{
public:
  struct Params
  {
    double p_loop_rate;
    double p_transition_timeout;
    XmlRpc::XmlRpcValue plugin_list;
    XmlRpc::XmlRpcValue expander_list;
    ros::NodeHandle server_nh;
    double p_default_linear_velocity;
    double p_default_angular_velocity;
  };

  TaskSupervisor(std::string name, Params param_list);
  ~TaskSupervisor();

  /**
   * @brief Load plugins
   * @return Returns an enum value
   */
  LOADER_RETURN loadPlugins();

  /**
   * @brief Load Expanders for various robots
   * @return Returns an enum value
   */
  LOADER_RETURN loadExpanders();

  RUN_TASK_RETURN runTask(movel_seirios_msgs::Task task);
  EXPAND_TASK_RETURN expandTask(movel_seirios_msgs::Task task, std::vector<movel_seirios_msgs::Task>& subtasks);

  boost::shared_ptr<task_supervisor::TaskHandler> getPlugin(uint16_t task_type);
  boost::shared_ptr<task_supervisor::TaskExpander> getExpander(uint16_t task_type);

  void startTaskList(const movel_seirios_msgs::RunTaskListGoalConstPtr& goal);
  void cancelSingleTask();
  void cancelAllTask();

  bool isActive();
  bool isPreempted();
  bool isStopped();
  bool isDone();
  bool getFeedback(movel_seirios_msgs::RunTaskListFeedback& feedback);

  ReturnCode getErrorCode();
  std::string getMsg();
  std::string errorMessage(RUN_TASK_RETURN error);
  std::string errorMessage(EXPAND_TASK_RETURN error);
  std::string errorMessage(LOADER_RETURN error);

  uint16_t getActiveTaskType();
  bool setVelocities(double linear_velocity, double angular_velocity);
  void revertVelocities();
  void pauseTask();
  void resumeTask();

private:
  std::atomic<SUPERVISOR_STATE> supervisor_state_;
  bool feedback_updated_ = false;

  std::string server_name_;
  std::string node_name_;
  ros::NodeHandle server_nh_;

  movel_seirios_msgs::RunTaskListFeedback feedback_;
  std::string action_id_;
  std::string error_msg_ = "";
  ReturnCode error_code_ = ReturnCode::SUCCESS;

  double p_loop_rate_;
  double p_transition_timeout_;
  XmlRpc::XmlRpcValue plugin_list_ = XmlRpc::XmlRpcValue();
  XmlRpc::XmlRpcValue expander_list_ = XmlRpc::XmlRpcValue();
  double p_default_linear_velocity_;
  double p_default_angular_velocity_;

  bool stopped_;
  ros::Subscriber sub_state_;

  std::shared_ptr<pluginlib::ClassLoader<task_supervisor::TaskHandler>> plugin_loader_ptr_;
  std::map<uint16_t, boost::shared_ptr<task_supervisor::TaskHandler>> plugins_;
  boost::shared_ptr<task_supervisor::TaskHandler> handler_ptr_;

  std::shared_ptr<pluginlib::ClassLoader<task_supervisor::TaskExpander>> expander_loader_ptr_;
  std::map<uint16_t, boost::shared_ptr<task_supervisor::TaskExpander>> expanders_;
  boost::shared_ptr<task_supervisor::TaskExpander> expander_ptr_;

  uint16_t active_task_type_;
};
}  // namespace task_supervisor
#endif
