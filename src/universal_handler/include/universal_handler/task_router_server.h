#ifndef TASK_ROUTER_SERVER_H_
#define TASK_ROUTER_SERVER_H_

#include <ros/ros.h>
#include <actionlib_msgs/GoalID.h>
#include <actionlib_msgs/GoalStatus.h>
#include <actionlib_msgs/GoalStatusArray.h>
#include <movel_seirios_msgs/RunTaskListActionGoal.h>
#include <movel_seirios_msgs/RunTaskListActionResult.h>
#include <movel_seirios_msgs/StringTrigger.h>
#include <movel_seirios_msgs/SubtaskFeedback.h>
#include <movel_seirios_msgs/Task.h>
#include <movel_seirios_msgs/TaskFeedback.h>
#include <movel_seirios_msgs/TaskHandlerFeedback.h>
#include <movel_seirios_msgs/UniversalHandlerRunTask.h>
#include <universal_handler/json.hpp>
#include <flexbe_msgs/BehaviorExecutionActionGoal.h>
#include <flexbe_msgs/BehaviorExecutionActionResult.h>
#include <std_msgs/Bool.h>
#include <std_srvs/Trigger.h>
#include <map>

using json = nlohmann::json;

namespace UniversalHandler
{

class TaskRouterServer
{
public:
  TaskRouterServer(std::string name);
  ~TaskRouterServer(){};


private:
  bool runTrailCb(movel_seirios_msgs::UniversalHandlerRunTask::Request &req,
                  movel_seirios_msgs::UniversalHandlerRunTask::Response &res);

  bool runWaypointCb(movel_seirios_msgs::UniversalHandlerRunTask::Request &req,
                     movel_seirios_msgs::UniversalHandlerRunTask::Response &res);

  bool runFlexbeCb(movel_seirios_msgs::UniversalHandlerRunTask::Request &req,
                   movel_seirios_msgs::UniversalHandlerRunTask::Response &res);

  void cancelCb(const actionlib_msgs::GoalID::ConstPtr& msg);

  void pauseCb(const std_msgs::Bool::ConstPtr& msg);

  bool constructTrailTaskMsg(const json& payload, movel_seirios_msgs::RunTaskListActionGoal& msg, std::string& err_msg);
  
  bool constructWaypointTaskMsg(const json& payload, movel_seirios_msgs::RunTaskListActionGoal& msg, std::string& err_msg);

  bool constructFlexbeTaskMsg(const json& payload, flexbe_msgs::BehaviorExecutionActionGoal& msg, std::string& err_msg);

  void publishPauseStatus();

  void resetStates();

  void updateNavigationState(std::string navigation_state);

  // task supervisor
  void tsStatusCb(const actionlib_msgs::GoalStatusArray::ConstPtr& msg);

  void tsResultCb(const movel_seirios_msgs::RunTaskListActionResult::ConstPtr& msg);

  // void tsFeedbackCb(const movel_seirios_msgs::RunTaskListActionResult::ConstPtr& msg);

  void tsSubtaskFeedbackCb(const movel_seirios_msgs::TaskHandlerFeedback::ConstPtr& msg);

  // flexbe
  void fbStatusCb(const actionlib_msgs::GoalStatusArray::ConstPtr& msg);

  void fbResultCb(const flexbe_msgs::BehaviorExecutionActionResult::ConstPtr& msg);

  uint8_t getTaskStatus(const uint8_t& actionlib_status);

  // check statuses
  bool checkLocalizeStatus(std::string& error_msg);


private:
  std::string server_name_;

  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  ros::ServiceServer run_trail_srv_;
  ros::ServiceServer run_waypoint_srv_;
  ros::ServiceServer run_flexbe_srv_;

  ros::Subscriber cancel_sub_;
  ros::Subscriber pause_sub_;
  ros::Publisher pause_status_pub_;

  ros::Publisher task_feedback_pub_;
  ros::Publisher subtask_feedback_pub_;

  // redis
  ros::ServiceClient redis_client_;

  // check statuses
  ros::ServiceClient ts_loc_status_client_;

  // task supervisor
  ros::Publisher ts_run_task_pub_;
  ros::Publisher ts_cancel_pub_;
  ros::Publisher ts_pause_pub_;
  ros::Subscriber ts_status_sub_;
  ros::Subscriber ts_result_sub_;
  ros::Subscriber ts_subtask_feedback_sub_;

  // flexbe
  ros::Publisher fb_run_task_pub_;
  ros::Publisher fb_cancel_pub_;
  ros::Publisher fb_pause_pub_;
  ros::Subscriber fb_status_sub_;
  ros::Subscriber fb_result_sub_;

  // states
  bool cancelled_;

  // navigation state for UI
  std::string navigation_state_;

  struct TaskInformation
  {
    std::string id;
    std::string type;
    uint8_t actionserver_id;
    uint8_t feedback_status;
    bool paused;
  };
  // TODO: for built-in task queue functionality can change to std::queue/std::deque
  TaskInformation current_ts_task_;
  TaskInformation current_fb_task_;
}; // class TaskRouterServer

} // namespace UniversalHandler

#endif // TASK_ROUTER_SERVER_H_