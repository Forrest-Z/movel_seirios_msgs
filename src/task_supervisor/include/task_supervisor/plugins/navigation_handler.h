#ifndef TASK_SUPERVISOR_NAVIGATION_HANDLER_H
#define TASK_SUPERVISOR_NAVIGATION_HANDLER_H

#include <ros/ros.h>

#include <task_supervisor/common.h>
#include <task_supervisor/plugins/task_handler.h>

#include <actionlib/client/simple_action_client.h>

#include <move_base_msgs/MoveBaseAction.h>
#include <std_msgs/Float64.h>
#include <std_srvs/SetBool.h>
#include <boost/thread/mutex.hpp>

#include <movel_seirios_msgs/Reports.h>

namespace task_supervisor
{
class NavigationHandler : public TaskHandler
{
private:
  // ROS params
  double p_server_timeout_;
  bool p_static_paths_;
  std::string p_navigation_server_;
  double p_human_detection_min_score_;
  std::string p_human_detection_topic_;
  std::string p_enable_human_detection_msg_;
  std::string p_disable_human_detection_msg_;

  std::shared_ptr<actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>> nav_ac_ptr_;

  bool loadParams();
  bool start_ActionClient();

  boost::mutex mtx_;
  bool enable_human_detection_;
  double human_detection_score_;
  bool task_cancelled_;
  bool isHealthy_;
  ros::ServiceServer enable_human_detection_srv_;
  ros::Subscriber human_detection_sub_;
  ros::Subscriber loc_report_sub_;

  /**
     * @brief Method called when navigation task is received while human detection is enabled
     * @param goal Navigation goal received
     */
  void startWithDetection(move_base_msgs::MoveBaseGoal goal);

  /**
     * @brief Callback when navigation_handler/enable_human_detection service is called
     */
  bool enableHumanDetectionCB(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res);

  /**
     * @brief Callback on human detection score topic published by human_detection node
     */
  void humanDetectionCB(const std_msgs::Float64::ConstPtr& msg);

  /**
     * @brief Loop during navigation to check for pause/resume
     */
  void navigationLoop(move_base_msgs::MoveBaseGoal goal);

   /**
     * @brief Callback on localization reporting 
     */
  void locReportingCB(const movel_seirios_msgs::Reports::ConstPtr& msg);

public:
  NavigationHandler();
  ~NavigationHandler(){};

  bool setupHandler();

  /**
     * @brief Method called by task_supervisor when a navigation task is received
     * @param task Relevant task passed to handler by task supervisor
     * @param error_message Error message returned by this handler if execution fails
     * @return ReturnCode which indicates failure, cancellation or success
     */
  virtual ReturnCode runTask(movel_seirios_msgs::Task& task, std::string& error_message);

  void cancelTask();
};
}  // namespace task_supervisor

#endif
