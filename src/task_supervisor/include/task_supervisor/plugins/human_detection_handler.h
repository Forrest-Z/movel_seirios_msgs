#ifndef TASK_SUPERVISOR_HUMAN_DETECTION_HANDLER_H
#define TASK_SUPERVISOR_HUMAN_DETECTION_HANDLER_H

#include <task_supervisor/plugins/task_handler.h>
#include <std_msgs/Bool.h>
#include <std_srvs/Trigger.h>
#include <movel_seirios_msgs/Reports.h>
#include <std_srvs/SetBool.h>

namespace task_supervisor
{

class HumanDetectionHandler : public TaskHandler
{
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
   * @brief Callback for checking if all launched nodes are ready
   */
  bool onStatus(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);

  /**
   * @brief Method called by startDetectionCB. Starts human detection
   * @return Returns a boolean indicating success
   */
  bool startDetection();

  /**
   * @brief Method called by stopDetectionCB. Stops human detection
   * @return Returns a boolean indicating success
   */
  bool stopDetection();

  /**
   * @brief Callback when human_detection_handler/start service is called
   */
  bool startDetectionCB(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);

  /**
   * @brief Callback when human_detection_handler/stop service is called
   */
  bool stopDetectionCB(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);

  /**
   * @brief Callback for health check timer
   */
  void onHealthTimerCallback(const ros::TimerEvent& timer_event);
  bool onCheckHumanDetection(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);

  ros::Timer health_timer_;

  ros::ServiceServer start_srv_;
  ros::ServiceServer stop_srv_;
  ros::ServiceServer status_srv_;
  ros::ServiceServer human_detection_checker;

  std_msgs::Bool detecting_;
  unsigned int human_detection_launch_id_ = 0;

  // ROS params
  double p_timer_rate_;
  std::string p_human_detection_launch_package_;
  std::string p_human_detection_launch_file_;
  std::string p_start_log_msg_;
  std::string p_stop_log_msg_;
  std::string p_start_error_msg_;
  std::string p_stop_error_msg_;

public:
  /**
   * @brief Method called by task_supervisor when a human detection task is received
   * @param task Relevant task passed to handler by task supervisor
   * @param error_message Error message returned by this handler if execution fails
   * @return ReturnCode which indicates failure, cancellation or success
   */
  virtual ReturnCode runTask(movel_seirios_msgs::Task& task, std::string& error_message);
  HumanDetectionHandler();
  ~HumanDetectionHandler(){};
};
}

#endif
