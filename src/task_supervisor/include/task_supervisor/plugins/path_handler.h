#ifndef TASK_SUPERVISOR_PATH_HANDLER_H
#define TASK_SUPERVISOR_PATH_HANDLER_H

#include <ros/ros.h>

#include <task_supervisor/common.h>
#include <task_supervisor/plugins/task_handler.h>
#include <actionlib_msgs/GoalID.h>

#include <std_msgs/Float64.h>
#include <std_srvs/SetBool.h>
#include <boost/thread/mutex.hpp>
#include <nav_msgs/Path.h>
#include <std_msgs/Bool.h>
#include <movel_seirios_msgs/Reports.h>

namespace task_supervisor
{
class PathHandler : public TaskHandler
{
private:
  bool loadParams();

  /**
     * @brief Callback when navigation_handler/enable_human_detection service is called
     */
  bool enableHumanDetectionCB(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res);

  /**
     * @brief Callback on human detection score topic published by human_detection node
     */
  void humanDetectionCB(const std_msgs::Float64::ConstPtr& msg);

  /**
   *  @brief Callback for checking if all launched nodes are ready
   */
  bool onStatus(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);

  /**
   *  @brief Callback for when path load finishes path execution
   */
  void onPathStatus(const std_msgs::BoolConstPtr& msg);

  /**
   *  @brief Callback for waiting for robot pose message
   */
  void onPose(const geometry_msgs::Pose::ConstPtr& msg);

  /**
   *  @brief Cancel currently loaded path. Used by handler to exit current task
   *  @return Returns success of cancellation by path_load
   */
  bool cancelPath();

  /**
   *  @brief Pause currently loaded path. Used by handler to pause current task
   *  @return Returns success of pausing by path_load
   */
  bool pausePath();

  /**
   *  @brief Resume currently loaded path. Used by handler to resume current task
   *  @return Returns success of resuming by path_load
   */
  bool resumePath();

   /**
     * @brief Callback on localization reporting 
     */
  void locReportingCB(const movel_seirios_msgs::Reports::ConstPtr& msg);

  unsigned int path_load_launch_id_ = 0;
  bool path_load_started_;
  bool pose_received_;

  boost::mutex mtx_;
  bool enable_human_detection_;
  double human_detection_score_;
  ros::ServiceServer enable_human_detection_srv_;
  ros::Subscriber human_detection_sub_;
  ros::Subscriber loc_report_sub_;
  ros::Publisher health_check_pub_;
  // ROS params
  std::string p_path_load_launch_package_;
  std::string p_path_load_launch_file_;
  double p_loop_rate_;
  double p_human_detection_min_score_;
  std::string p_human_detection_topic_;
  std::string p_enable_human_detection_msg_;
  std::string p_disable_human_detection_msg_;

  bool isRunning_;
  bool isLocHealthy_;
  bool isPathHealthy_;
public:
  PathHandler();
  ~PathHandler(){};

  /**
    *  @brief Setup ROS params and interfaces
    */
  bool setupHandler();

  /**
     * @brief Method called by task_supervisor when a navigation task is received
     * @param task Relevant task passed to handler by task supervisor
     * @param error_message Error message returned by this handler if execution fails
     * @return ReturnCode which indicates failure, cancellation or success
     */
  virtual ReturnCode runTask(movel_seirios_msgs::Task& task, std::string& error_message);
  virtual bool healthCheck();
};
}  // namespace task_supervisor

#endif
