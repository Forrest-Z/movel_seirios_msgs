#ifndef TASK_SUPERVISOR_NAVIGATION_HANDLER_H
#define TASK_SUPERVISOR_NAVIGATION_HANDLER_H

#include <ros/ros.h>

#include <task_supervisor/common.h>
#include <task_supervisor/plugins/task_handler.h>

#include <actionlib/client/simple_action_client.h>

#include <move_base_msgs/MoveBaseAction.h>
#include <std_msgs/Float64.h>
#include <std_srvs/SetBool.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/GetPlan.h>

#include <boost/thread/mutex.hpp>


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
  bool p_enable_best_effort_goal_;
  bool p_normal_nav_if_best_effort_unavailable_;
  // variables
  std::shared_ptr<actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>> nav_ac_ptr_;
  boost::mutex mtx_;
  bool enable_human_detection_;
  double human_detection_score_;
  bool task_cancelled_;
  geometry_msgs::Pose robot_pose_;
//   bool wait_for_robot_pose_;
  // topics/services
  ros::ServiceServer enable_human_detection_srv_;
  ros::ServiceServer enable_best_effort_goal_srv_;
  ros::ServiceClient make_plan_client_;
  ros::ServiceClient make_reachable_plan_client_;   // planner_utils
  ros::Subscriber human_detection_sub_;
  ros::Subscriber robot_pose_sub_;

  bool loadParams();
  bool start_ActionClient();
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
     * @brief Callback when navigation_handler/enable_best_effort_goal service is called
     */
  bool enableBestEffortGoalCB(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res);
  /**
     * @brief Callback on robot pose topic /pose
     */
  void robotPoseCB(const geometry_msgs::Pose::ConstPtr& msg);
  /**
     * @brief Loop during navigation to check for pause/resume
     */
  void navigationLoop(const move_base_msgs::MoveBaseGoal& goal);
  /**
     * @brief 
     */
  void navigationAttemptGoal(const move_base_msgs::MoveBaseGoal& goal);
  /**
     * @brief 
     */
  void navigationDirect(const geometry_msgs::Pose& goal);
  /**
     * @brief 
     */
  void navigationBestEffort(const geometry_msgs::Pose& goal);

public:
   
   NavigationHandler();
  ~NavigationHandler(){};

   /**
     * @brief Method called by task_supervisor when a navigation task is received
     * @param task Relevant task passed to handler by task supervisor
     * @param error_message Error message returned by this handler if execution fails
     * @return ReturnCode which indicates failure, cancellation or success
     */
  virtual ReturnCode runTask(movel_seirios_msgs::Task& task, std::string& error_message);

  bool setupHandler();
  void cancelTask();
  
};
}  // namespace task_supervisor

#endif
