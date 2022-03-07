#ifndef TASK_SUPERVISOR_NAVIGATION_HANDLER_BASE_H
#define TASK_SUPERVISOR_NAVIGATION_HANDLER_BASE_H

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

#include <movel_seirios_msgs/Reports.h>
#include <movel_seirios_msgs/ObstructionStatus.h>

#include <boost/thread/mutex.hpp>
#include <std_srvs/Empty.h>

namespace task_supervisor
{
class NavigationHandlerBase : public TaskHandler
{
public:
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
  double p_best_effort_retry_timeout_sec_;
  double p_best_effort_retry_sleep_sec_;
  double p_smooth_transition_dist_;
  // variables
  std::shared_ptr<actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>> nav_ac_ptr_;
  boost::mutex mtx_;
  bool enable_human_detection_;
  double human_detection_score_;
  bool task_cancelled_;
  bool isHealthy_;
  geometry_msgs::Pose robot_pose_;
  geometry_msgs::Pose current_sub_goal_;
  std::vector<geometry_msgs::Pose> waypoints_;
  bool isLastWaypoint_;
  bool isObstructed_;
  // track why navigationLoop() exited
  enum class NavLoopResult {
    FINAL_WAYPOINT_REACHED,
    SUBGOAL_REACHED,
    SMOOTH_TRANSITION,
    FAILED,
  };
  // topics/services
  ros::ServiceServer enable_human_detection_srv_;
  ros::ServiceServer enable_best_effort_goal_srv_;
  ros::ServiceClient make_sync_plan_client_;   // planner_utils
  ros::ServiceClient make_reachable_plan_client_;   // planner_utils
  ros::Subscriber human_detection_sub_;
  ros::Subscriber robot_pose_sub_;
  ros::Subscriber loc_report_sub_;
  ros::Publisher movebase_cancel_pub_;
  ros::Publisher obstruction_status_pub_;

  /** 
    * @brief Method called when navigation task is received while human detection is enabled 
  */
  void startWithDetection();
  /** 
    * @brief Callback on human detection score topic published by human_detection node 
  */
  bool start_ActionClient();
  /** 
    * @brief Loop during navigation to check for pause/resume 
  */
  NavLoopResult navigationLoop();
  
  NavigationHandlerBase();
  ~NavigationHandlerBase() = default;   
  template <typename param_type>
  bool load_param_util(std::string param_name, param_type& output);
  bool loadParams();
  bool setupHandler();
  void humanDetectionCB(const std_msgs::Float64::ConstPtr& msg);
  bool enableHumanDetectionCB(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res);
  bool enableBestEffortGoalCB(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res);
  void robotPoseCB(const geometry_msgs::Pose::ConstPtr& msg);
  NavLoopResult navigationAttemptGoal();
  void navigationDirect();
  void navigationBestEffort();
  bool runTaskChooseNav(const std::vector<geometry_msgs::Pose>& goal_poses);   // (for multimap nav)
  bool runTaskChooseNav(const geometry_msgs::Pose& goal_pose);   // (for multimap nav)
  void cancelTask();
  void locReportingCB(const movel_seirios_msgs::Reports::ConstPtr& msg);
  void reportObstruction(bool status, const geometry_msgs::Pose& location);
  void navACSendGoal(const geometry_msgs::Pose& goal);
};


class CountdownTimer
{
private:
  double start_time_secs;
  double countdown_secs;

public:
  void start(double countdown_secs)
  {
    this->start_time_secs = ros::Time::now().toSec();
    this->countdown_secs = countdown_secs;
  }
  bool expired()
  {
    double time_delta_secs = ros::Time::now().toSec() - start_time_secs;
    return time_delta_secs > countdown_secs;
  }
};

}  // namespace task_supervisor

#endif
