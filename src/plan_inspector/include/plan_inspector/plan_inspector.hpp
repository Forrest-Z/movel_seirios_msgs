#ifndef plan_inspector_hpp
#define plan_inspector_hpp

#include <cmath>
#include <ros/ros.h>
#include <signal.h>
#include <math.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TransformStamped.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <std_srvs/SetBool.h>
#include <std_msgs/String.h>
#include <rosgraph_msgs/Log.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib_msgs/GoalStatusArray.h>
#include <std_msgs/Bool.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <movel_seirios_msgs/ObstructionStatus.h>
#include <dynamic_reconfigure/DoubleParameter.h>
#include <dynamic_reconfigure/IntParameter.h>
#include <dynamic_reconfigure/BoolParameter.h>
#include <dynamic_reconfigure/Reconfigure.h>

using std::string;

class PlanInspector
{
public:
  PlanInspector();
  ~PlanInspector(){};

  bool setupTopics();
  bool setupParams();

private:
  // infrastructure
  ros::NodeHandle nh_;
  ros::Timer abort_timer_;
  ros::Timer control_timer_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_ear_;

  std::shared_ptr< actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> > nav_ac_ptr_;

  // parameters
  string action_server_name_;
  string plan_topic_;
  string costmap_topic_;
  string cmd_vel_topic_;
  string local_planner_;
  string config_topic_;

  int obstruction_threshold_;
  double clearing_timeout_;
  double control_frequency_;
  double stop_distance_;

  // dynamic reconfigure
  double frequency_temp_;
  double rotation_speed_;
  double osc_timeout_;
  int retries_temp_;
  bool rotate_behavior_temp_;
  bool clearing_rotation_temp_;
  double weight_obstacle_temp_;

  // bookkeeping
  bool enable_;
  geometry_msgs::Twist zero_vel_;
  bool have_plan_;
  bool have_costmap_;
  bool have_action_status_;
  bool use_teb_;
  bool task_pause_status_;
  nav_msgs::OccupancyGrid latest_costmap_;
  nav_msgs::Path latest_plan_;
  actionlib_msgs::GoalStatus latest_goal_status_;
  geometry_msgs::PoseStamped first_path_obs_;
  geometry_msgs::PoseStamped base_link_map_;
  geometry_msgs::PoseStamped first_path_map_;

  bool timer_active_;
  bool path_obstructed_;
  bool reconfigure_;
  bool stop_;
  bool rotate_fov_;
  bool align_;
  double error_;
  bool override_velo_;
  
  // subscribers
  ros::Subscriber plan_sub_;
  ros::Subscriber costmap_sub_;
  ros::Subscriber action_status_sub_;
  ros::Subscriber logger_sub_;
  ros::Subscriber pause_status_sub_;

  // publishers
  ros::Publisher zerovel_pub_;
  ros::Publisher action_cancel_pub_;
  ros::Publisher action_pause_pub_;
  ros::Publisher planner_report_pub_;
  ros::Publisher obstruction_status_pub_;

  // services
  ros::ServiceServer enable_sub_;

  ros::ServiceClient set_common_params_;
  ros::ServiceClient set_DWA_params_;
  ros::ServiceClient set_teb_params_;

  // callbacks
  void pathCb(nav_msgs::Path msg);
  void costmapCb(nav_msgs::OccupancyGrid msg);
  void abortTimerCb(const ros::TimerEvent& msg);
  void controlTimerCb(const ros::TimerEvent& msg);
  void actionStatusCb(actionlib_msgs::GoalStatusArray msg);
  bool enableCb(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res);
  void loggerCb(rosgraph_msgs::Log msg);

  // abstractions
  bool checkObstruction();
  void processNewInfo();
  void processNewInfo2();
  bool reconfigureParams(std::string op);
  void saveParams();
  double calculateDistance();
  double calculateDistance(geometry_msgs::Pose a, geometry_msgs::Pose b);
  bool checkPose();
  double angleFromVector();
  void getRobotPose();
  void pauseTask();
  void resumeTask();
  void pauseStatusCb(std_msgs::Bool msg);
  double calcYaw(geometry_msgs::Pose a, geometry_msgs::Pose b);
  bool getRobotPose(geometry_msgs::PoseStamped& pose);
};

#endif
