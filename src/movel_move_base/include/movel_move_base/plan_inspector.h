#ifndef plan_inspector_h
#define plan_inspector_h

#include <cmath>
#include <ros/ros.h>
#include <signal.h>
#include <math.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/GetPlan.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <std_srvs/SetBool.h>
#include <std_srvs/Trigger.h>
#include <std_msgs/String.h>
#include <rosgraph_msgs/Log.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib_msgs/GoalStatusArray.h>
#include <std_msgs/Bool.h>
#include <tf/transform_listener.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <movel_seirios_msgs/ObstructionStatus.h>
#include <movel_seirios_msgs/GetTaskType.h>
#include <movel_seirios_msgs/StopReconfig.h>
#include <movel_seirios_msgs/RunTaskListActionResult.h>
#include <movel_seirios_msgs/RunTaskListActionGoal.h>
#include <dynamic_reconfigure/DoubleParameter.h>
#include <dynamic_reconfigure/IntParameter.h>
#include <dynamic_reconfigure/BoolParameter.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <dynamic_reconfigure/server.h>
#include <plan_inspector/PlanInspectorConfig.h>
#include <pluginlib/class_loader.hpp>
#include <nav_core/base_global_planner.h>

using std::string;

class PlanInspector
{
public:
  enum class BlockageType {
    PARTIAL,
    FULL,
    FAILED,
    NEW_GOAL, // pseudotype to indicate first path from a new goal
  };

  PlanInspector(tf2_ros::Buffer* tf);
  PlanInspector(tf2_ros::Buffer* tf, int obstruction_thresh, double partial_blockage_path_length_thresh);

  ~PlanInspector(){
    current_plan_->clear();
    if (current_plan_ != NULL)
      delete current_plan_;
  };

  bool setupTopics();
  bool setupParams();
  // goal in global frame
  // void setNewGoal(const geometry_msgs::PoseStamped& goal);
  void updateRobotPose(const geometry_msgs::PoseStamped& pose);
  BlockageType processNewPlan(const std::vector<geometry_msgs::PoseStamped>& new_plan, const geometry_msgs::PoseStamped& current_goal);
  // BlockageType processNewPlan();
  void processCurrentPath(std::vector<geometry_msgs::PoseStamped>* path, costmap_2d::Costmap2DROS* costmap);
  bool checkObstruction(const costmap_2d::Costmap2DROS& costmap, const geometry_msgs::PoseStamped& robot_pose, geometry_msgs::PoseStamped& obstruction_pos);

private:
  // infrastructure
  // ros::NodeHandle nh_;
  // ros::Timer abort_timer_;
  // ros::Timer control_timer_;
  // ros::Timer partial_blockage_timer_;
  tf2_ros::Buffer* tf_buffer_;
  // tf2_ros::TransformListener tf_ear_;

  std::shared_ptr< actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> > nav_ac_ptr_;

  // parameters
  int obstruction_threshold_;
  // double stop_distance_;
  // double clearing_timeout_;
  // double control_frequency_;
  // double stop_distance_;
  // double angular_tolerance_;
  // bool enable_replan_;
  bool allow_partial_blockage_replan_;
  double partial_blockage_path_length_threshold_;

  // dynamic reconfigure

  // bookkeeping
  // bool have_costmap_;
  // bool have_action_status_;
  // bool have_result_;
  // bool use_teb_;
  // bool use_pebble_;
  // bool use_obstacle_pebble_;
  // bool task_pause_status_;
  // bool internal_pause_trigger_;
  // nav_msgs::OccupancyGrid latest_costmap_;
  // nav_msgs::Path latest_plan_;
  // actionlib_msgs::GoalStatus latest_goal_status_;
  geometry_msgs::PoseStamped first_path_obs_;
  // geometry_msgs::PoseStamped base_link_map_;
  geometry_msgs::PoseStamped first_path_map_;
  // double target_yaw_;
  // bool yaw_calculated_;

  geometry_msgs::PoseStamped current_pose_;
  bool have_plan_;
  geometry_msgs::PoseStamped current_goal_;
  std::vector<geometry_msgs::PoseStamped> *current_plan_;
  // std::vector<geometry_msgs::PoseStamped> *previous_plan_;

  // bool timer_active_;
  // bool path_obstructed_;
  // bool reconfigure_;
  // bool stop_;
  // bool rotate_fov_;
  // bool align_;
  // double error_;
  // bool override_velo_;
  
  // bool terminal_state_;
  // std::string configuration_;
  // bool reconfigure_triggered_;
  // // subscribers
  // ros::Subscriber plan_sub_;
  // ros::Subscriber costmap_sub_;
  // ros::Subscriber action_goal_sub_;
  // ros::Subscriber action_status_sub_;
  // ros::Subscriber action_result_sub_;
  // ros::Subscriber logger_sub_;
  // ros::Subscriber pause_status_sub_;
  // ros::Subscriber odom_sub_;
  // // publishers
  // ros::Publisher zerovel_pub_;
  // ros::Publisher action_cancel_pub_;
  // ros::Publisher action_pause_pub_;
  // ros::Publisher planner_report_pub_;
  // ros::Publisher obstruction_status_pub_;
  // ros::Publisher partial_blockage_check_pub_;

  // // services
  // ros::ServiceServer enable_sub_;
  // ros::ServiceServer reconfig_srv_;
  // ros::ServiceServer enable_plan_;
  // ros::ServiceClient set_common_params_;
  // ros::ServiceClient set_DWA_params_;
  // ros::ServiceClient set_teb_params_,set_pebble_params_;
  // ros::ServiceClient task_supervisor_type_;
  // ros::ServiceClient make_sync_plan_client_;
  // ros::ServiceServer stop_obstacle_checker_;
  // ros::ServiceClient set_stop_obs_mb_;


  // // dynamic reconfigure for internal params
  // dynamic_reconfigure::Server<plan_inspector::PlanInspectorConfig> dyn_config_srv_;
  // dynamic_reconfigure::Server<plan_inspector::PlanInspectorConfig>::CallbackType dyn_config_cb_;

  double calculateDistance(geometry_msgs::Pose a, geometry_msgs::Pose b);

};

#endif
