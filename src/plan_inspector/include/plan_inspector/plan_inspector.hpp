#ifndef plan_inspector_hpp
#define plan_inspector_hpp

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TransformStamped.h>
#include <std_msgs/Bool.h>
#include <actionlib_msgs/GoalStatusArray.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

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

  // parameters
  string action_server_name_;
  string plan_topic_;
  string costmap_topic_;
  string cmd_vel_topic_;
  int obstruction_threshold_;
  double clearing_timeout_;
  double control_frequency_;
  
  // bookkeeping
  bool enable_;
  geometry_msgs::Twist zero_vel_;
  bool have_plan_;
  bool have_costmap_;
  bool have_action_status_;
  nav_msgs::OccupancyGrid latest_costmap_;
  nav_msgs::Path latest_plan_;
  actionlib_msgs::GoalStatus latest_goal_status_;
  bool timer_active_;
  bool path_obstructed_;

  // subscribers
  ros::Subscriber plan_sub_;
  ros::Subscriber costmap_sub_;
  ros::Subscriber action_status_sub_;
  ros::Subscriber enable_sub_;

  // publishers
  ros::Publisher zerovel_pub_;
  ros::Publisher action_cancel_pub_;

  // callbacks
  void pathCb(nav_msgs::Path msg);
  void costmapCb(nav_msgs::OccupancyGrid msg);
  void abortTimerCb(const ros::TimerEvent& msg);
  void controlTimerCb(const ros::TimerEvent& msg);
  void actionStatusCb(actionlib_msgs::GoalStatusArray msg);
  void enableCb(std_msgs::Bool msg);

  // abstractions
  bool checkObstruction();
  void processNewInfo();
};

#endif