#ifndef GENERAL_DOCKING_HANDLER_H
#define GENERAL_DOCKING_HANDLER_H

#include <ros/ros.h>
#include <task_supervisor/plugins/task_handler.h>
#include <task_supervisor/common.h>
#include <movel_seirios_msgs/Reports.h>
#include <movel_seirios_msgs/Task.h>
#include <ros_utils/ros_utils.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
#include <sensor_msgs/BatteryState.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_srvs/Trigger.h>
#include <std_srvs/SetBool.h>
#include <nav_msgs/Odometry.h>
#include <boost/algorithm/string/predicate.hpp>  //For case insensitive string comparison
#include <string.h>                              //Payload parsing

namespace general_docking_handler
{

class GeneralDockingHandler: public task_supervisor::TaskHandler
{
private:
  // ROS interfaces
  ros::Subscriber odom_sub_;
  ros::ServiceClient external_process_client_;
  ros::Publisher health_check_pub_;
  ros::Publisher vel_pub_;
  ros::Subscriber internal_feedback_sub_;
  ros::Subscriber external_feedback_sub_;
  ros::Publisher external_cancel_pub_;

  ros::ServiceClient pause_docking_client_;
  ros::ServiceClient enable_smoother_client_;
  ros::ServiceClient smoother_status_client_;

  // ROS params
  std::string launch_pkg_;
  std::string launch_file_;
  std::string camera_name_;
  double undocking_distance_;
  double undocking_speed_;
  double loop_rate_;
  bool dock_;
  std::string odom_topic_;
  bool use_external_service_;
  std::string external_service_;
  std::string external_cancel_topic_;
  std::string external_topic_;
  std::string internal_topic_;
  std::string pause_service_;
  std::string rack_args_;
  bool enable_retry_;
  bool use_external_feedback_;
  double feedback_timeout_;
  bool disable_smoother_;

  // General bookkeeping
  int docking_launch_id_;
  bool healthy_;
  bool odom_received_;
  bool task_cancelled_;
  nav_msgs::Odometry odom_pose_;
  geometry_msgs::PoseStamped goal_pose_;
  bool goal_received_;
  bool smoother_on_;
  bool external_process_running_;

  // End of docking bookkeeping
  bool docking_success_;
  bool task_done_;
  bool docking_success_internal_;
  bool docking_success_external_;
  bool task_done_internal_;
  bool task_done_external_;
  ros::Time start_wait_time_;
  bool waiting_;

  // Setup parameters and topics
  bool setupHandler();

  // Load parameters
  bool loadParams();

  // Start dock/undock task
  std::string startTask();

  // Function to launch docking launch file
  std::string startDock();

  // Undock task handling
  bool startUndock();

  // Trigger external process
  std::string externalProcess();
  
  // Calculate distance between 2 poses
  double calcDistance(nav_msgs::Odometry pose1, nav_msgs::Odometry pose2);

  // Get odom feedbaack
  void odomCb(nav_msgs::Odometry odom);

  // Get feedback from docking node
  void internalCb(std_msgs::Bool success);

  // Get external feedback during docking
  void externalCb(std_msgs::Bool success);
  
  void healthCheck(std::string error_message);

public:
  GeneralDockingHandler();

  virtual task_supervisor::ReturnCode runTask(movel_seirios_msgs::Task& task,
                                              std::string& error_message);

  void cancelTask();
};

}

#endif
