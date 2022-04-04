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
  //ros::Subscriber battery_status_sub_;
  ros::Subscriber odom_sub_;
  //ros::ServiceClient start_charging_client_;
  //ros::ServiceClient stop_charging_client_;
  ros::ServiceClient external_process_client_;
  ros::Publisher health_check_pub_;
  ros::Publisher vel_pub_;
  //ros::ServiceClient run_client_;
  //ros::Subscriber success_sub_;
  ros::Subscriber internal_feedback_sub_;
  ros::Subscriber external_feedback_sub_;
  //ros::Publisher goal_pub_;
  ros::Publisher external_cancel_pub_;

  ros::ServiceClient enable_smoother_client_;
  //ros::ServiceClient smoother_status_client_;

  std::string launch_pkg_;
  std::string launch_file_;
  std::string camera_name_;
  double undocking_distance_;
  double undocking_speed_;
  //double battery_status_timeout_;
  double loop_rate_;
  bool dock_;
  //bool use_apriltag_;
  std::string odom_topic_;
  bool use_external_service_;
  std::string external_service_;
  std::string external_cancel_topic_;
  std::string external_topic_;
  std::string internal_topic_;
  bool enable_retry_;
  double retry_undocking_distance_;
  bool use_external_feedback_;

  int docking_launch_id_;
  bool healthy_;
  //bool charging_;
  //float charging_current_;
  //bool battery_status_received_;
  bool odom_received_;
  bool task_cancelled_;
  nav_msgs::Odometry odom_pose_;
  geometry_msgs::PoseStamped goal_pose_;
  //bool paused_;
  bool goal_received_;
  bool smoother_on_;
  bool external_process_running_;

  bool docking_success_;
  bool task_done_;
  bool docking_success_internal_;
  bool docking_success_external_;
  bool task_done_internal_;
  bool task_done_external_;
  //bool retry_;

  // Setup parameters and topics
  bool setupHandler();

  // Load parameters
  bool loadParams();

  // Start dock/undock task
  std::string startTask();

  // Function to launch docking launch file
  std::string startDock();

  // Undock task handling
  bool startUndock(bool retry);

  // Call services to start/stop charging
  //std::string stopCharging();
  //std::string startCharging();
  std::string externalProcess();
  
  // Calculate distance between 2 poses
  double calcDistance(nav_msgs::Odometry pose1, nav_msgs::Odometry pose2);

  // Callbacks
  void odomCb(nav_msgs::Odometry odom);
  //void batteryStatusCb(sensor_msgs::BatteryState msg);
  //void successCb(std_msgs::Bool success);
  void internalCb(std_msgs::Bool success);
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
