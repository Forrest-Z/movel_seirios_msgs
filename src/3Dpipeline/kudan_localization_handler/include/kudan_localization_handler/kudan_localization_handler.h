#pragma once

#include <task_supervisor/plugins/task_handler.h>

// File System
#include <boost/filesystem.hpp>
#include <math.h>
// Callback Variables
#include <std_srvs/Trigger.h>
#include <std_srvs/Empty.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/SetMap.h>
#include <movel_seirios_msgs/StringTrigger.h>
#include <movel_seirios_msgs/Reports.h>
#include <rtabmap_ros_multi/LoadDatabase.h>
#include <actionlib_msgs/GoalID.h>
#include <dynamic_reconfigure/Config.h>  
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PointStamped.h>

// Bookeeping things
#include <geometry_msgs/TransformStamped.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Pose.h>

// TF Things
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf/transform_listener.h>

namespace task_supervisor
{

class KudanLocalizationHandler : public TaskHandler
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
   * @brief Method called by startLocalizationCB
   * @return Returns a boolean indicating success
   */
  bool startLocalization();

  /**
   * @brief Method called by stopLocalizationCB. Stops localization
   * @return Returns a boolean indicating success
   */
  bool stopLocalization();

  /**
   * @brief Callback when localization_handler/start service is called
   */
  bool startLocalizationCB(movel_seirios_msgs::StringTrigger::Request& req, movel_seirios_msgs::StringTrigger::Response& res);

  /**
   * @brief Callback when localization_handler/stop service is called
   */
  bool stopLocalizationCB(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);

  /**
   * @brief Callback on /map topic that should be published by SLAM node
   */
  void mapCB(const nav_msgs::OccupancyGrid::ConstPtr& map);

  /**
   * @brief Parse payload received from task_supervisor to determine whether to start or stop localization
   * @param payload Payload string received from task_supervisor
   * @return A vector of strings with the parsed arguments
   */
  std::vector<std::string> parseArgs(std::string payload);

  bool healthCheck();
  void healthTimerCb(const ros::TimerEvent& te);

  /**
   * @brief Callback when the map has been updated at map_editor
   */
  bool relaunchMapCb(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);


  /**  ---------------------------------------------------------
   *            POINT-BASED MAPPING PRIMITIVE FUNCTION 
   *   ---------------------------------------------------------
   * **/


  /**
   * @brief Calculate euclidean distance between two poses
   * @param p1 first pose
   * @param p2 second pose
   * */
  double euclideanDistance(const geometry_msgs::Pose & p1, const geometry_msgs::Pose &p2);

  /**
   * @brief Change quaternion to euler
   * @param p quaternion
   * @return euler euler
   * */
  double quaternionToYaw(const geometry_msgs::Quaternion &q);
  
  void costmapProhibCB(const dynamic_reconfigure::Config::ConstPtr& msg); 
  /**
   * @brief callback for costmap prohibition layer
   * 
   */

  void initialposeCB(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg); 

  // ROS
  ros::Publisher localizing_pub_;
  ros::Timer loc_health_timer_;
  ros::ServiceServer start_srv_serv_;
  ros::ServiceServer stop_srv_serv_;
  ros::ServiceServer status_srv_serv_;
  ros::ServiceServer relaunch_serv_;
  ros::ServiceClient set_map_client_;
  ros::ServiceClient clear_costmap_serv_;
  ros::ServiceClient prohib_layer_client_;
  ros::Subscriber map_subscriber_;
  ros::Subscriber prohib_layer_subscriber_;
  tf::TransformListener tf_listener_;
  ros::Publisher cancel_task_;
  ros::Publisher timeout_pub_;
  ros::Timer dynamic_timeout_;
  ros::Subscriber pose_sub_;
  ros::Subscriber initial_pose_sub;
  ros::Publisher clicked_points_pub_;

  // Rtabmap ros service 
  ros::ServiceClient update_params_;

  ros::Publisher initpose_pub_;

  std_msgs::Bool localizing_;
  bool start_localization_ = false;
  bool preempted_by_user_ = false;
  nav_msgs::OccupancyGrid map_;
  tf::StampedTransform tf_base_to_map_;
  geometry_msgs::PoseWithCovarianceStamped pose_;
  unsigned int localization_launch_id_ = 0;
  unsigned int loc_map_server_launch_id_ = 0;
  unsigned int nav_map_server_launch_id_ = 0;
  unsigned int move_base_launch_id_ = 0;
  unsigned int map_name_pub_id_ = 0;
  unsigned int map_editor_id_ = 0;
  unsigned int point_mapping_launch_id_ = 0;
  std::string loc_map_dir_ = "";
  std::string nav_map_dir_ = "";
  std::string nav_map_path_;
  std::string loc_map_path_;
  std::string map_name_;

  // ROS params
  double p_loop_rate_ = 0;
  double p_set_map_timeout_ = 0;
  std::string p_loc_map_topic_;
  std::string p_nav_map_topic_;
  std::string p_map_frame_;
  std::string p_base_link_frame_;
  std::string p_set_map_srv_;
  std::string p_localization_launch_package_;
  std::string p_localization_launch_file_;
  std::string p_localization_launch_nodes_;
  std::string p_navigation_launch_file_;
  std::string p_update_param_launch_file_;
  std::string p_move_base_launch_package_;
  std::string p_move_base_launch_file_;
  
  //Param to check if pcl_localization is launched
  bool pcl_localization_;

  // Map server launch
  std::string p_map_saver_package_;
  std::string p_map_saver_launch_;

  // Temp pose when called cancel dynamic mapping
  geometry_msgs::TransformStamped last_pose_map_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_ear_;

  // Point Based Mapping things
  bool isPBMapping_ = false;
  std::string p_map_name_;
  
public:
  /**
   * @brief Method called by task_supervisor when a mapping task is received
   * @param task Relevant task passed to handler by task supervisor
   * @param error_message Error message returned by this handler if execution fails
   * @return ReturnCode which indicates failure, cancellation or success
   */
  virtual ReturnCode runTask(movel_seirios_msgs::Task& task, std::string& error_message);
  KudanLocalizationHandler();
  ~KudanLocalizationHandler(){};
};
}

