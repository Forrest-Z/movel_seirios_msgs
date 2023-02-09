#ifndef TASK_SUPERVISOR_CLEANING_HANDLER_H
#define TASK_SUPERVISOR_CLEANING_HANDLER_H

#include <task_supervisor/plugins/task_handler.h>
#include <std_msgs/Bool.h>
#include <std_srvs/Trigger.h>

// Includes for launch_manager services
#include <movel_seirios_msgs/StartLaunch.h>
#include <movel_seirios_msgs/StopLaunch.h>
#include <movel_seirios_msgs/LaunchExists.h>
#include <movel_seirios_msgs/StringTrigger.h>
#include <nav_msgs/Path.h>
#include <actionlib/client/simple_action_client.h>
#include <movel_seirios_msgs/Reports.h>

#include <yaml-cpp/yaml.h>

//for rosservice on path density reduction 
#include <movel_common_libs/json.hpp>
using json = nlohmann::json;

#include <nav_msgs/OccupancyGrid.h>

namespace task_supervisor
{
struct arg_flags
{
  bool run_now;
  bool use_name;
  std::string name;
  bool use_poly;
};

class CleaningHandler : public TaskHandler
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
   * @brief Callback for when path_load finishes path execution
   */
  void onPathStatus(const std_msgs::BoolConstPtr& msg);

  /**
   * @brief Callback for when coverage planner is complete planning
   */
  void plannerResultCB(const nav_msgs::PathConstPtr& path);

  /**
   * @brief Crop map by calling map cropper's service
   */
  void cropMap();

  /**
   * @brief Get path by calling coverage planner's service
   * @return Boolean indicating success of planning, timeout can be set
   */
  bool getPath();

  /**
   * @brief Start the path by calling path recall's load service
   * @return Boolean indicating success of path execution
   */
  bool startPath(std::string path_name);

  /**
   * @brief Check if the first point of generated path is close enough to robot's current position
   * @return Returns true if distance is within threshold, can be set in task_supervisor's yaml
   */
  bool checkPathWithinThresh(std::string path_name);

  /**
   * @brief Cancel currently loaded path. Used by handler to exit current task
   * @return Return success of cancellation by path_load
   */
  bool cancelPath();

  /**
   * @brief Stop all required launch files for this handler
   */
  void stopAllLaunch();

  /**
   * @brief Start all required launch files for this handler
   */
  void startAllLaunch();

  /**
   * @brief Parse arguments from payload, get polygon text file location and map location if provided
   */
  bool parseArgs(std::string payload, arg_flags& flags);


  //rosservice path density reduction feature
  bool pathDensityReductionCB(movel_seirios_msgs::StringTrigger::Request& req, movel_seirios_msgs::StringTrigger::Response& res);
  ros::ServiceServer start_path_density_reduction_;

  // add in resolution of map into parameter
  void mapCB (const nav_msgs::OccupancyGrid::ConstPtr& msg);
  ros::Subscriber map_resolution_subscriber_;

  // Launch IDs
  unsigned int path_recovery_id_ = 0;
  unsigned int path_load_id_ = 0;
  unsigned int path_saver_id_ = 0;
  unsigned int planner_server_id_ = 0;
  unsigned int planner_client_id_ = 0;
  unsigned int move_base_id_ = 0;

  std::string polygon_txt_;
  bool path_load_ended_ = false;
  bool path_planned_ = false;
  bool run_immediately_ = true;
  bool crop_fail_ = false;
  
  // ROS params
  std::string p_yaml_path_;
  std::string path_to_big_map_;
  std::string path_to_polygon_txt_;
  std::string path_to_cropped_map_;
  std::string path_to_coordinates_txt_;
  double robot_radius_ = 0;
  double p_radius_multiplier_ = 0;
  double p_planning_timeout_ = 0;
  double p_pose_distance_threshold_ = 0;
  double p_loop_rate_ = 0;
  double p_start_distance_thresh_ = 0;
  //double p_map_resolution_ = 0.05;
  std::string p_move_base_package_;
  std::string p_move_base_launch_;
  std::string p_planned_path_name_;
  
  
public:
  /**
   * @brief Method called by task_supervisor when a mapping task is received
   * @param task Relevant task passed to handler by task supervisor
   * @param error_message Error message returned by this handler if execution fails
   * @return ReturnCode which indicates failure, cancellation or success
   */
  virtual ReturnCode runTask(movel_seirios_msgs::Task& task, std::string& error_message);
  CleaningHandler(){};
  ~CleaningHandler(){};
  virtual bool healthCheck();
};
}

#endif
