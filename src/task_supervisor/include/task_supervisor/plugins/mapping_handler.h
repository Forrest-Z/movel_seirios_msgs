#ifndef TASK_SUPERVISOR_MAPPING_HANDLER_H
#define TASK_SUPERVISOR_MAPPING_HANDLER_H

#include <task_supervisor/plugins/task_handler.h>
#include <movel_seirios_msgs/StringTrigger.h>
#include <movel_seirios_msgs/Reports.h>
#include <orb_slam2_ros/SaveMap.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
#include <rosgraph_msgs/Log.h>
#include <actionlib_msgs/GoalID.h>
#include <boost/thread/mutex.hpp>
#include <fstream>
#include <boost/filesystem.hpp>
#include <yaml_utils/yaml_utils.h>

namespace task_supervisor
{
class MappingHandler : public TaskHandler
{
private:

  /**
   * @brief function to copy map files (if use split map and use multi map) from source to destination folder
   */
  bool copyMapFiles(const std::string& source_folder_path, const std::string& destination_folder_path);

  /**
   * @brief  This function generates a MongoDB ObjectId string by combining a hexadecimal
   * representation of the current timestamp with a random 16-character hexadecimal
   * string. The ObjectId format is commonly used in MongoDB databases (seirios-backend and seirios-mongo)
   * to uniquely identify documents.
   * 
   * This mongo object id will replace the current filename of the files.
   *
   * @return A string representing a MongoDB ObjectId.
   */
  
  std::string mongo_object_id();

  /**
   * @brief Callback method for orbslam transform done
   */
  void orbTransCallback(std_msgs::Bool msg);

  /**
   * @brief Callback method for save_map service
   */
  bool onSaveServiceCall(movel_seirios_msgs::StringTrigger::Request& req, movel_seirios_msgs::StringTrigger::Response& res);

  /**
   * @brief Callback for checking if all launched nodes are ready
   */
  bool onStatus(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);

  /**
   * @brief Callback for async map saving, does not stop mapping
   */
  bool onAsyncSave(movel_seirios_msgs::StringTrigger::Request& req, movel_seirios_msgs::StringTrigger::Response& res);

  /**
   * @brief Callback for orb map restart, does not stop slam mapping
   */
  bool onOrbRestartServiceCall(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);

  /**
   * @brief Save map using map_saver
   */
  bool saveMap(std::string map_name);

  /**
   * @brief Method to start mapping, which is mapping_handler's main purpose
   * @return Returns a boolean indicating success
   */
  bool runMapping();

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

  bool healthCheck();

  void logCB(const rosgraph_msgs::LogConstPtr& msg);

  // Interal vars
  boost::mutex mtx_;
  std::string path_;
  unsigned int mapping_launch_id_ = 0;
  unsigned int orb_map_launch_id_ = 0;
  unsigned int orb_ui_launch_id = 0;
  unsigned int automap_launch_id_ = 0;
  bool saved_ = false;
  bool ui_done_ = false;
  bool sync_mode_ = false;
  bool mapping_launches_stopped_ = false;

  // ROS params
  bool p_orb_slam_;
  bool p_split_map_;
  bool p_save_split_map_to_library_;
  bool p_auto_;
  bool p_use_aruco_;

  double p_save_timeout_ = 0;
  double p_loop_rate_ = 0;
  std::string p_map_topic_;
  std::string p_mapping_launch_package_;
  std::string p_mapping_launch_file_;
  std::string p_mapping_launch_nodes_;

  std::string p_rgb_color_topic_;
  std::string p_rgbd_depth_topic_;
  std::string p_rgbd_camera_info_;

  std::string p_orb_map_launch_package_;
  std::string p_orb_map_launch_file_;  
  std::string p_orb_map_launch_nodes_;  
  std::string p_orb_ui_launch_nodes_;  

  ros::ServiceClient serv_orb_save_;
  ros::Subscriber orb_trans_ui_;
  ros::Publisher cancel_pub_;
  ros::Publisher stopped_pub_;

  std::string map_name_save_;
public:
  /**
   * @brief Method called by task_supervisor when a mapping task is received
   * @param task Relevant task passed to handler by task supervisor
   * @param error_message Error message returned by this handler if execution fails
   * @return ReturnCode which indicates failure, cancellation or success
   */
  virtual ReturnCode runTask(movel_seirios_msgs::Task& task, std::string& error_message);
  MappingHandler(){};
  ~MappingHandler(){};
};
}

#endif
