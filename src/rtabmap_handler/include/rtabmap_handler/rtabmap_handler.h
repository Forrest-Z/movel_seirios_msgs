#ifndef RTABMAP_HANDLER_H
#define RTABMAP_HANDLER_H

#include <ros/ros.h>
#include <task_supervisor/plugins/task_handler.h>
#include <task_supervisor/common.h>
#include <std_srvs/Trigger.h>
#include <std_msgs/Bool.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Empty.h>
#include <rosgraph_msgs/Log.h>
#include <geometry_msgs/Twist.h>
#include <movel_seirios_msgs/Task.h>
#include <movel_seirios_msgs/StringTrigger.h>
#include <ros_utils/ros_utils.h>
#include <movel_seirios_msgs/Reports.h>
#include <boost/filesystem.hpp>  

namespace rtabmap_handler
{
class RtabmapHandler : public task_supervisor::TaskHandler
{
private:
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

  // Interal vars
  std::string path_;
  unsigned int mapping_launch_id_ = 0;
  bool saved_ = false;
  std::string map_name_;

  // ROS params
  double p_save_timeout_ = 0;
  double p_loop_rate_ = 0;
  std::string p_map_topic_;
  std::string p_mapping_launch_package_;
  std::string p_mapping_launch_file_;
  std::string p_rgb_topic_;
  std::string p_depth_topic_;
  std::string p_camera_info_topic_;

  ros::Publisher health_check_pub_;

public:
  /**
   * @brief Method called by task_supervisor when a mapping task is received
   * @param task Relevant task passed to handler by task supervisor
   * @param error_message Error message returned by this handler if execution fails
   * @return ReturnCode which indicates failure, cancellation or success
   */
  virtual task_supervisor::ReturnCode runTask(movel_seirios_msgs::Task& task, std::string& error_message);
  //virtual bool healthCheck();

  RtabmapHandler(){};
  ~RtabmapHandler(){};
};
}

#endif
