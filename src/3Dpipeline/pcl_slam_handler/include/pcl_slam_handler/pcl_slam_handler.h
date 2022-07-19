#ifndef PCL_SLAM_HANDLER_H
#define PCL_SLAM_HANDLER_H

#include <ros/ros.h>
#include <task_supervisor/plugins/task_handler.h>
#include <task_supervisor/common.h>
#include <movel_seirios_msgs/Task.h>
#include <movel_seirios_msgs/StringTrigger.h>
#include <movel_seirios_msgs/Reports.h>
#include <pluginlib/class_list_macros.h>
#include <hdl_graph_slam/SaveMap.h>
#include <ros_utils/ros_utils.h>  //For loadParams function contents
#include <fstream>
#include <boost/filesystem.hpp>

namespace task_supervisor
{
class PCLSlamHandler : public task_supervisor::TaskHandler
{
private:
  /**
   * @brief Callback method for save_map service
   */
  bool onSaveServiceCall(movel_seirios_msgs::StringTrigger::Request& req,
                         movel_seirios_msgs::StringTrigger::Response& res);

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

  void healthTimerCb(const ros::TimerEvent& te);

  // Interal vars
  std::string path_;
  unsigned int pcl_slam_launch_id_ = 0;
  bool saved_ = false;
  std::string p_map_name_;
  
  // ROS params
  bool p_utm_ = false;
  double p_save_timeout_ = 0;
  double p_loop_rate_ = 0;
  double p_resolution_ = 0.0;
  std::string p_map_topic_;
  std::string p_pcl_slam_launch_package_;
  std::string p_pcl_slam_launch_;
  std::string p_pcl_slam_launch_nodes_;
  std::string p_map_saver_package_;
  std::string p_map_saver_launch_;
  std::string p_3Dto2D_package_;
  std::string p_3Dto2D_launch_;
  std::string p_rtabmap_pcl_slam_launch_package_;
  std::string p_rtabmap_pcl_slam_launch_;
  std::string p_rtabmap_pcl_slam_launch_nodes_;
  bool p_use_dynamic_2d_;
  ros::ServiceClient save_map_client_;
  ros::ServiceClient save_map_client_rtabmap_;
  bool p_use_rtabmap_;
  ros::Timer map_health_timer_;

  std::string map_name_save_;
  bool mapping_ = false;

public:
  /**
   * @brief Method called by task_supervisor when a mapping task is received
   * @param task Relevant task passed to handler by task supervisor
   * @param error_message Error message returned by this handler if execution fails
   * @return ReturnCode which indicates failure, cancellation or success
   */
  virtual task_supervisor::ReturnCode runTask(movel_seirios_msgs::Task& task, std::string& error_message);
  PCLSlamHandler(){};
  ~PCLSlamHandler(){};
};
}

#endif

