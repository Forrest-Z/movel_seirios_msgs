#ifndef TASK_SUPERVISOR_MAPPING_HANDLER_H
#define TASK_SUPERVISOR_MAPPING_HANDLER_H

#include <std_srvs/Trigger.h>
#include <task_supervisor/plugins/task_handler.h>
#include <movel_seirios_msgs/StringTrigger.h>
#include <movel_seirios_msgs/Reports.h>

namespace task_supervisor {

class PointBasedMappingHandler : public TaskHandler {

public:
  /**
   * @brief Method called by task_supervisor when a mapping task is received
   * @param task Relevant task passed to handler by task supervisor
   * @param error_message Error message returned by this handler if execution fails
   * @return ReturnCode which indicates failure, cancellation or success
   */
  virtual ReturnCode runTask(movel_seirios_msgs::Task& task, std::string& error_message);
  PointBasedMappingHandler(){};
  ~PointBasedMappingHandler(){};

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
   * @brief Callback method for save_map service
   */
  bool onSaveServiceCall(movel_seirios_msgs::StringTrigger::Request& req, movel_seirios_msgs::StringTrigger::Response& res);

  /**
   * @brief Callback for async map saving, does not stop mapping
   */
  bool onAsyncSave(movel_seirios_msgs::StringTrigger::Request& req, movel_seirios_msgs::StringTrigger::Response& res);

  /**
   * @brief Save map using map_saver
   */
  bool saveMap(std::string map_name);

  /**
   * @brief Main logic of point based mapping handler
   * @return Returns a boolean indicating success
   */
  bool runPointBasedMapping();

  bool healthCheck();

  bool onStatus(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);

  /* Stop point based mapping */
  bool onStopCall(movel_seirios_msgs::StringTrigger::Request& req, movel_seirios_msgs::StringTrigger::Response& res);

  // Internal vars
  std::string path_;
  unsigned int pb_mapping_launch_id_ = 0;
  bool mapping_started_ = false;
  bool saved_ = false;

    // ROS params
  double p_save_timeout_ = 0;
  double p_loop_rate_ = 0;
  std::string p_map_topic_;
  std::string p_mapping_launch_package_;
  std::string p_mapping_launch_file_;
  std::string p_mapping_launch_nodes_;
  std::string p_map_dir_;
  std::string map_path_;
  ros::ServiceServer serv_status_;
  ros::ServiceServer serv_save_;
  ros::ServiceServer serv_save_async_;
  ros::ServiceServer serv_cancel_;
};
}
#endif