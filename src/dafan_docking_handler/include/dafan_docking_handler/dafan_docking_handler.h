#ifndef DAFAN_DOCKING_HANDLER_H
#define DAFAN_DOCKING_HANDLER_H

#include <ros/ros.h>
#include <task_supervisor/plugins/task_handler.h>
#include <task_supervisor/common.h>
#include <dafan_docking/ToggleDocking.h>
#include <movel_seirios_msgs/Reports.h>
#include <movel_seirios_msgs/Task.h>
#include <ros_utils/ros_utils.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Bool.h>

namespace dafan_docking_handler
{

class DafanDockingHandler: public task_supervisor::TaskHandler
{
private:
  ros::Subscriber docking_status_sub_;
  ros::ServiceClient start_docking_clt_;
  ros::Publisher stop_control_pub_;
  ros::Publisher health_report_pub_;

  std::string launch_pkg_;
  std::string launch_file_;
  double loop_rate_;
  bool docking_success_;
  bool localization_healthy_;

  int docking_launch_id_;

  // Setup parameters and topics
  bool setupHandler();

  // Load parameters
  bool loadParams();

  // Function to launch docking launch file
  void startDocking();

  void stopDocking();

  void tearDown();

  // Callbacks
  void dockingStatusCb(std_msgs::Int8 msg);
  void healthReportCb(movel_seirios_msgs::Reports msg);

public:
  DafanDockingHandler();

  virtual task_supervisor::ReturnCode runTask(movel_seirios_msgs::Task& task,
                                              std::string& error_message);

  virtual bool healthCheck();
};

}

#endif