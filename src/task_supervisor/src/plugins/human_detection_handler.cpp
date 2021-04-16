#include <task_supervisor/plugins/human_detection_handler.h>
#include <pluginlib/class_list_macros.h>         //For pluginlib registration
#include <ros_utils/ros_utils.h>                 //For loadParams function contents
#include <boost/algorithm/string/predicate.hpp>  //For case insensitive string comparison
#include <ros/master.h>                          //For checking currently running nodes
#include <string.h>                              //Payload parsing
#include <stdio.h>                               //Check if file exists

PLUGINLIB_EXPORT_CLASS(task_supervisor::HumanDetectionHandler, task_supervisor::TaskHandler);

namespace task_supervisor
{
HumanDetectionHandler::HumanDetectionHandler()
{
  detecting_.data = false;
}

bool HumanDetectionHandler::onStatus(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
  res.success = launchStatus(human_detection_launch_id_);
  return true;
}

bool HumanDetectionHandler::startDetectionCB(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res)
{
  ROS_INFO("[%s] %s", name_.c_str(), p_start_log_msg_.c_str());
  movel_seirios_msgs::Task task;
  std::string error_message;
  task.type = 5;
  task.payload = std::string("start");

  if (runTask(task, error_message) == ReturnCode::SUCCESS)
    res.success = true;
  else
    res.success = false;

  res.message = message_;
  return true;
}

bool HumanDetectionHandler::stopDetectionCB(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res)
{
  ROS_INFO("[%s] %s", name_.c_str(), p_stop_log_msg_.c_str());
  movel_seirios_msgs::Task task;
  std::string error_message;
  task.type = 5;
  task.payload = std::string("stop");

  if (runTask(task, error_message) == ReturnCode::SUCCESS)
    res.success = true;
  else
    res.success = false;

  res.message = message_;
  return true;
}

bool HumanDetectionHandler::loadParams()
{
  ros_utils::ParamLoader param_loader(nh_handler_);

  param_loader.get_optional("watchdog_rate", p_timer_rate_, 2.0);
  param_loader.get_required("human_detection_launch_package", p_human_detection_launch_package_);
  param_loader.get_required("human_detection_launch_file", p_human_detection_launch_file_);
  param_loader.get_required("human_detection_start_log_msg", p_start_log_msg_);
  param_loader.get_required("human_detection_stop_log_msg", p_stop_log_msg_);
  param_loader.get_required("human_detection_start_error_msg", p_start_error_msg_);
  param_loader.get_required("human_detection_stop_error_msg", p_stop_error_msg_);

  return param_loader.params_valid();
}

bool HumanDetectionHandler::startDetection()
{
  if (!detecting_.data)
  {
    //Start human detection using launch_manager
    std::string args = "";
    human_detection_launch_id_ = startLaunch(p_human_detection_launch_package_, p_human_detection_launch_file_, args);
    if (!human_detection_launch_id_)
    {
      ROS_ERROR("[%s] %s", name_.c_str(), p_start_error_msg_.c_str());
      message_ = "Failed to launch human detection launch file";
      return false;
    }
  }
  detecting_.data = true;
  return true;
}

bool HumanDetectionHandler::stopDetection()
{
  if (!detecting_.data)
  {
    ROS_WARN("[%s] %s", name_.c_str(), p_stop_error_msg_.c_str());
    return false;
  }

  stopLaunch(human_detection_launch_id_);
  human_detection_launch_id_ = 0;
  detecting_.data = false;
  return true;
}

bool HumanDetectionHandler::setupHandler()
{
  if (!loadParams())
    return false;

  //Advertise service for starting and stopping human detection
  start_srv_ = nh_handler_.advertiseService("start", &HumanDetectionHandler::startDetectionCB, this);
  stop_srv_ = nh_handler_.advertiseService("stop", &HumanDetectionHandler::stopDetectionCB, this);
  status_srv_ = nh_handler_.advertiseService("status", &HumanDetectionHandler::onStatus, this);

  // Health Reporter
  health_check_pub_ = nh_handler_.advertise<movel_seirios_msgs::Reports>("/task_supervisor/health_report", 1);

  // Localization Timer
  health_timer_ = nh_handler_.createTimer(ros::Duration(1.0 / p_timer_rate_), &HumanDetectionHandler::onHealthTimerCallback, this);
  return true;
}

ReturnCode HumanDetectionHandler::runTask(movel_seirios_msgs::Task& task, std::string& error_message)
{
  task_active_ = true;
  task_parsed_ = false;
  start_ = ros::Time::now();

  //Check first arg of task payload to see if start or stop, uses boost case insensitive string compare
  bool result;
  if (boost::iequals(task.payload, "start"))
    result = startDetection();
  else if (boost::iequals(task.payload, "stop"))
    result = stopDetection();
  else
  {
    result = false;
    error_message = "[" + name_ + "] Payload command format invalid, input 'start' or 'stop'";
  }

  setTaskResult(result);
  return code_;
}

void HumanDetectionHandler::onHealthTimerCallback(const ros::TimerEvent& timer_event)
{
  if (detecting_.data)
  {
    bool isHealthy = launchStatus(human_detection_launch_id_);
    if (!isHealthy)
    {
      ROS_INFO("[%s] Some nodes are disconnected", name_.c_str());
      movel_seirios_msgs::Reports report;
      report.header.stamp = ros::Time::now();
      report.handler = "human_detection_handler";
      report.task_type = task_type_;
      report.healthy = false;
      report.message = "human detection nodes is not running";
      health_check_pub_.publish(report);
      stopDetection();
    } 
  }
}

}
