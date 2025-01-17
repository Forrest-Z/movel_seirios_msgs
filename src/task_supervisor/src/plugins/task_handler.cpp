#include <task_supervisor/plugins/task_handler.h>


using json = nlohmann::json;

namespace task_supervisor
{
bool TaskHandler::initialize(ros::NodeHandle nh_supervisor, std::string name, uint8_t task_type)
{
  nh_supervisor_ = nh_supervisor;
  name_ = name;
  task_type_ = task_type;
  nh_handler_ = ros::NodeHandle(nh_supervisor_, name_);

  handler_feedback_pub_ = nh_handler_.advertise<movel_seirios_msgs::TaskHandlerFeedback>("/task_supervisor/handler_feedback", 1);
  health_report_pub_ = nh_handler_.advertise<movel_seirios_msgs::Reports>("/task_supervisor/health_report", 1);
  vel_pub_ = nh_handler_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);

  task_active_ = false;
  task_parsed_ = false;
  task_paused_ = false;

  if (!setupHandler())
  {
    ROS_FATAL("[%s] Error during handler setup. Shutting down.", name_.c_str());
    return false;
  }

  if (!setupWatchdog())
  {
    ROS_FATAL("[%s] Error setting up watchdog. Shutting down.", name_.c_str());
    return false;
  }

  return true;
}

ReturnCode TaskHandler::runTask(movel_seirios_msgs::Task& task, std::string& error_message)
{
  task_paused_ = false;
  task_parsed_ = false;
  start_ = ros::Time::now();
  message_ = "";

  if (onRunTask(task, error_message))
  {
    task_active_ = true;
    return ReturnCode::SUCCESS;
  }
  else
  {
    task_active_ = false;
    return ReturnCode::FAILED;
  }
}

bool TaskHandler::isTaskParsed()
{
  return task_parsed_;
}

bool TaskHandler::isTaskActive()
{
  return task_active_;
}

bool TaskHandler::isTaskPaused()
{
  return task_paused_;
}

void TaskHandler::cancelTask()
{
  message_ = "Task cancelled.";
  code_ = ReturnCode::CANCELLED;
  task_parsed_ = true;
  task_active_ = false;
  task_paused_ = false;
}

void TaskHandler::pauseTask()
{
  if (!task_paused_)
  {
    message_ = "Task paused.";
    task_paused_ = true;
  }
  else
    ROS_WARN("[%s] Task already paused", name_.c_str());
}

void TaskHandler::resumeTask()
{
  if (task_paused_)
  {
    message_ = "Task resumed.";
    task_paused_ = false;
  }
  else
    ROS_WARN("[%s] Task wasn't paused", name_.c_str());
}

ReturnCode TaskHandler::getTaskResult(std::string& status_message)
{
  //status_message = message_;
  return code_;
}

bool TaskHandler::setupWatchdog()
{
  if (!nh_handler_.getParam("watchdog_rate", p_watchdog_rate_))
  {
    ROS_ERROR("[%s] Failed to load parameter: watchdog_rate", name_.c_str());
    return false;
  }
  ROS_INFO("[%s] watchdog_rate: %f", name_.c_str(), p_watchdog_rate_);

  if (!nh_handler_.getParam("watchdog_timeout", p_watchdog_timeout_))
  {
    ROS_ERROR("[%s] Failed to load parameter: watchdog_timeout", name_.c_str());
    return false;
  }
  ROS_INFO("[%s] watchdog_timeout: %f", name_.c_str(), p_watchdog_timeout_);

  watchdog_timer_ =
      nh_handler_.createTimer(ros::Duration(1.0 / p_watchdog_rate_), &TaskHandler::onWatchdogCallback, this);

  return true;
}

void TaskHandler::onWatchdogCallback(const ros::TimerEvent& watchdog_event)
{
  if (!isTaskActive())
    return;
  else
    healthCheck();

  if (p_watchdog_timeout_ > 0. && ros::Time::now() - start_ > ros::Duration(p_watchdog_timeout_))
  {
    ROS_WARN("[%s] Watchdog timed out", name_.c_str());
    message_ = "Task timed out.";
    code_ = ReturnCode::FAILED;
    task_parsed_ = true;
    task_active_ = false;
    task_paused_ = false;
  }
}

void TaskHandler::setTaskResult(bool success)
{
  task_parsed_ = true;
  task_active_ = false;
  task_paused_ = false;
  code_ = success ? ReturnCode::SUCCESS : ReturnCode::FAILED;
}

void TaskHandler::setMessage(std::string message)
{
  message_ = message;
}

unsigned int TaskHandler::startLaunch(std::string package, std::string launch_file, std::string args)
{
  ros::ServiceClient start_launch_client =
      nh_handler_.serviceClient<movel_seirios_msgs::StartLaunch>("/launch_manager/start_launch");
  movel_seirios_msgs::StartLaunch launch_saver;
  launch_saver.request.package = package;
  launch_saver.request.launch_file = launch_file;
  launch_saver.request.args = args;

  // Check if service call is successful
  if (start_launch_client.call(launch_saver))
    return launch_saver.response.launch_id;

  else
    return 0;
}

bool TaskHandler::stopLaunch(unsigned int launch_id)
{
  ros::ServiceClient stop_launch_client =
      nh_handler_.serviceClient<movel_seirios_msgs::StopLaunch>("/launch_manager/stop_launch");
  movel_seirios_msgs::StopLaunch stop_launch;
  stop_launch.request.launch_id = launch_id;
  stop_launch_client.call(stop_launch);

  return stop_launch.response.success;
}

bool TaskHandler::stopLaunch(unsigned int launch_id, std::string nodes)
{
  ros::ServiceClient stop_launch_client =
      nh_handler_.serviceClient<movel_seirios_msgs::StopLaunch>("/launch_manager/stop_launch");
  movel_seirios_msgs::StopLaunch stop_launch;
  stop_launch.request.launch_id = launch_id;
  stop_launch.request.nodes = nodes;
  stop_launch_client.call(stop_launch);

  return stop_launch.response.success;
}

bool TaskHandler::launchExists(unsigned int launch_id)
{
  ros::ServiceClient launch_exists_client =
      nh_handler_.serviceClient<movel_seirios_msgs::LaunchExists>("/launch_manager/launch_exists");
  movel_seirios_msgs::LaunchExists launch_exists;
  launch_exists.request.launch_id = launch_id;
  launch_exists_client.call(launch_exists);

  return launch_exists.response.exists;
}

bool TaskHandler::launchStatus(unsigned int launch_id, bool publish_report)
{
  ros::ServiceClient launch_status_client = nh_handler_.serviceClient<movel_seirios_msgs::LaunchExists>("/launch_manager/launch_status");
  movel_seirios_msgs::LaunchExists launch_status;
  launch_status.request.launch_id = launch_id;

  // handle any potential exceptions that may occur during the execution of the service client code.
  try
  {
    launch_status_client.call(launch_status);
    if(!launch_status.response.message.empty())
    {
      publishZeroVelocity(); //publish 0 vel when facing unhealthy node, publishes before publishing the report

      if (publish_report)
      {
        ROS_ERROR("[%s] Error found: %s", name_.c_str(), launch_status.response.message.c_str());
        movel_seirios_msgs::Reports report;
        report.header.stamp = ros::Time::now();
        report.handler = name_;
        report.task_type = task_type_;
        report.healthy = false;
        report.message = launch_status.response.message;
        health_report_pub_.publish(report);
      }
    }
    return launch_status.response.exists;
  }
  catch (const std::exception& e)
  {
    ROS_ERROR("[%s] Service call failed with exception: %s", name_.c_str(), e.what());
    publishZeroVelocity();
    return false;
  }
}

void TaskHandler::publishHandlerFeedback(json feedback_payload)
{
  movel_seirios_msgs::TaskHandlerFeedback msg;
  msg.task_type = task_type_;
  msg.message = feedback_payload.dump();
  handler_feedback_pub_.publish(msg);
}

void TaskHandler::publishZeroVelocity()
{
  // std::cout << "[TaskHandler] publishing zero velocity" << std::endl;
  geometry_msgs::Twist cmd_vel;
  cmd_vel.linear.x = 0.0;
  cmd_vel.linear.y = 0.0;
  cmd_vel.angular.z = 0.0;
  vel_pub_.publish(cmd_vel);
}

} // end task_supervisor namespace
