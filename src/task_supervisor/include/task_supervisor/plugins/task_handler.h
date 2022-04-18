#ifndef TASK_SUPERVISOR_TASK_HANDLER_H
#define TASK_SUPERVISOR_TASK_HANDLER_H

#include <ros/ros.h>
#include <task_supervisor/common.h>
#include <movel_seirios_msgs/Task.h>
#include <atomic>

#include <movel_seirios_msgs/StartLaunch.h>
#include <movel_seirios_msgs/StopLaunch.h>
#include <movel_seirios_msgs/LaunchExists.h>
#include <movel_seirios_msgs/TaskHandlerFeedback.h>
#include <std_srvs/Trigger.h>

namespace task_supervisor
{
class TaskHandler
{
public:
  /**
   * @brief Sets up the handler and watchdog for the handler. Called on loading of plugins by task_supervisor
   * @param nh_supervisor Node handle of supervisor, used for creating this handler's relative node handle
   * @param name Name of this task handler
   * @return Returns a boolean indicating success
   */
  bool initialize(ros::NodeHandle nh_supervisor, std::string name, uint8_t task_type);
  virtual ~TaskHandler(){};

  /**
   * @brief Method called by task_supervisor when a task is received
   * @param task Relevant task passed to handler by task supervisor
   * @param error_message Error message returned by this handler if execution fails
   * @return ReturnCode which indicates failure, cancellation or success
   */
  virtual ReturnCode runTask(movel_seirios_msgs::Task& task, std::string& error_message);

  /**
   * @brief Mark this handler as inactive and cancelled
   */
  virtual void cancelTask();

  /**
   * @brief Get the result of the recently executed task
   * @param &error_message Reference to a string for storing the status messages of the task handler
   * @return ReturnCode Code result of execution. Either SUCCESS, CANCELLED, or FAILED
   */
  virtual ReturnCode getTaskResult(std::string& status_message);

  /**
   * @brief For checking whether this task has been parsed
   * @return Boolean true if task is parsed
   */
  bool isTaskParsed();

  /**
   * @brief For checking whether this task handler is still active
   * @return Boolean true if task is active
   */
  bool isTaskActive();

  bool isTaskPaused();

  virtual void pauseTask();
  virtual void resumeTask();

protected:
  TaskHandler(){};

  std::atomic<bool> task_active_;
  std::atomic<bool> task_parsed_;
  std::atomic<bool> task_paused_;
  ros::Time start_;
  ReturnCode code_ = ReturnCode::FAILED;
  std::string message_ = "";

  std::string name_;
  uint8_t task_type_;
  ros::NodeHandle nh_supervisor_;  // node handle in the task supervisor node namespace
  ros::NodeHandle nh_handler_;     // node handle in the handler plugin namespace

  ros::Publisher handler_feedback_pub_;

  ros::Timer watchdog_timer_;

  double p_watchdog_rate_ = 0;
  double p_watchdog_timeout_ = 0;

  /**
   * @brief Setup the watchdog for this task handler.
   *  If watchdog timeout is exceeded, this task is automatically failed.
   *  Can be set to 0 in task_supervisor's yaml config to disable watchdog
   */
  bool setupWatchdog();

  /**
   * @brief Virtual method for any task handlers to implement if there are specific setup to
   *  be done on the handler during initialization
   */
  virtual bool setupHandler()
  {
    return true;
  }

  /**
   * @brief Callback for watchdog timer. Checks how long this task handler has been active
   */
  void onWatchdogCallback(const ros::TimerEvent& watchdog_event);

  /**
   * @brief Called by runTask, used to execute handler specific tasks
   * @param task Relevant task received from task_supervisor
   * @param error_message Error message returned by this handler if execution fails
   * @return Boolean of success
   */
  virtual bool onRunTask(movel_seirios_msgs::Task& task, std::string& error_message)
  {
    return true;
  }

  /**
   * @brief Method to end this handler's current task internally
   *     Automatically sets internal code_ variable
   * @param success Boolean indicating whether the task execution was successful
   */
  virtual void setTaskResult(bool success);

  /**
   * @brief To set internal message_ variable
   * @param message String message to set
   */
  void setMessage(std::string message);

  /**
   * @brief Start launch file indicated in method arguments through service calls to launch_manager
   * @param package String pointing to package to be launched, for example 'task_supervisor'
   * @param launch_file String pointing to launch file, for example 'task_supervisor.launch'
   * @param args String of arguments that can be passed into the launch file, 'arg1:=arg1 arg2:=arg2'
   * @return Positive unsigned integer number specific to this launch file, 0 if launching failed
   */
  unsigned int startLaunch(std::string package, std::string launch_file, std::string args);

  /**
   * @brief Stop launch file with launch_id returned by startLaunch
   * @param launch_id Unsigned int returned by startLaunch
   * @return Boolean indiciating success. If launch_id did not correspond to a launch or launch is already dead,
   *     this method returns false
   */
  bool stopLaunch(unsigned int launch_id);

  /**
   * @brief Stop launch file with node cleanup
   * @param launch_id and list of nodes to be removed in this format: "/node1 /node2 ..."
   * @return Boolean indicatiing success
   */
  bool stopLaunch(unsigned int launch_id, std::string nodes);

  /**
   * @brief Check if the launch file is alive
   * @param launch_id Unsigned int returned by startLaunch
   * @return Boolean indicating existence of launch corresponding to the launch_id. Returns false if launch is dead
   */
  bool launchExists(unsigned int launch_id);

  /**
   * @brief Check if all launched nodes in the launch file are ready
   * @param launch_id Unsigned int returned by startLaunch
   * @return Boolean indicating whether all launched nodes are ready. Returns true when the nodes are ready.
   */
   bool launchStatus(unsigned int launch_id);

  /**
   * @brief Called by onWatchdogCallback, to check if the nodes are still on
   * @return true
   */
  virtual bool healthCheck()
  {
    return true;
  }

  /**
   * @brief To publish any message related to the handler or task currently run by the handler
   * @param feedback_message String message to send
   */
  void publishHandlerFeedback(std::string feedback_message);
};
} // end namespace task_supervisor


#endif
