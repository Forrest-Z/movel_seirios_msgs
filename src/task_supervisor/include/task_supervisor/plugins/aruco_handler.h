#ifndef TASK_SUPERVISOR_ARUCO_HANDLER_H
#define TASK_SUPERVISOR_ARUCO_HANDLER_H

#include <task_supervisor/plugins/task_handler.h>
#include <std_msgs/Bool.h>
#include <std_srvs/Trigger.h>
#include <movel_seirios_msgs/Reports.h>
#include <std_srvs/SetBool.h>
#include <movel_seirios_msgs/StringTrigger.h>
namespace task_supervisor
{

class ArucoHandler : public TaskHandler
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
   * @brief Method called by startDetectionCB. Starts human detection
   * @return Returns a boolean indicating success
   */
  bool startArucoSaver();

  /**
   * @brief Method called by stopArucoCB. Stops human Aruco
   * @return Returns a boolean indicating success
   */
  bool stopAruco();

  // start ArucoACML
  bool startArucoAcml(std::string map_name_);


  bool saveArucoPose(std::string map_name_);

  /**
   * @brief Callback when human_Aruco_handler/start service is called
   */
  bool startArucoSaverCB(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);
  //call back to start ArucoAcml 
  bool startArucoAcmlCB(movel_seirios_msgs::StringTrigger::Request& req, movel_seirios_msgs::StringTrigger::Response& res);
/**
   * @brief Callback when Aruco_handler/save_pose service is called
   */
  bool saveArucoCB(movel_seirios_msgs::StringTrigger::Request& req, movel_seirios_msgs::StringTrigger::Response& res);

  /**
   * @brief Callback when Aruco_handler/stop service is called
   */
  bool stopArucoCB(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);

  /**
   * @brief Callback for health check timer
   */
  void onHealthTimerCallback(const ros::TimerEvent& timer_event);
  bool onCheckArucoChecker(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);


  ros::Publisher health_check_pub_;

  ros::Timer health_timer_;

  ros::ServiceServer start_saver_srv_;
  ros::ServiceServer stop_saver_srv_;
  ros::ServiceServer status_saver_srv_;
  ros::ServiceServer save_aruco_pose_;
  ros::ServiceServer aruco_checker_;
  ros::ServiceServer start_aruco_amcl_srv_;

  std_msgs::Bool detecting_;
  unsigned int aruco_launch_id_ = 0;

  // ROS params
  double p_timer_rate_;

  //aruco detect params used by both aruco acml and arcuo saver
  std::string p_aruco_launch_package_;
  std::string p_aruco_camera_topic_;
  std::string p_aruco_image_topic;
  std::string p_aruco_file_path;

  //aruco saver specific params
  std::string p_aruco_saver_launch_file_;
  
  //aruco amcl specific 
  std::string p_aruco_amcl_launch_file_;
public:
  /**
   * @brief Method called by task_supervisor when a human detection task is received
   * @param task Relevant task passed to handler by task supervisor
   * @param error_message Error message returned by this handler if execution fails
   * @return ReturnCode which indicates failure, cancellation or success
   */
  virtual ReturnCode runTask(movel_seirios_msgs::Task& task, std::string& error_message);
  ArucoHandler();
  ~ArucoHandler(){};
};
}

#endif
