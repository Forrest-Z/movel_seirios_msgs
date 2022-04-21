#include <task_supervisor/plugins/aruco_handler.h>
#include <pluginlib/class_list_macros.h>         //For pluginlib registration
#include <ros_utils/ros_utils.h>                 //For loadParams function contents
#include <boost/algorithm/string/predicate.hpp>  //For case insensitive string comparison
#include <ros/master.h>                          //For checking currently running nodes
#include <string.h>                              //Payload parsing
#include <stdio.h>                               //Check if file exists
#include <movel_seirios_msgs/StringTrigger.h>

PLUGINLIB_EXPORT_CLASS(task_supervisor::ArucoHandler, task_supervisor::TaskHandler);

namespace task_supervisor
{
ArucoHandler::ArucoHandler()
{
  detecting_.data = false;
}

bool ArucoHandler::onStatus(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
  res.success = launchStatus(aruco_launch_id_);
  return true;
}

bool ArucoHandler::startArucoSaverCB(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res)
{
  ROS_INFO("[%s] start aruco saver service called", name_.c_str());
  movel_seirios_msgs::Task task;
  std::string error_message;
  task.type = 38;
  task.payload = std::string("start_aruco_saver");

  if (runTask(task, error_message) == ReturnCode::SUCCESS)
    res.success = true;
  else
    res.success = false;

  res.message = message_;
  return true;
}

bool ArucoHandler::startArucoAcmlCB(movel_seirios_msgs::StringTrigger::Request& req, movel_seirios_msgs::StringTrigger::Response& res)
{
  ROS_INFO("[%s] %s", name_.c_str(), "Aruco Acml starting...");
  std::string map_name_ = req.input;
  res.success = startArucoAcml(map_name_);
  res.message = message_;
  return true;
}

bool ArucoHandler::stopArucoCB(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res)
{
  ROS_INFO("[%s] stop aruco handler service called", name_.c_str());
  movel_seirios_msgs::Task task;
  std::string error_message;
  task.type = 38;
  task.payload = std::string("stop_aruco_handler");

  if (runTask(task, error_message) == ReturnCode::SUCCESS)
    res.success = true;
  else
    res.success = false;

  res.message = message_;
  return true;
}


bool ArucoHandler::saveArucoCB(movel_seirios_msgs::StringTrigger::Request& req, movel_seirios_msgs::StringTrigger::Response& res){
 ROS_INFO("[%s] Aruco pose saving", name_.c_str());
  std::string map_name_ = req.input;
  res.success = saveArucoPose(map_name_);
  res.message = message_;
  return true;
}


bool ArucoHandler::loadParams()
{
  ros_utils::ParamLoader param_loader(nh_handler_);

//general parameter loading for aruco handler
  param_loader.get_optional("watchdog_rate", p_timer_rate_, 2.0);
  param_loader.get_required("aruco_launch_package", p_aruco_launch_package_);
  param_loader.get_required("aruco_detect_camera_info_topic", p_aruco_camera_topic_);
  param_loader.get_required("aruco_detect_image_topic_compressed", p_aruco_image_topic);
  param_loader.get_required("aruco_loc_map_path", p_aruco_file_path);


//aruco saver specific parameter loading
  param_loader.get_required("aruco_saver_launch_file", p_aruco_saver_launch_file_);

  //aruco acml parameter loading
  param_loader.get_required("aruco_acml_launch_file", p_aruco_amcl_launch_file_);


  return param_loader.params_valid();
}

bool ArucoHandler::startArucoSaver()
{
  if (!detecting_.data)
  {
    //Start human detection using launch_manager
    std::string args = " camera_info_topic:="+ p_aruco_camera_topic_+" image_topic_compressed:=" +p_aruco_image_topic;
    aruco_launch_id_ = startLaunch(p_aruco_launch_package_, p_aruco_saver_launch_file_, args);
    ROS_INFO("%s %s %s %i" ,p_aruco_launch_package_.c_str(),p_aruco_saver_launch_file_.c_str(),args.c_str(),aruco_launch_id_);
    if (!aruco_launch_id_)
    {
      ROS_ERROR("[%s] Failed to launch Aruco saver launch file", name_.c_str());
      message_ = "Failed to launch Aruco launch file";
      return false;
    }
    
  }
  detecting_.data = true;
  return true;
}
bool ArucoHandler::startArucoAcml(std::string map_name_)
{
  if (!detecting_.data)
  {
    //Start Aruco Acml using launch_manager
    std::string args = " camera_info_topic:="+ p_aruco_camera_topic_+" image_topic_compressed:=" +p_aruco_image_topic;
    aruco_launch_id_ = startLaunch(p_aruco_launch_package_, p_aruco_amcl_launch_file_, args);
    ROS_INFO("%s %s %s %i" ,p_aruco_launch_package_.c_str(),p_aruco_amcl_launch_file_.c_str(),args.c_str(),aruco_launch_id_);
    if (!aruco_launch_id_)
    {
      ROS_ERROR("[%s] Failed to launch Aruco AMCL launch file", name_.c_str());
      message_ = "Failed to launch Aruco AMCL launch file";
      return false;
    }
    // enable the aruco_amcl and aruco detect to load fully before doing a service call and loading the map
    ros::Duration(2).sleep();
    std::string aruco_path = p_aruco_file_path + map_name_;
      aruco_path += ".txt";
      ROS_INFO("[%s] aruco_path including txt", aruco_path.c_str());
      ros::ServiceClient aruco_client = nh_handler_.serviceClient<movel_seirios_msgs::StringTrigger>("/movel_aruco_amcl/load_aruco");
      movel_seirios_msgs::StringTrigger load_aruco;
      load_aruco.request.input = aruco_path;
      aruco_client.call(load_aruco);
      if (load_aruco.response.success == false)
      {
          ROS_ERROR("[%s] Failed to load aruco file", aruco_path.c_str());
      }
  }
  detecting_.data = true;
  return true;
}

bool ArucoHandler::saveArucoPose(std::string map_name_){

if (!detecting_.data)
  {
    ROS_WARN("[%s] Save Aruco Pose service is called....Aruco handler will close after saving..", name_.c_str());
    return false;
  }
    ros::ServiceClient aruco_client = nh_handler_.serviceClient<movel_seirios_msgs::StringTrigger>("/movel_aruco_saver/save_aruco");
    movel_seirios_msgs::StringTrigger write_aruco;
    write_aruco.request.input = p_aruco_file_path + map_name_ +".txt";
    aruco_client.call(write_aruco);
    if (write_aruco.response.success == false)
    {
        ROS_ERROR("[%s] Failed to save aruco file", map_name_.c_str());
    }
  stopLaunch(aruco_launch_id_);
  aruco_launch_id_ = 0;
  detecting_.data = false;

  return true;

}

bool ArucoHandler::stopAruco()
{
  if (!detecting_.data)
  {
    ROS_WARN("[%s] Failed to stop Aruco Saver", name_.c_str());
    return false;
  }

  stopLaunch(aruco_launch_id_);
  aruco_launch_id_ = 0;
  detecting_.data = false;
  return true;
}

bool ArucoHandler::setupHandler()
{
  if (!loadParams())
    return false;

  //Advertise service for starting and stopping human detection
  start_saver_srv_ = nh_handler_.advertiseService("start_arucoSaver", &ArucoHandler::startArucoSaverCB, this);
  start_aruco_amcl_srv_ = nh_handler_.advertiseService("start_arucoAcml", &ArucoHandler::startArucoAcmlCB, this);
  stop_saver_srv_ = nh_handler_.advertiseService("stop", &ArucoHandler::stopArucoCB, this);
  save_aruco_pose_ = nh_handler_.advertiseService("save_pose", &ArucoHandler::saveArucoCB, this);
  status_saver_srv_ = nh_handler_.advertiseService("status", &ArucoHandler::onStatus, this);
  

  // check is it detecting
 aruco_checker_ = nh_handler_.advertiseService("/check_Aruco", &ArucoHandler::onCheckArucoChecker, this);
  
  // Health Reporter
  health_check_pub_ = nh_handler_.advertise<movel_seirios_msgs::Reports>("/task_supervisor/health_report", 1);

  // Localization Timer
  health_timer_ = nh_handler_.createTimer(ros::Duration(1.0 / p_timer_rate_), &ArucoHandler::onHealthTimerCallback, this);
  return true;
}

ReturnCode ArucoHandler::runTask(movel_seirios_msgs::Task& task, std::string& error_message)
{
  task_active_ = true;
  task_parsed_ = false;
  start_ = ros::Time::now();

  //Check first arg of task payload to see if start or stop, uses boost case insensitive string compare
  bool result;
  if (boost::iequals(task.payload, "start_aruco_saver"))
    result = startArucoSaver();
  else if (boost::iequals(task.payload, "stop_aruco_handler"))
    result = stopAruco();
  else
  {
    result = false;
    error_message = "[" + name_ + "] Payload command format invalid, input 'start' or 'stop'";
  }

  setTaskResult(result);
  return code_;
}

bool ArucoHandler::onCheckArucoChecker(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res){
  if(detecting_.data){
    res.success = true;
    res.message = "aruco detection enabled";
  }
  else {
    res.success = false;
    res.message = "aruco detection not enabled";
  }
  return true;
}


void ArucoHandler::onHealthTimerCallback(const ros::TimerEvent& timer_event)
{
  static int fail_count = 0;
  if (detecting_.data)
  {
    bool isHealthy = launchStatus(aruco_launch_id_);
    if (!isHealthy)
    {
      fail_count += 1;
      if (fail_count >= 2*p_timer_rate_)
      {
        ROS_INFO("[%s] Some nodes are disconnected", name_.c_str());
        movel_seirios_msgs::Reports report;
        report.header.stamp = ros::Time::now();
        report.handler = "aruco_handler";
        report.task_type = task_type_;
        report.healthy = false;
        report.message = "aruco nodes is not running";
        health_check_pub_.publish(report);
        stopAruco();
      }
    }
    else
      fail_count = 0; 
  }
}

}
