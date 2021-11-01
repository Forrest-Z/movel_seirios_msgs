#include <task_supervisor/plugins/path_handler.h>
#include <task_supervisor/json.hpp>
#include <ros_utils/ros_utils.h>
#include <path_recall/SavePath.h>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(task_supervisor::PathHandler, task_supervisor::TaskHandler);

using json = nlohmann::json;

namespace task_supervisor
{
PathHandler::PathHandler() : enable_human_detection_(false),
			     human_detection_score_(0.0),
            isRunning_(false),
            isLocHealthy_(true),
            isPathHealthy_(true)
{
}

bool PathHandler::setupHandler()
{
  if (!loadParams())
    return false;
  else
  {
    enable_human_detection_srv_ = nh_handler_.advertiseService("enable_human_detection", &PathHandler::enableHumanDetectionCB, this);
    human_detection_sub_ = nh_handler_.subscribe(p_human_detection_topic_, 1, &PathHandler::humanDetectionCB, this);
    loc_report_sub_ = nh_handler_.subscribe("/task_supervisor/health_report", 1, &PathHandler::locReportingCB, this);
    health_check_pub_ = nh_handler_.advertise<movel_seirios_msgs::Reports>("/task_supervisor/health_report", 1);

    teardown_timer_ = nh_handler_.createTimer(ros::Duration(p_teardown_timeout_), &PathHandler::teardownTimerCb, this, true, false);
    return true;
  }
}

bool PathHandler::loadParams()
{
  ros_utils::ParamLoader param_loader(nh_handler_);

  param_loader.get_optional("loop_rate", p_loop_rate_, 10.0);
  param_loader.get_required("path_load_launch_file", p_path_load_launch_file_);
  param_loader.get_required("path_load_launch_package", p_path_load_launch_package_);
  param_loader.get_required("human_detection_topic", p_human_detection_topic_);
  param_loader.get_required("human_detection_min_score", p_human_detection_min_score_);
  param_loader.get_required("enable_human_detection_msg", p_enable_human_detection_msg_);
  param_loader.get_required("disable_human_detection_msg", p_disable_human_detection_msg_);
  param_loader.get_required("teardown_timeout", p_teardown_timeout_);

  return param_loader.params_valid();
}

bool PathHandler::onStatus(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res)
{
  res.success = launchStatus(path_load_launch_id_);
  return true;
}

bool PathHandler::enableHumanDetectionCB(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res)
{
  enable_human_detection_ = req.data;

  if(req.data)
    ROS_INFO("[%s] %s", name_.c_str(), p_enable_human_detection_msg_.c_str());
  else
    ROS_INFO("[%s] %s", name_.c_str(), p_disable_human_detection_msg_.c_str());

  res.success = true;
  return true;
}

void PathHandler::humanDetectionCB(const std_msgs::Float64::ConstPtr& msg)
{
  boost::unique_lock<boost::mutex> scoped_lock(mtx_);
  human_detection_score_ = msg->data;
}

void PathHandler::onPose(const geometry_msgs::Pose::ConstPtr& msg)
{
  pose_received_ = true;
}

void PathHandler::onPathStatus(const std_msgs::BoolConstPtr& msg)
{
  path_load_started_ = msg->data;
}

void PathHandler::onPathFailStatus(const std_msgs::BoolConstPtr& msg)
{
  path_run_failed_ = msg->data;
}

ReturnCode PathHandler::runTask(movel_seirios_msgs::Task& task, std::string& error_message)
{
  task_active_ = true;
  task_parsed_ = false;
  path_load_started_ = true;
  path_run_failed_ = false;
  pose_received_ = false;
  start_ = ros::Time::now();
  isRunning_ = false;

  if (!loadParams())
  {
    setTaskResult(false);
    return code_;
  }

  ros::ServiceServer serv_status_ = nh_handler_.advertiseService("status", &PathHandler::onStatus, this);
  ros::Publisher goal_pub_ = nh_handler_.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1);
  ros::Publisher cancel_pub_ = nh_handler_.advertise<actionlib_msgs::GoalID>("/move_base/cancel", 1);

  // Launch path loading node
  ros::Time t_start_launch = ros::Time::now();
  if (path_load_launch_id_ == 0 || !launchExists(path_load_launch_id_))
  {
    ROS_INFO("[%s] Starting path load package: %s, launch file: %s", name_.c_str(), p_path_load_launch_package_.c_str(), p_path_load_launch_file_.c_str());
    path_load_launch_id_ = startLaunch(p_path_load_launch_package_, p_path_load_launch_file_, "");
    if (!path_load_launch_id_)
    {
      ROS_ERROR("[%s] Failed to launch path loading launch file", name_.c_str());
      setTaskResult(false);
      return code_;
    }

    // Wait for path loading node to establish connection with subscribed and published topics
    ros::Duration(0.5).sleep();

    path_load_client_ = nh_handler_.serviceClient<path_recall::SavePath>("/path_load/path_input");
    path_state_sub_ = nh_handler_.subscribe("/path_load/start", 1, &PathHandler::onPathStatus, this);
    path_run_fail_sub_ = nh_handler_.subscribe("/path_load/fail", 1, &PathHandler::onPathFailStatus, this);
    pose_sub_ = nh_handler_.subscribe("/pose", 1, &PathHandler::onPose, this);
  }

  if (teardown_timer_.hasStarted())
    teardown_timer_.stop();

  double dt_launch = (ros::Time::now() - t_start_launch).toSec();
  ROS_INFO("[%s] Ready. Prep and launch took %5.2f [s]", name_.c_str(), dt_launch);

  ROS_INFO("[%s] Task payload %s", name_.c_str(), task.payload.c_str());
  json payload = json::parse(task.payload);
  ROS_INFO("[%s] Payload parsed", name_.c_str());

  nav_msgs::Path path;
  path.header.frame_id = "map";

  ros::Rate r(p_loop_rate_);
  if (payload.find("path") != payload.end())
  {
    for (auto& elem : payload["path"])
    {
      geometry_msgs::PoseStamped waypoint;
      waypoint.header.frame_id = "map";
      waypoint.header.stamp = ros::Time::now();

      waypoint.pose.position.x = elem["position"]["x"].get<double>();
      waypoint.pose.position.y = elem["position"]["y"].get<double>();
      waypoint.pose.position.z = elem["position"]["z"].get<double>();

      waypoint.pose.orientation.x = elem["orientation"]["x"].get<double>();
      waypoint.pose.orientation.y = elem["orientation"]["y"].get<double>();
      waypoint.pose.orientation.z = elem["orientation"]["z"].get<double>();
      waypoint.pose.orientation.w = elem["orientation"]["w"].get<double>();

      path.poses.push_back(waypoint);
    }

    // Wait for robot pose message to be received by path loading node
    while(!pose_received_)
    {
      ROS_INFO("[%s] Waiting for robot pose message", name_.c_str());
      ros::Duration(0.1).sleep();
    }

    if (enable_human_detection_)
    {
      bool init = true;

      // Human(s) detected, wait until no detection before proceeding
      while (human_detection_score_ > p_human_detection_min_score_)
      {
        // Cancellation called
        if (!isTaskActive())
        {
          ROS_INFO("[%s] Task cancelled, running required cleanup tasks", name_.c_str());

          stopLaunch(path_load_launch_id_);

          error_message = "[" + name_ + "] Task cancelled";
          setTaskResult(false);
          return code_;
        }
        if (init)
        {
          ROS_INFO("[%s] Human(s) detected, pausing path following", name_.c_str());
          init = false;
        }
        r.sleep();
      }
    }

    // Start path following
    ROS_INFO("[%s] Starting path following", name_.c_str());
    path_recall::SavePath load_srv;
    load_srv.request.name = "current_path";
    load_srv.request.path = path;
    path_load_client_.waitForExistence(ros::Duration(10.0));
    if (!path_load_client_.call(load_srv))
    {
      message_ = "[" + name_ + "] Unable to call /path_load/path_input service";
      setTaskResult(false);
      stopLaunch(path_load_launch_id_);
      return code_;
    }

    if(!load_srv.response.success)
    {
      message_ = "Path loading failed";
      setTaskResult(false);
      stopLaunch(path_load_launch_id_);
      return code_;
    }
  }

  else
  {
    setMessage("Malformed payload");
    error_message = message_;
    setTaskResult(false);
    stopLaunch(path_load_launch_id_);
    return code_;
  }

  isRunning_ = true;
  isLocHealthy_ = true;
  isPathHealthy_ = true;
  
  bool navigating = true;
  while (ros::ok())
  {
    // Cancellation is called
    if (!isTaskActive())
    {
      ROS_INFO("[%s] Task cancelled, running required cleanup tasks", name_.c_str());
      cancelPath();
      ros::Time cancel_start = ros::Time::now();
      while (path_load_started_)
      {
        ROS_INFO("[%s] Waiting for path loading to end", name_.c_str());
        if (ros::Time::now().toSec() - cancel_start.toSec() > 5.0)
          break;
        ros::Duration(0.2).sleep();
      }

      // stopLaunch(path_load_launch_id_);
      teardown_timer_.setPeriod(ros::Duration(p_teardown_timeout_));
      teardown_timer_.start();

      actionlib_msgs::GoalID move_base_cancel;
      cancel_pub_.publish(move_base_cancel);

      error_message = "[" + name_ + "] Task cancelled";
      setTaskResult(false);
      isRunning_ = false;
      path_run_failed_ = false;
      return code_;
    }

    // Check for pause/resume
    if (isTaskPaused() && navigating)
    {
      navigating = false;
      pausePath();
    }
    else if (!isTaskPaused() && !navigating && !enable_human_detection_)
    {
      navigating = true;
      resumePath();
    }
    else if (!isTaskPaused() && !navigating && enable_human_detection_)
    {
      if (human_detection_score_ < p_human_detection_min_score_)
      {
        ROS_INFO("[%s] No human detected", name_.c_str());
        navigating = true;
        resumePath();
      }
    }
    else if (!isTaskPaused() && navigating && enable_human_detection_)
    {
      if (human_detection_score_ > p_human_detection_min_score_)
      {
        ROS_INFO("[%s] Human(s) detected", name_.c_str());
        navigating = false;
        pausePath();
      }
    }
    // Reached end of path
    if (!path_load_started_ and !path_run_failed_)
    {
      ROS_INFO("[%s] Path completed", name_.c_str());

      // stopLaunch(path_load_launch_id_);
      teardown_timer_.setPeriod(ros::Duration(p_teardown_timeout_));
      teardown_timer_.start();

      setTaskResult(true);
      isRunning_ = false;
      return code_;
    }

    if (path_run_failed_)
    {
      ROS_INFO("[%s] Full path cannot be completed", name_.c_str());

      // stopLaunch(path_load_launch_id_);
      teardown_timer_.setPeriod(ros::Duration(p_teardown_timeout_));
      teardown_timer_.start();

      setTaskResult(false);
      isRunning_ = false;

      return code_;
    }

    if (!isLocHealthy_ || !isPathHealthy_ )     // When localization node is not available
    {
      ROS_INFO("[%s] Some node are disconnected. Stopping navigation.", name_.c_str());
      
      if (!isPathHealthy_)
      {
        actionlib_msgs::GoalID move_base_cancel;
        cancel_pub_.publish(move_base_cancel);
      }

      // stopLaunch(path_load_launch_id_);
      teardown_timer_.setPeriod(ros::Duration(p_teardown_timeout_));
      teardown_timer_.start();

      setTaskResult(false);
      error_message = "[" + name_ + "] Some node are disconnected";
      isRunning_ = false;
      return code_;
    }

    r.sleep();
  }

  error_message = "[" + name_ + "] Task failed, ROS was shutdown";
  setTaskResult(false);
  return code_;
}

bool PathHandler::cancelPath()
{
  ros::ServiceClient path_load_cancel = nh_handler_.serviceClient<std_srvs::Trigger>("/path_load/cancel");
  std_srvs::Trigger cancel;
  if(!path_load_cancel.call(cancel))
  {
    ROS_INFO("Failed to call path loading cancel service");
    return false;
  }
  else
    return cancel.response.success;
}

bool PathHandler::pausePath()
{
  ROS_INFO("[%s] Pausing path following", name_.c_str());
  ros::ServiceClient path_load_pause = nh_handler_.serviceClient<std_srvs::Trigger>("/path_load/pause");
  std_srvs::Trigger pause;
  if(!path_load_pause.call(pause))
  {
    ROS_INFO("Failed to call path loading pause service");
    return false;
  }
  else
    return pause.response.success;
}

bool PathHandler::resumePath()
{
  ROS_INFO("[%s] Resuming path following", name_.c_str());
  ros::ServiceClient path_load_resume = nh_handler_.serviceClient<std_srvs::Trigger>("/path_load/resume");
  std_srvs::Trigger resume;
  if(!path_load_resume.call(resume))
  {
    ROS_INFO("Failed to call path loading resume service");
    return false;
  }
  else
    return resume.response.success;
}

bool PathHandler::healthCheck()
{
  if (isRunning_)
  {
    bool isHealthy = launchStatus(path_load_launch_id_);
    if (!isHealthy)
    {
      isPathHealthy_ = false;
      movel_seirios_msgs::Reports report;
      report.header.stamp = ros::Time::now();
      report.handler = "path_handler";
      report.task_type = task_type_;
      report.healthy = false;
      report.message = "path_recall nodes is not running";
      health_check_pub_.publish(report);
    }
  }
  
  return true;
}

void PathHandler::locReportingCB(const movel_seirios_msgs::Reports::ConstPtr& msg)
{
  if (msg->handler == "localization_handler" && msg->healthy == false && isRunning_)    // Localization failed
    isLocHealthy_ = false;
}

void PathHandler::teardownTimerCb(const ros::TimerEvent& e)
{
  ROS_INFO("[%s] tearing down launch", name_.c_str());

  if (isTaskActive())
  {
    ROS_INFO("[%s] but task is active? Restart teardown timer", name_.c_str());
    teardown_timer_.setPeriod(ros::Duration(p_teardown_timeout_));
    teardown_timer_.start();
    return;
  }
  
  stopLaunch(path_load_launch_id_);
  path_load_launch_id_ = 0;
}

}  // namespace task_supervisor
