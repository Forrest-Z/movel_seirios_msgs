#include <task_supervisor/plugins/navigation_handler.h>
#include <task_supervisor/json.hpp>
#include <pluginlib/class_list_macros.h>
#include <actionlib_msgs/GoalID.h>
#include <movel_seirios_msgs/GetReachableSubplan.h>
#include <type_traits>

PLUGINLIB_EXPORT_CLASS(task_supervisor::NavigationHandler, task_supervisor::TaskHandler);


using json = nlohmann::json;

namespace task_supervisor
{

NavigationHandler::NavigationHandler() : 
  enable_human_detection_(false),
  human_detection_score_(0.0),
  task_cancelled_(false),
  isHealthy_(true)
{
}


bool NavigationHandler::setupHandler()
{
  if (!loadParams()) {
    ROS_FATAL("[%s] Error during parameter loading. Shutting down.", name_.c_str());
    return false;
  }
  enable_human_detection_srv_ = nh_handler_.advertiseService("/enable_human_detection", &NavigationHandler::enableHumanDetectionCB, this);
  enable_best_effort_goal_srv_ = nh_handler_.advertiseService("/enable_best_effort_goal", &NavigationHandler::enableBestEffortGoalCB, this);
  make_sync_plan_client_ = nh_handler_.serviceClient<nav_msgs::GetPlan>("/planner_utils/make_sync_plan");
  make_reachable_plan_client_ = nh_handler_.serviceClient<nav_msgs::GetPlan>("/planner_utils/make_reachable_plan");
  human_detection_sub_ = nh_handler_.subscribe(p_human_detection_topic_, 1, &NavigationHandler::humanDetectionCB, this);
  robot_pose_sub_ = nh_handler_.subscribe("/pose", 1, &NavigationHandler::robotPoseCB, this);
  loc_report_sub_ = nh_handler_.subscribe("/task_supervisor/health_report", 1, &NavigationHandler::locReportingCB, this);
  movebase_cancel_pub_ = nh_handler_.advertise<actionlib_msgs::GoalID>("/move_base/cancel", 1);
  obstruction_status_pub_ = nh_handler_.advertise<movel_seirios_msgs::ObstructionStatus>("/obstruction_status", 1);
  return true;
}


template <typename param_type>
bool NavigationHandler::load_param_util(std::string param_name, param_type& output)
{
  if (!nh_handler_.getParam(param_name, output)) {
    ROS_ERROR("[%s] Failed to load parameter: %s", name_.c_str(), param_name.c_str());
    return false;
  }
  else {  
    if (std::is_same<param_type, bool>::value) {   // bool
      ROS_INFO_STREAM("[" << name_ << "] " << param_name << ": " << output ? "true" : "false");
    }
    else {   // all others 
      ROS_INFO_STREAM("[" << name_ << "] " << param_name << ": " << output);
    }
    return true;
  }
}


bool NavigationHandler::loadParams()
{
  ROS_WARN("[%s] Loading of plugin parameters by ros_utils has not been implemented. Loading directly from Parameter "
           "Server instead.",
           name_.c_str());
  if (!load_param_util("server_timeout", p_server_timeout_)) { return false; }
  if (!load_param_util("static_paths", p_static_paths_)) { return false; }
  if (!load_param_util("navigation_server", p_navigation_server_)) { return false; }
  if (!load_param_util("human_detection_min_score", p_human_detection_min_score_)) { return false; }
  if (!load_param_util("human_detection_topic", p_human_detection_topic_)) { return false; }
  if (!load_param_util("enable_human_detection_msg", p_enable_human_detection_msg_)) { return false; }
  if (!load_param_util("disable_human_detection_msg", p_disable_human_detection_msg_)) { return false; }
  if (!load_param_util("enable_best_effort_goal", p_enable_best_effort_goal_)) { return false; }
  if (!load_param_util("normal_nav_if_best_effort_unavailable", p_normal_nav_if_best_effort_unavailable_)) { return false; }
  if (!load_param_util("best_effort_retry_timeout_sec", p_best_effort_retry_timeout_sec_)) { return false; }
  if (!load_param_util("best_effort_retry_sleep_sec", p_best_effort_retry_sleep_sec_)) { return false; }
  return true;
}


bool NavigationHandler::enableHumanDetectionCB(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res)
{
  enable_human_detection_ = req.data;

  if(req.data)
    ROS_INFO("[%s] %s", name_.c_str(), p_enable_human_detection_msg_.c_str());
  else
    ROS_INFO("[%s] %s", name_.c_str(), p_disable_human_detection_msg_.c_str());

  res.success = true;
  return true;
}


void NavigationHandler::humanDetectionCB(const std_msgs::Float64::ConstPtr& msg)
{
  boost::unique_lock<boost::mutex> scoped_lock(mtx_);
  human_detection_score_ = msg->data;
}


bool NavigationHandler::enableBestEffortGoalCB(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res)
{
  p_enable_best_effort_goal_ = req.data;
  ROS_INFO("[%s] %s navigation with best effort goal", name_.c_str(), p_enable_best_effort_goal_ ? "Enabled" : "Disabled");
  res.success = true;
  return true;
}


void NavigationHandler::robotPoseCB(const geometry_msgs::Pose::ConstPtr& msg)
{
  if (task_active_) { robot_pose_ = *msg; }
}


bool NavigationHandler::start_ActionClient()
{
  nav_ac_ptr_ =
      std::make_shared<actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>>(p_navigation_server_, true);

  if (!nav_ac_ptr_->waitForServer(ros::Duration(p_server_timeout_)))
  {
    ROS_ERROR("[%s] Could not communicate with navigation server after waiting for %f seconds. Shutting down.",
              name_.c_str(), p_server_timeout_);
    message_ = "Could not communicate with navigation server after waiting for %f seconds. ";
    return false;
  }
  ROS_INFO("[%s] Communication with move base action server started", name_.c_str());

  return true;
}


void NavigationHandler::startWithDetection(const move_base_msgs::MoveBaseGoal goal)
{
  // No human detected
  if(human_detection_score_ < p_human_detection_min_score_)
    nav_ac_ptr_->sendGoal(goal);
  else {
    bool init = true;
    // Wait until no human is detected
    while(human_detection_score_ > p_human_detection_min_score_ && !task_cancelled_)
    {
      if (init) {
        ROS_INFO("[%s] Human(s) detected, pausing navigation", name_.c_str());
        init = false;
      }
      ros::Duration(0.1).sleep();
    }
    if (!task_cancelled_) {
      ROS_INFO("[%s] Resuming navigation", name_.c_str());
      nav_ac_ptr_->sendGoal(goal);
    }
    else {
      task_cancelled_ = false;
      return;
    }
  } 
}


bool NavigationHandler::navigationLoop(const move_base_msgs::MoveBaseGoal& goal)
{
  bool navigating = true;
  bool succeeded = false;

  // Loops until cancellation is called or navigation task is complete
  while(!task_cancelled_)
  {
    // check pause
    if (isTaskPaused() && navigating) {
      ROS_INFO("[%s] Navigation paused", name_.c_str());
      navigating = false;
      nav_ac_ptr_->cancelGoal();
    }
    else if (!isTaskPaused() && !navigating && !enable_human_detection_) {
      ROS_INFO("[%s] Navigation resumed", name_.c_str());
      navigating = true;
      nav_ac_ptr_->sendGoal(goal);
    }
    else if (!isTaskPaused() && !navigating && enable_human_detection_) {
      // No human detected and robot stopped
      if (human_detection_score_ < p_human_detection_min_score_) {
        ROS_INFO("[%s] No human detected, resuming navigation", name_.c_str());
        navigating = true;
        nav_ac_ptr_->sendGoal(goal);
      }
    }
    else if (!isTaskPaused() && navigating && enable_human_detection_) {
      // Human(s) detected and robot is moving
      if (human_detection_score_ > p_human_detection_min_score_) {
        ROS_INFO("[%s] Human(s) detected, pausing navigation", name_.c_str());
        navigating = false;
        nav_ac_ptr_->cancelGoal();
      }
    }
    // check finished
    actionlib::SimpleClientGoalState state = nav_ac_ptr_->getState();
    if (state.isDone() && !isTaskPaused()) {
      if (state == actionlib::SimpleClientGoalState::SUCCEEDED)
        succeeded = true;
      break;
    }
    // check launch health
    if (!isHealthy_) {
      ROS_INFO("[%s] Some nodes are disconnected. Stopping Navigation", name_.c_str());
      succeeded = false;
      break;
    }
    // loop
    ros::Duration(0.1).sleep();
  }
  // result
  if (!task_cancelled_)
    return succeeded;
  else
    return false;
}


bool NavigationHandler::navigationAttemptGoal(const move_base_msgs::MoveBaseGoal& goal_msg)
{
  if (!enable_human_detection_) {
    nav_ac_ptr_->sendGoal(goal_msg);
    return navigationLoop(goal_msg);
  }
  else {
    startWithDetection(goal_msg);
    return navigationLoop(goal_msg);
  }
}


void NavigationHandler::navigationDirect(const geometry_msgs::Pose& goal_pose)
{
  // get plan from move_base at current robot pose
  nav_msgs::GetPlan srv{};
  srv.request.start.pose = robot_pose_;   // current pose
  srv.request.start.header.frame_id = "map";
  srv.request.goal.pose = goal_pose;   // main goal
  srv.request.goal.header.frame_id = "map";
  // start navigation
  ROS_INFO("[%s] Starting navigation to goal", name_.c_str());
  move_base_msgs::MoveBaseGoal goal_msg;
  goal_msg.target_pose.pose = goal_pose;   // main goal
  goal_msg.target_pose.header.frame_id = "map";
  bool success = navigationAttemptGoal(goal_msg);
  if (success)
    ROS_INFO("[%s] Navigation to goal success", name_.c_str());
  else
    ROS_INFO("[%s] Navigation to goal failed", name_.c_str());
  setTaskResult(success);
  return;
} 
  

void NavigationHandler::navigationBestEffort(const geometry_msgs::Pose& goal_pose)
{
  // main loop  
  // try until main goal is reached or no more best effort goals can be found
  CountdownTimer countdown_timer{};
  ros::Duration retry_sleep{p_best_effort_retry_sleep_sec_};
  bool retry_at_obstacle = false;
  while (!task_cancelled_ && isHealthy_)
  {
    // retry expiry check
    if (retry_at_obstacle) {
      // sleep to not spam while loop
      retry_sleep.sleep();
      // retry timed out, navigation failed
      if (countdown_timer.expired()) {
        ROS_ERROR("[%s] Best effort retries timed out", name_.c_str());
        setTaskResult(false);
        return;  
      }
      ROS_INFO("[%s] Best effort retry at obstacle", name_.c_str());
    }
    // choose between direct goal or best effort goal
    nav_msgs::GetPlan srv{};
    srv.request.start.header.frame_id = "map";
    srv.request.goal.header.frame_id = "map"; 
    // trying direct goal
    srv.request.start.pose = robot_pose_;   // current pose
    srv.request.goal.pose = goal_pose;   // main goal
    // get sync/dirty plan at current robot pose
    if (!make_sync_plan_client_.call(srv)) {
      ROS_ERROR("[%s] Service call to make_sync_plan_client failed", name_.c_str());
      ROS_INFO("[%s] Direct goal failed", name_.c_str());
      ROS_INFO("[%s] Retrying with best effort goal", name_.c_str());
    }
    // direct goal exists, no best effort needed
    else if (srv.response.plan.poses.size() > 0) {
      ROS_INFO("[%s] Starting navigation to direct goal", name_.c_str());
      // report obstruction
      movel_seirios_msgs::ObstructionStatus report_obs;
      report_obs.reporter = "navigation_handler";
      report_obs.status = "false";
      report_obs.location = goal_pose;
      obstruction_status_pub_.publish(report_obs);
      // start navigation
      move_base_msgs::MoveBaseGoal goal_msg;
      goal_msg.target_pose.pose = goal_pose;   // main goal
      goal_msg.target_pose.header.frame_id = "map";
      bool success = navigationAttemptGoal(goal_msg);
      if (task_cancelled_) { return; }
      if (success) {
        ROS_INFO("[%s] Direct goal success", name_.c_str());
        setTaskResult(true);
        return;
      } 
      else {   // nav failed, continue to best effort
        ROS_INFO("[%s] Direct goal failed", name_.c_str());
        ROS_INFO("[%s] Retrying with best effort goal", name_.c_str());
      } 
    }
    // trying best effort goal
    srv.request.start.pose = robot_pose_;   // current pose
    srv.request.goal.pose = goal_pose;   // main goal
    if (!make_reachable_plan_client_.call(srv)) {
      ROS_ERROR("[%s] Service call to make_reachable_plan_client failed, retrying best effort", name_.c_str());
      if (!retry_at_obstacle) { 
        retry_at_obstacle = true;
        countdown_timer.start(p_best_effort_retry_timeout_sec_);
      }
      continue;   // continue retry loop
    }
    // best effort goal exists
    {   // local scope (formatting purposes)
      ROS_INFO("[%s] Starting navigation to best effort goal", name_.c_str());
      geometry_msgs::Pose reachable_pose = srv.response.plan.poses.back().pose;
      // report obstruction
      movel_seirios_msgs::ObstructionStatus report_obs;
      report_obs.reporter = "navigation_handler";
      report_obs.status = "true";
      report_obs.location = reachable_pose;
      obstruction_status_pub_.publish(report_obs);
      // update/disable retry loop 
      retry_at_obstacle = false;
      // start navigation
      move_base_msgs::MoveBaseGoal goal_msg;
      goal_msg.target_pose.header.frame_id = "map";
      goal_msg.target_pose.pose = reachable_pose;
      bool success = navigationAttemptGoal(goal_msg);
      if (success) { 
        ROS_INFO("[%s] Best effort goal success, continue best effort navigation from this position", name_.c_str());
      }   
      else {   
        // false if dynamic obstacle causes blockage, obstacle_idx should not be updated
        ROS_INFO("[%s] Best effort goal failed (possibly from dynamic obstacle)", name_.c_str());
        ROS_INFO("[%s] Retrying with another best effort goal", name_.c_str());
      }
      continue;   // continue while loop and try main goal again
    }
  }
}


bool NavigationHandler::runTaskChooseNav(const geometry_msgs::Pose& goal_pose)
{
  // navigation
  task_cancelled_ = false;
  // best effort
  if (p_enable_best_effort_goal_) {
    ROS_INFO("[%s] Starting navigation - best effort: enabled", name_.c_str());
    // planner_utils service available
    if (make_reachable_plan_client_.exists()) {
      navigationBestEffort(goal_pose);
    }
    // planner_utils service unavailable
    else {
      std::string service_name = make_reachable_plan_client_.getService();
      if (p_normal_nav_if_best_effort_unavailable_) {
        ROS_WARN("[%s] Services %s are not available, using normal navigation", name_.c_str(), service_name.c_str());
        navigationDirect(goal_pose);
      }
      else {
        ROS_ERROR("[%s] Services %s are not available", name_.c_str(), service_name.c_str());
        setTaskResult(false);
      }
    }
  }
  // normal navigation
  else {
    ROS_INFO("[%s] Starting navigation - best effort: disabled", name_.c_str());
    navigationDirect(goal_pose);
  }
  // check navigation success (for multimap nav)
  if (code_ == ReturnCode::SUCCESS) { return true; }
  if (code_ == ReturnCode::FAILED) { return false; }
}


ReturnCode NavigationHandler::runTask(movel_seirios_msgs::Task& task, std::string& error_message)
{
  task_active_ = true;
  task_parsed_ = false;
  isHealthy_ = true;
  start_ = ros::Time::now();

  ros::ServiceClient clear_costmap_serv_ = nh_handler_.serviceClient<std_srvs::Empty>("/move_base/clear_costmaps");

  if(ros::service::waitForService("/move_base/clear_costmaps",ros::Duration(2.0))){
    std_srvs::Empty clear_costmap_msg;
    clear_costmap_serv_.call(clear_costmap_msg);
  }
  else{
    ROS_WARN("[%s] Could not contact clear_costmap service", name_.c_str());
  }
  
  if (start_ActionClient()) {
    ROS_INFO("[%s] Task payload %s", name_.c_str(), task.payload.c_str());
    json payload = json::parse(task.payload);
    if (payload.find("position") != payload.end()) {
      // input goal
      geometry_msgs::Pose goal_pose;
      goal_pose.position.x = payload["position"]["x"].get<float>();
      goal_pose.position.y = payload["position"]["y"].get<float>();
      goal_pose.position.z = payload["position"]["z"].get<float>();
      goal_pose.orientation.x = payload["orientation"]["x"].get<float>();
      goal_pose.orientation.y = payload["orientation"]["y"].get<float>();
      goal_pose.orientation.z = payload["orientation"]["z"].get<float>();
      goal_pose.orientation.w = payload["orientation"]["w"].get<float>();
      // navigation
      runTaskChooseNav(goal_pose);
    }
    else {
      setMessage("Malformed payload Example: {\"position\":{\"x\":-4,\"y\":0.58,\"z\":0}, "
                 "\"orientation\":{\"x\":0,\"y\":0,\"z\":0.71,\"w\":0.69}}");
      error_message = message_;
      setTaskResult(false);
    }
  }
  else {
    setMessage("Unable to talk to Move Base action server on " + p_navigation_server_);
    error_message = message_;
    setTaskResult(false);
  }
  return code_;
}


void NavigationHandler::cancelTask()
{
  task_cancelled_ = true;
  if (nav_ac_ptr_->isServerConnected()) {
    nav_ac_ptr_->cancelGoal();
    // wait for action to be properly cancelled
    while (nav_ac_ptr_->getState() == actionlib::SimpleClientGoalState::ACTIVE) {
      ros::Duration(0.01).sleep();
    }
    // cancel move_base goal
    actionlib_msgs::GoalID goal_id;
    movebase_cancel_pub_.publish(goal_id);
    setMessage(" navigation goal was cancelled");
    setTaskResult(false);
  }
  else {
    setMessage("Server disconnected before navigation goal was cancelled");
    setTaskResult(false);
  }
  task_parsed_ = true;
  task_active_ = false;
  task_paused_ = false;
}


void NavigationHandler::locReportingCB(const movel_seirios_msgs::Reports::ConstPtr& msg)
{
  if (msg->handler == "localization_handler" && msg->healthy == false && task_active_)    // Localization failed
    isHealthy_ = false;
}

}  // namespace task_supervisor
