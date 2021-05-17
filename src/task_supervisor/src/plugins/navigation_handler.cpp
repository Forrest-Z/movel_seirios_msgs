#include <task_supervisor/plugins/navigation_handler.h>
#include <task_supervisor/json.hpp>
#include <pluginlib/class_list_macros.h>
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
  make_movebase_plan_client_ = nh_handler_.serviceClient<nav_msgs::GetPlan>("/move_base/make_plan");
  make_clean_plan_client_ = nh_handler_.serviceClient<nav_msgs::GetPlan>("/make_clean_plan");
  calc_reachable_subplan_client_ = nh_handler_.serviceClient<movel_seirios_msgs::GetReachableSubplan>("/calc_reachable_subplan");
  human_detection_sub_ = nh_handler_.subscribe(p_human_detection_topic_, 1, &NavigationHandler::humanDetectionCB, this);
  robot_pose_sub_ = nh_handler_.subscribe("/pose", 1, &NavigationHandler::robotPoseCB, this);
  loc_report_sub_ = nh_handler_.subscribe("/task_supervisor/health_report", 1, &NavigationHandler::locReportingCB, this);

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
  robot_pose_ = *msg;
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
  else
  {
    bool init = true;

    // Wait until no human is detected
    while(human_detection_score_ > p_human_detection_min_score_ && !task_cancelled_)
    {
      if (init)
      {
        ROS_INFO("[%s] Human(s) detected, pausing navigation", name_.c_str());
        init = false;
      }
      ros::Duration(0.1).sleep();
    }

    if (!task_cancelled_)
    {
      ROS_INFO("[%s] Resuming navigation", name_.c_str());
      nav_ac_ptr_->sendGoal(goal);
    }
    else
    {
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
    actionlib::SimpleClientGoalState state = nav_ac_ptr_->getState();
    if (state.isDone() && !isTaskPaused()) {
      if (state == actionlib::SimpleClientGoalState::SUCCEEDED)
        succeeded = true;
      break;
    }

    if (!isHealthy_) {
      ROS_INFO("[%s] Some nodes are disconnected. Stopping Navigation", name_.c_str());
      succeeded = false;
      break;
    }

    ros::Duration(0.1).sleep();
  }

  if (!task_cancelled_) {
    return succeeded;
  }
  else {
    return false;
  }
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
  if (!make_movebase_plan_client_.call(srv)) {
    ROS_ERROR("[%s] Service call to make_plan failed", name_.c_str());
    setTaskResult(false);
    return;
  }
  // direct goal exists
  if (srv.response.plan.poses.size() > 0) {
    ROS_INFO("[%s] Starting navigation to goal", name_.c_str());
    // start navigation
    move_base_msgs::MoveBaseGoal goal_msg;
    goal_msg.target_pose.pose = goal_pose;   // main goal
    goal_msg.target_pose.header.frame_id = "map";
    bool success = navigationAttemptGoal(goal_msg);
    setTaskResult(success);
    return;
  }
  // no direct goal
  else {
    ROS_INFO("[%s] No direct plan to goal exists", name_.c_str());
    setTaskResult(false);
    return;
  }
} 
  

void NavigationHandler::navigationBestEffort(const geometry_msgs::Pose& goal_pose)
{
  nav_msgs::GetPlan srv{};
  srv.request.start.header.frame_id = "map";
  srv.request.goal.header.frame_id = "map"; 
  // get global clean_plan
  srv.request.start.pose = robot_pose_;   // current pose
  srv.request.goal.pose = goal_pose;   // main goal
  if (!make_clean_plan_client_.call(srv)) {
    ROS_ERROR("[%s] Service call to make_clean_plan failed", name_.c_str());
    setTaskResult(false);
    return;
  }
  if (srv.response.plan.poses.size() == 0) {
    ROS_ERROR("[%s] No valid plan avaliable from make_clean_plan", name_.c_str());
    setTaskResult(false);
    return;
  }
  std::vector<geometry_msgs::PoseStamped> clean_plan = srv.response.plan.poses;
  // main loop  
  // try until main goal is reached or no more best effort goals can be found
  CountdownTimer countdown_timer{};
  ros::Duration retry_sleep{p_best_effort_retry_sleep_sec_};
  bool retry_at_obstacle = false;
  int obstacle_idx = 0;
  while (!task_cancelled_)
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
    // trying direct goal
    srv.request.start.pose = robot_pose_;   // current pose
    srv.request.goal.pose = goal_pose;   // main goal
    // get plan from move_base at current robot pose
    if (!make_movebase_plan_client_.call(srv)) {
      ROS_ERROR("[%s] Service call to make_plan failed", name_.c_str());
      setTaskResult(false);
      return;
    }
    // direct goal exists, no best effort needed
    if (srv.response.plan.poses.size() > 0) {
      ROS_INFO("[%s] Starting navigation to goal", name_.c_str());

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
        setTaskResult(true);
        return;
      } 
      else {}   // nav failed, continue to best effort
    }
    // trying best effort goal
    // get subplan idx from planner_utils clean_plan based on current obstacle idx
    //  This avoids cases where the forward march is truncated due to another dynamic obstacle 
    //  appearing between the start of the clean_plan and the current obstacle idx 
    movel_seirios_msgs::GetReachableSubplan subplan_srv{};
    subplan_srv.request.plan.poses = clean_plan;
    subplan_srv.request.start_from_idx = obstacle_idx;
    if (!calc_reachable_subplan_client_.call(subplan_srv)) {
      ROS_ERROR("[%s] Service call to calc_reachable_subplan failed", name_.c_str());
      setTaskResult(false);
      return;
    }
    // best effort goal exists
    int subplan_idx = subplan_srv.response.reachable_idx;
    // stuck at obstacle, retry at obstacle 
    if (subplan_idx == obstacle_idx) {
      // initiate retry loop
      if (!retry_at_obstacle) { 
        retry_at_obstacle = true;
        countdown_timer.start(p_best_effort_retry_timeout_sec_);
      }
      continue;   // continue retry loop
    }
    // advance to new subgoal
    else {
      ROS_INFO("[%s] Starting navigation to best effort goal", name_.c_str());

      // report obstruction
      movel_seirios_msgs::ObstructionStatus report_obs;
      report_obs.reporter = "navigation_handler";
      report_obs.status = "true";
      report_obs.location = clean_plan[subplan_srv.response.blocked_idx].pose;
      obstruction_status_pub_.publish(report_obs);

      // update/disable retry loop 
      obstacle_idx = subplan_idx;
      retry_at_obstacle = false;
      // start navigation
      move_base_msgs::MoveBaseGoal goal_msg;
      goal_msg.target_pose.pose = clean_plan.at(subplan_idx).pose;
      goal_msg.target_pose.header.frame_id = "map";
      navigationAttemptGoal(goal_msg);   // success ignored, continue loop
      if (task_cancelled_) { return; }
      continue;   // continue while loop and try main goal again
    }
  }
}


ReturnCode NavigationHandler::runTask(movel_seirios_msgs::Task& task, std::string& error_message)
{
  task_active_ = true;
  task_parsed_ = false;
  isHealthy_ = true;
  start_ = ros::Time::now();

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
      task_cancelled_ = false;
      // best effort
      if (p_enable_best_effort_goal_) {
        ROS_INFO("[%s] Starting navigation - best effort: enabled", name_.c_str());
        // planner_utils service available
        if (make_clean_plan_client_.exists() && calc_reachable_subplan_client_.exists()) {
          navigationBestEffort(goal_pose);
        }
        // planner_utils service unavailable
        else {
          std::string service_name = make_clean_plan_client_.getService() + " & " + calc_reachable_subplan_client_.getService();
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
  if (nav_ac_ptr_->isServerConnected())
  {
    nav_ac_ptr_->cancelGoal();
    // wait for action to be properly cancelled
    while (nav_ac_ptr_->getState() == actionlib::SimpleClientGoalState::ACTIVE)
    {
    }
    setMessage(" navigation goal was cancelled");
    setTaskResult(false);
  }
  else
  {
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
