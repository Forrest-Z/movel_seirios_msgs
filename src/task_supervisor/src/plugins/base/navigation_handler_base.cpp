#include <task_supervisor/plugins/base/navigation_handler_base.h>
#include <movel_common_libs/json.hpp>
#include <pluginlib/class_list_macros.h>
#include <actionlib_msgs/GoalID.h>
#include <movel_seirios_msgs/GetReachableSubplan.h>
#include <type_traits>


using json = nlohmann::json;

namespace task_supervisor
{

NavigationHandlerBase::NavigationHandlerBase() : 
  enable_human_detection_(false),
  human_detection_score_(0.0),
  task_cancelled_(false),
  isHealthy_(true),
  is_navigating_to_transit_point_(false)
{
}


bool NavigationHandlerBase::setupHandler()
{
  if (!loadParams()) {
    ROS_FATAL("[%s] Error during parameter loading. Shutting down.", name_.c_str());
    return false;
  }
  enable_human_detection_srv_ = nh_handler_.advertiseService("/enable_human_detection", &NavigationHandlerBase::enableHumanDetectionCB, this);
  enable_best_effort_goal_srv_ = nh_handler_.advertiseService("/enable_best_effort_goal", &NavigationHandlerBase::enableBestEffortGoalCB, this);
  make_sync_plan_client_ = nh_handler_.serviceClient<nav_msgs::GetPlan>("/planner_utils/make_sync_plan");
  make_reachable_plan_client_ = nh_handler_.serviceClient<nav_msgs::GetPlan>("/planner_utils/make_reachable_plan");
  human_detection_sub_ = nh_handler_.subscribe(p_human_detection_topic_, 1, &NavigationHandlerBase::humanDetectionCB, this);
  robot_pose_sub_ = nh_handler_.subscribe("/pose", 1, &NavigationHandlerBase::robotPoseCB, this);
  loc_report_sub_ = nh_handler_.subscribe("/task_supervisor/health_report", 1, &NavigationHandlerBase::locReportingCB, this);
  movebase_cancel_pub_ = nh_handler_.advertise<actionlib_msgs::GoalID>("/move_base/cancel", 1);
  obstruction_status_pub_ = nh_handler_.advertise<movel_seirios_msgs::ObstructionStatus>("/obstruction_status", 1);
  last_waypoint_pebble_client_ = nh_handler_.serviceClient<std_srvs::SetBool>("/at_last_waypoint_pebble");
  return true;
}


template <typename param_type>
bool NavigationHandlerBase::load_param_util(std::string param_name, param_type& output)
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


bool NavigationHandlerBase::loadParams()
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
  if (!load_param_util("smooth_transition_dist", p_smooth_transition_dist_)) { return false; }
  return true;
}


bool NavigationHandlerBase::enableHumanDetectionCB(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res)
{
  enable_human_detection_ = req.data;
  if(req.data)
    ROS_INFO("[%s] %s", name_.c_str(), p_enable_human_detection_msg_.c_str());
  else
    ROS_INFO("[%s] %s", name_.c_str(), p_disable_human_detection_msg_.c_str());
  res.success = true;
  return true;
}


void NavigationHandlerBase::humanDetectionCB(const std_msgs::Float64::ConstPtr& msg)
{
  boost::unique_lock<boost::mutex> scoped_lock(mtx_);
  human_detection_score_ = msg->data;
}


bool NavigationHandlerBase::enableBestEffortGoalCB(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res)
{
  p_enable_best_effort_goal_ = req.data;
  ROS_INFO("[%s] %s navigation with best effort goal", name_.c_str(), p_enable_best_effort_goal_ ? "Enabled" : "Disabled");
  res.success = true;
  return true;
}


void NavigationHandlerBase::robotPoseCB(const geometry_msgs::Pose::ConstPtr& msg)
{
  if (task_active_) { robot_pose_ = *msg; }
}


bool NavigationHandlerBase::start_ActionClient()
{
  nav_ac_ptr_ =
      std::make_shared<actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>>(p_navigation_server_, true);

  if (!nav_ac_ptr_->waitForServer(ros::Duration(p_server_timeout_))) {
    ROS_ERROR("[%s] Could not communicate with navigation server after waiting for %f seconds. Shutting down.",
              name_.c_str(), p_server_timeout_);
    message_ = "Could not communicate with navigation server after waiting for %f seconds. ";
    return false;
  }
  ROS_INFO("[%s] Communication with move base action server started", name_.c_str());
  return true;
}


void NavigationHandlerBase::startWithDetection()
{
  // No human detected
  if(human_detection_score_ < p_human_detection_min_score_)
    navACSendGoal(current_sub_goal_);
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
      navACSendGoal(current_sub_goal_);
    }
    else {
      task_cancelled_ = false;
      return;
    }
  } 
}


NavigationHandlerBase::NavLoopResult NavigationHandlerBase::navigationLoop()
{
  bool navigating = true;
  bool succeeded = false;
  std::string state_msg;
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
      navACSendGoal(current_sub_goal_);
    }
    else if (!isTaskPaused() && !navigating && enable_human_detection_) {
      // No human detected and robot stopped
      if (human_detection_score_ < p_human_detection_min_score_) {
        ROS_INFO("[%s] No human detected, resuming navigation", name_.c_str());
        navigating = true;
        navACSendGoal(current_sub_goal_);
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
      state_msg = state.getText();
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
    // smooth transitiion
    if(navigating && !is_last_waypoint_) {
      double dx = robot_pose_.position.x - current_sub_goal_.position.x;
      double dy = robot_pose_.position.y - current_sub_goal_.position.y;
      double dist_to_goal = std::sqrt(dx*dx + dy*dy);
      if (dist_to_goal <= p_smooth_transition_dist_)
        return NavLoopResult::SMOOTH_TRANSITION;
    }
    // loop
    ros::Duration(0.1).sleep();
  }
  // result
  if (!task_cancelled_ && succeeded) {
    if (is_last_waypoint_)
      return NavLoopResult::FINAL_WAYPOINT_REACHED;
    else
      return NavLoopResult::SUBGOAL_REACHED;
  }
  else {
    // because of simpleactionserver limitations, we use the description text sent by movel_move_base during task
    // abortion to determine that the current goal should immediately fail and not proceed to best efort navigation
    // due to the path being obstructed.
    // This only happens when movel_move_base is used and stop_at_obstacle is enabled and if one of these criteria
    // is met:
    // 1. state is PLANNING but no valid plan (after partial plan check) is found after timeout
    // 2. state is CONTROLLING and an obstruction is found within a threshold distance to the robot and
    //    enable_replan_after_timeout_ is disabled
    // Refer to move_base.cpp in movel_move_base package for more details.
    if (state_msg == "MOVE_BASE_FORCED_FAILURE")
      return NavLoopResult::MOVE_BASE_FORCED_FAILURE;
    else
      return NavLoopResult::FAILED;
  }
}


NavigationHandlerBase::NavLoopResult NavigationHandlerBase::navigationAttemptGoal()
{
  if (!enable_human_detection_) {
    navACSendGoal(current_sub_goal_);
    return navigationLoop();
  }
  else {
    startWithDetection();
    return navigationLoop();
  }
}


void NavigationHandlerBase::navigationDirect()
{
  std::vector<geometry_msgs::Pose>::const_iterator waypoint_it = waypoints_.begin() + start_at_idx_;
  int current_idx = start_at_idx_;
  while (!task_cancelled_ && isHealthy_)
  {
    ROS_INFO("[%s] Starting navigation to waypoint goal", name_.c_str());
    // start navigation
    current_sub_goal_ = *waypoint_it;
    is_last_waypoint_ = (waypoint_it + 1) == waypoints_.end();
    NavLoopResult nav_result = navigationAttemptGoal();
    if (task_cancelled_) { return; }
    if (nav_result == NavLoopResult::SMOOTH_TRANSITION) {
      ROS_INFO("[%s] Waypoint goal smooth transition", name_.c_str());
      json handler_feedback(current_idx++);
      publishHandlerFeedback(handler_feedback);
      waypoint_it++;
      continue;
    } 
    else if (nav_result == NavLoopResult::SUBGOAL_REACHED) {
      ROS_INFO("[%s] Waypoint goal success", name_.c_str());
      json handler_feedback(current_idx++);
      publishHandlerFeedback(handler_feedback);
      waypoint_it++;
      continue;
    } 
    else if (nav_result == NavLoopResult::FINAL_WAYPOINT_REACHED) {
      ROS_INFO("[%s] Final waypoint goal success", name_.c_str());
      json handler_feedback(current_idx);
      publishHandlerFeedback(handler_feedback);
      if (is_navigating_to_transit_point_)
      {
        // don't set task result just yet because we still have another round of navigation, just set code_
        code_ = ReturnCode::SUCCESS;
      }
      else
      {
        setTaskResult(true);
      }
      return;
    }
    else {   // nav failed, abort
      ROS_WARN("[%s] Waypoint goal failed, aborting", name_.c_str());
      setTaskResult(false);
      return;
    } 
  }
} 
  

void NavigationHandlerBase::navigationBestEffort()
{
  // main loop  
  // try until final waypoint goal is reached or no more best effort goals can be found
  CountdownTimer countdown_timer{};
  ros::Duration retry_sleep{p_best_effort_retry_sleep_sec_};
  bool retry_at_obstacle = false;
  std::vector<geometry_msgs::Pose>::const_iterator waypoint_it = waypoints_.begin() + start_at_idx_;
  int current_idx = start_at_idx_;
  while (!task_cancelled_ && isHealthy_)
  {
    // retry expiry check
    if (retry_at_obstacle) {
      // sleep to not spam while loop
      retry_sleep.sleep();
      // retry timed out, navigation failed
      if (countdown_timer.expired()) {
        ROS_WARN("[%s] Best effort retries timed out, aborting", name_.c_str());
        setTaskResult(false);
        return;  
      }
      ROS_INFO("[%s] Best effort retry at obstacle", name_.c_str());
    }
    // choose between direct waypoint goal or best effort goal
    nav_msgs::GetPlan srv{};
    srv.request.start.header.frame_id = "map";
    srv.request.goal.header.frame_id = "map"; 
    // trying direct goal to waypoint
    srv.request.start.pose = robot_pose_;   // current pose
    srv.request.goal.pose = *waypoint_it;   // waypoint goal
    // get sync/dirty plan at current robot pose
    ROS_INFO("[%s] Calling make_sync_plan_client service", name_.c_str());
    if (!make_sync_plan_client_.call(srv)) {
      ROS_WARN("[%s] Service call to make_sync_plan_client failed", name_.c_str());
      ROS_WARN("[%s] Waypoint goal failed", name_.c_str());
      ROS_WARN("[%s] Retrying with best effort goal", name_.c_str());
    }
    // direct goal exists, no best effort needed
    else if (srv.response.plan.poses.size() > 0) {
      ROS_INFO("[%s] Starting navigation to waypoint goal", name_.c_str());
      // UI toast msg
      if (is_obstructed_) {   // this check is to prevent spam
        reportObstruction(false, *waypoint_it);
        is_obstructed_ = false;
      }
      // update/disable retry loop 
      retry_at_obstacle = false;
      // start navigation
      current_sub_goal_ = *waypoint_it;
      is_last_waypoint_ = (waypoint_it + 1) == waypoints_.end();
      //Param to topic check if at last goal
      if (is_last_waypoint_)
      {
        std_srvs::SetBool decelerate;
        decelerate.request.data = true;
        last_waypoint_pebble_client_.call(decelerate);
      }
      NavLoopResult nav_result = navigationAttemptGoal();
      if (task_cancelled_) { return; }
      if (nav_result == NavLoopResult::SMOOTH_TRANSITION) {
        ROS_INFO("[%s] Waypoint goal smooth transition", name_.c_str());
        json handler_feedback(current_idx++);
        publishHandlerFeedback(handler_feedback);
        waypoint_it++;
        continue;
      } 
      else if (nav_result == NavLoopResult::SUBGOAL_REACHED) {
        ROS_INFO("[%s] Waypoint goal success", name_.c_str());
        json handler_feedback(current_idx++);
        publishHandlerFeedback(handler_feedback);
        waypoint_it++;
        continue;
      } 
      else if (nav_result == NavLoopResult::FINAL_WAYPOINT_REACHED) {
        ROS_INFO("[%s] Final waypoint goal success", name_.c_str());
        json handler_feedback(current_idx);
        publishHandlerFeedback(handler_feedback);
        if (is_navigating_to_transit_point_)
        {
          // don't set task result just yet because we still have another round of navigation, just set code_
          code_ = ReturnCode::SUCCESS;
        }
        else
        {
          setTaskResult(true);
        }
        return;
      }
      else if (nav_result == NavLoopResult::MOVE_BASE_FORCED_FAILURE) {
        ROS_INFO("[%s] Forced failure by move_base, not retrying with best effort", name_.c_str());
        setTaskResult(false);
        return;
      }
      else {   // nav failed, continue to best effort
        ROS_WARN("[%s] Waypoint goal failed", name_.c_str());
        ROS_WARN("[%s] Retrying with best effort goal", name_.c_str());
      } 
    }
    // trying best effort goal
    srv.request.start.pose = robot_pose_;   // update current pose
    ROS_INFO("[%s] Calling make_reachable_plan_client service", name_.c_str());
    if (!make_reachable_plan_client_.call(srv)) {
      ROS_WARN("[%s] Service call to make_reachable_plan_client failed, retrying best effort", name_.c_str());
      if (!retry_at_obstacle) { 
        retry_at_obstacle = true;
        ROS_INFO("[%s] Starting best effort timeout countdown", name_.c_str());
        countdown_timer.start(p_best_effort_retry_timeout_sec_);
      }
      continue;   // continue retry loop
    }
    // if the distance between reachable goal and robot pose is within p_smooth_transition_dist_ threshold,
    // it is assumed that we can't get any closer to the goal, so the timeout countdown shall start.
    double dx = robot_pose_.position.x - srv.response.plan.poses.back().pose.position.x;
    double dy = robot_pose_.position.y - srv.response.plan.poses.back().pose.position.y;
    if (std::sqrt(dx*dx + dy*dy) <= p_smooth_transition_dist_)
    {
      ROS_WARN("[%s] Cannot get a closer reachable goal to actual waypoint", name_.c_str());
      if (!retry_at_obstacle) { 
        retry_at_obstacle = true;
        ROS_INFO("[%s] Starting best effort timeout countdown", name_.c_str());
        countdown_timer.start(p_best_effort_retry_timeout_sec_);
      }
      continue;   // continue retry loop
    }
    // best effort goal exists
    {   // local scope (formatting purposes)
      ROS_INFO("[%s] Starting navigation to best effort goal", name_.c_str());
      geometry_msgs::Pose reachable_pose = srv.response.plan.poses.back().pose;
      // UI toast msg
      if (!is_obstructed_) {   // this check is to prevent spam
        reportObstruction(true, reachable_pose);
        is_obstructed_ = true;
      }
      // update/disable retry loop 
      bool call_reachable_plan_failed = retry_at_obstacle;
      retry_at_obstacle = false;
      // start navigation
      current_sub_goal_ = reachable_pose;
      is_last_waypoint_ = false;
      NavLoopResult nav_result = navigationAttemptGoal();
      if (task_cancelled_) { return; }
      if (nav_result == NavLoopResult::SMOOTH_TRANSITION) {
        ROS_INFO("[%s] Best effort goal smooth transition", name_.c_str());
        if (!(waypoint_it != waypoints_.end() && next(waypoint_it) == waypoints_.end())) {
          // not last waypoint, immediately go to next waypoint if stop at obstacle is disabled
          if (!stopAtObstacleEnabled()) {
            json handler_feedback(current_idx++);
            publishHandlerFeedback(handler_feedback);
            waypoint_it++;
          }
        }
        else {
          // prevent starting countdown more than once
          retry_at_obstacle = call_reachable_plan_failed;
        }
        continue;
      } 
      else if (nav_result == NavLoopResult::SUBGOAL_REACHED) {
        ROS_INFO("[%s] Best effort goal success", name_.c_str());
        if (!(waypoint_it != waypoints_.end() && next(waypoint_it) == waypoints_.end())) {
          if (!stopAtObstacleEnabled()) {
            json handler_feedback(current_idx++);
            publishHandlerFeedback(handler_feedback);
            waypoint_it++;
          }
        }
        continue;
      }
      else if (nav_result == NavLoopResult::FAILED) {
        ROS_WARN("[%s] Best effort goal failed (possibly from dynamic obstacle)", name_.c_str());
        ROS_WARN("[%s] Retrying ...", name_.c_str());
        continue;   // continue while loop and try waypoint goal again
      }
      else {
        ROS_ERROR("[%s] Best effort got wrong NavLoopResult", name_.c_str());
        ROS_ERROR("[%s] This should not happen, aborting task", name_.c_str());
        setTaskResult(false);
        return;
      }
    }
  }
}


bool NavigationHandlerBase::runTaskChooseNav(const std::vector<geometry_msgs::Pose>& goal_poses, int start_at_idx)
{
  // navigation
  task_cancelled_ = false;
  waypoints_ = goal_poses;
  start_at_idx_ = start_at_idx;
  //Deceleration planner
  std_srvs::SetBool decelerate;
  decelerate.request.data = false;
  last_waypoint_pebble_client_.call(decelerate);
  // best effort
  if (p_enable_best_effort_goal_) {
    ROS_INFO("[%s] Starting navigation - best effort: enabled", name_.c_str());
    // planner_utils service available
    if (make_reachable_plan_client_.exists())
      navigationBestEffort();
    // planner_utils service unavailable
    else {
      std::string service_name = make_reachable_plan_client_.getService();
      if (p_normal_nav_if_best_effort_unavailable_) {
        ROS_WARN("[%s] Services %s are not available, using normal navigation", name_.c_str(), service_name.c_str());
        navigationDirect();
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
    navigationDirect();
  }
  // check navigation success (for multimap nav)
  return code_ == ReturnCode::SUCCESS;
}


bool NavigationHandlerBase::runTaskChooseNav(const geometry_msgs::Pose& goal_pose, int start_at_idx)
{
  std::vector<geometry_msgs::Pose> goal_poses{goal_pose};
  return runTaskChooseNav(goal_poses, start_at_idx);
}


void NavigationHandlerBase::cancelTask()
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


void NavigationHandlerBase::locReportingCB(const movel_seirios_msgs::Reports::ConstPtr& msg)
{
  if (msg->handler == "localization_handler" && msg->healthy == false && task_active_)    // Localization failed
    isHealthy_ = false;
}


void NavigationHandlerBase::reportObstruction(bool status, const geometry_msgs::Pose& location)
{
  movel_seirios_msgs::ObstructionStatus report_obs;
  report_obs.reporter = "navigation_handler";
  report_obs.status = status ? "true" : "false";
  report_obs.location = location;
  obstruction_status_pub_.publish(report_obs);
}


void NavigationHandlerBase::navACSendGoal(const geometry_msgs::Pose& goal)
{
  move_base_msgs::MoveBaseGoal goal_msg;
  goal_msg.target_pose.header.stamp = ros::Time::now();
  goal_msg.target_pose.header.frame_id = "map";
  goal_msg.target_pose.pose = goal;
  nav_ac_ptr_->sendGoal(goal_msg);
}

bool NavigationHandlerBase::stopAtObstacleEnabled()
{
  ros::ServiceClient client = nh_handler_.serviceClient<std_srvs::Trigger>("/stop_obstacle_check");
  std_srvs::Trigger srv;
  if (client.call(srv))
  {
    if (srv.response.success)
      return true;
    else
      return false;
  }

  return false;
}

}  // namespace task_supervisor
