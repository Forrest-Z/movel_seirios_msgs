#include <task_supervisor/plugins/navigation_handler.h>
#include <task_supervisor/json.hpp>
#include <pluginlib/class_list_macros.h>


PLUGINLIB_EXPORT_CLASS(task_supervisor::NavigationHandler, task_supervisor::TaskHandler);

using json = nlohmann::json;


namespace task_supervisor
{
NavigationHandler::NavigationHandler() : enable_human_detection_(false),
					 human_detection_score_(0.0),
					 task_cancelled_(false)
{
}


bool NavigationHandler::setupHandler()
{
  if (!loadParams())
  {
    ROS_FATAL("[%s] Error during parameter loading. Shutting down.", name_.c_str());
    return false;
  }
  enable_human_detection_srv_ = nh_handler_.advertiseService("/enable_human_detection", &NavigationHandler::enableHumanDetectionCB, this);
  enable_best_effort_goal_srv_ = nh_handler_.advertiseService("/enable_best_effort_goal", &NavigationHandler::enableBestEffortGoalCB, this);
  make_plan_client_ = nh_handler_.serviceClient<nav_msgs::GetPlan>("/move_base/make_plan");
  make_reachable_plan_client_ = nh_handler_.serviceClient<nav_msgs::GetPlan>("/make_reachable_plan");
  human_detection_sub_ = nh_handler_.subscribe(p_human_detection_topic_, 1, &NavigationHandler::humanDetectionCB, this);
  robot_pose_sub_ = nh_handler_.subscribe("/pose", 1, &NavigationHandler::robotPoseCB, this);
  return true;
}


bool NavigationHandler::loadParams()
{
  ROS_WARN("[%s] Loading of plugin parameters by ros_utils has not been implemented. Loading directly from Parameter "
           "Server instead.",
           name_.c_str());
  
  // TODO: clean up param loading using lambda function

  // server_timeout
  if (!nh_handler_.getParam("server_timeout", p_server_timeout_)) {
    ROS_ERROR("[%s] Failed to load parameter: server_timeout", name_.c_str());
    return false;
  }
  ROS_INFO("[%s] server_timeout: %f", name_.c_str(), p_server_timeout_);
  // static_paths
  if (!nh_handler_.getParam("static_paths", p_static_paths_)) {
    ROS_ERROR("[%s] Failed to load parameter: static_paths", name_.c_str());
    return false;
  }
  ROS_INFO("[%s] static_paths: %s", name_.c_str(), p_static_paths_ ? "true" : "false");
  // navigation_server
  if (!nh_handler_.getParam("navigation_server", p_navigation_server_)) {
    ROS_ERROR("[%s] Failed to load parameter: navigation_server", name_.c_str());
    return false;
  }
  ROS_INFO("[%s] navigation_server: %s", name_.c_str(), p_navigation_server_.c_str());
  // human_detection_min_score
  if (!nh_handler_.getParam("human_detection_min_score", p_human_detection_min_score_)) {
    ROS_ERROR("[%s] Failed to load parameter: human_detection_min_score", name_.c_str());
    return false;
  }
  ROS_INFO("[%s] human_detection_min_score: %f", name_.c_str(), p_human_detection_min_score_);
  // human_detection_topic
  if (!nh_handler_.getParam("human_detection_topic", p_human_detection_topic_)) {
    ROS_ERROR("[%s] Failed to load parameter: human_detection_topic", name_.c_str());
    return false;
  }
  ROS_INFO("[%s] human_detection_topic: %s", name_.c_str(), p_human_detection_topic_.c_str());
  // enable_human_detection_msg
  if (!nh_handler_.getParam("enable_human_detection_msg", p_enable_human_detection_msg_)) {
    ROS_ERROR("[%s] Failed to load parameter: enable_human_detection_msg", name_.c_str());
    return false;
  }
  ROS_INFO("[%s] enable_human_detection_msg: %s", name_.c_str(), p_enable_human_detection_msg_.c_str());
  // disable_human_detection_msg
  if (!nh_handler_.getParam("disable_human_detection_msg", p_disable_human_detection_msg_)) {
    ROS_ERROR("[%s] Failed to load parameter: disable_human_detection_msg", name_.c_str());
    return false;
  }
  ROS_INFO("[%s] disable_human_detection_msg: %s", name_.c_str(), p_disable_human_detection_msg_.c_str());
  // enable_best_effort_goal
  if (!nh_handler_.getParam("enable_best_effort_goal", p_enable_best_effort_goal_)) {
    ROS_ERROR("[%s] Failed to load parameter: enable_best_effort_goal", name_.c_str());
    return false;
  }
  ROS_INFO("[%s] enable_best_effort_goal: %s", name_.c_str(), p_enable_best_effort_goal_ ? "true" : "false");
  // normal_nav_if_best_effort_unavailable
  if (!nh_handler_.getParam("normal_nav_if_best_effort_unavailable", p_normal_nav_if_best_effort_unavailable_)) {
    ROS_ERROR("[%s] Failed to load parameter: normal_nav_if_best_effort_unavailable", name_.c_str());
    return false;
  }
  ROS_INFO("[%s] normal_nav_if_best_effort_unavailable: %s", name_.c_str(), p_normal_nav_if_best_effort_unavailable_ ? "true" : "false");
  
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


void NavigationHandler::startWithDetection(move_base_msgs::MoveBaseGoal goal)
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


void NavigationHandler::navigationLoop(const move_base_msgs::MoveBaseGoal& goal)
{
  bool navigating = true;
  bool succeeded = false;

  // Loops until cancellation is called or navigation task is complete
  while(!task_cancelled_)
  {
    if (isTaskPaused() && navigating)
    {
      ROS_INFO("[%s] Navigation paused", name_.c_str());
      navigating = false;
      nav_ac_ptr_->cancelGoal();
    }
    else if (!isTaskPaused() && !navigating && !enable_human_detection_)
    {
      ROS_INFO("[%s] Navigation resumed", name_.c_str());
      navigating = true;
      nav_ac_ptr_->sendGoal(goal);
    }
    else if (!isTaskPaused() && !navigating && enable_human_detection_)
    {
      // No human detected and robot stopped
      if (human_detection_score_ < p_human_detection_min_score_)
      {
        ROS_INFO("[%s] No human detected, resuming navigation", name_.c_str());
        navigating = true;
        nav_ac_ptr_->sendGoal(goal);
      }
    }

    else if (!isTaskPaused() && navigating && enable_human_detection_)
    {
      // Human(s) detected and robot is moving
      if (human_detection_score_ > p_human_detection_min_score_)
      {
        ROS_INFO("[%s] Human(s) detected, pausing navigation", name_.c_str());
        navigating = false;
        nav_ac_ptr_->cancelGoal();
      }
    }
    actionlib::SimpleClientGoalState state = nav_ac_ptr_->getState();
    if (state.isDone() && !isTaskPaused())
    {
      if (state == actionlib::SimpleClientGoalState::SUCCEEDED)
        succeeded = true;
      break;
    }
    ros::Duration(0.1).sleep();
  }
  if (!task_cancelled_)
  {
    if (succeeded)
      setTaskResult(true);
    else
      setTaskResult(false);
  }
  else
    task_cancelled_ = false;
}


void NavigationHandler::navigationAttemptGoal(const move_base_msgs::MoveBaseGoal& goal_msg)
{
  if (!enable_human_detection_) {
    nav_ac_ptr_->sendGoal(goal_msg);
    navigationLoop(goal_msg);
  }
  else {
    startWithDetection(goal_msg);
    navigationLoop(goal_msg);
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
  if (!make_plan_client_.call(srv)) {
    ROS_ERROR("[%s] Service call to make_plan failed", name_.c_str());
    setTaskResult(false);
    return;
  }
  // direct goal exists
  if (srv.response.plan.poses.size() > 0) {
    ROS_INFO("[%s] Starting navigation to goal", name_.c_str());
    move_base_msgs::MoveBaseGoal goal_msg;
    goal_msg.target_pose.pose = goal_pose;   // main goal
    goal_msg.target_pose.header.frame_id = "map";
    // start navigation
    navigationAttemptGoal(goal_msg);
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
  // try until main goal is reached or no more best effort goals can be found
  while (true)
  {
    // choose between direct goal or best effort goal
    nav_msgs::GetPlan srv{};
    srv.request.start.pose = robot_pose_;   // current pose
    srv.request.start.header.frame_id = "map";
    srv.request.goal.pose = goal_pose;   // main goal
    srv.request.goal.header.frame_id = "map"; 

    ROS_WARN_STREAM("robot_pose: \n" 
      << robot_pose_.position.x << '\n'
      << robot_pose_.position.y << '\n'
      << robot_pose_.position.z << '\n'
      << robot_pose_.orientation.x << '\n'
      << robot_pose_.orientation.y << '\n'
      << robot_pose_.orientation.z << '\n'
      << robot_pose_.orientation.w << '\n'
    );

    // srv.request.start.pose.position.x = -3.22;
    // srv.request.start.pose.position.y = -0.962;
    // srv.request.start.pose.position.z = 0.00644;
    // srv.request.start.pose.orientation.w = 1.0;
    // srv.request.start.header.frame_id = "map";
    // srv.request.goal.pose.position.x = -1.02;
    // srv.request.goal.pose.position.y = 1.09;
    // srv.request.goal.pose.position.z = 0.00247;
    // srv.request.goal.pose.orientation.w = 1.0;
    // srv.request.goal.header.frame_id = "map"; 

    // trying direct goal
    // get plan from move_base at current robot pose
    if (!make_plan_client_.call(srv)) {
      ROS_ERROR("[%s] Service call to make_plan failed", name_.c_str());
      setTaskResult(false);
      return;
    }
    // direct goal exists, no best effort needed
    if (srv.response.plan.poses.size() > 0) {
      ROS_INFO("[%s] Starting navigation to goal", name_.c_str());
      move_base_msgs::MoveBaseGoal goal_msg;
      goal_msg.target_pose.pose = goal_pose;   // main goal
      goal_msg.target_pose.header.frame_id = "map";
      // start navigation
      navigationAttemptGoal(goal_msg);
      return;
    }
    // trying best effort goal
    // get plan from planner_utils at current robot pose
    if (!make_reachable_plan_client_.call(srv)) {
      ROS_ERROR("[%s] Service call to make_reachable_plan failed", name_.c_str());
      setTaskResult(false);
      return;
    }
    // best effort goal exists
    if (srv.response.plan.poses.size() > 0) {
      ROS_INFO("[%s] Starting navigation to best effort goal", name_.c_str());
      move_base_msgs::MoveBaseGoal goal_msg;
      goal_msg.target_pose.pose = srv.response.plan.poses.back().pose;   // last pose in best effort path 
      goal_msg.target_pose.header.frame_id = "map";
      // start navigation
      navigationAttemptGoal(goal_msg);
      continue;   // continue while loop and try main goal again
    }
    // no best effort goal, failure
    else {
      ROS_INFO("[%s] No best effort plan to goal exists", name_.c_str());
      setTaskResult(false);
      return;
    }
  }
}


ReturnCode NavigationHandler::runTask(movel_seirios_msgs::Task& task, std::string& error_message)
{
  task_active_ = true;
  task_parsed_ = false;
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
      // best effort
      if (p_enable_best_effort_goal_) {
        ROS_INFO("[%s] Starting navigation - best effort: enabled", name_.c_str());
        if (make_reachable_plan_client_.exists()) {
          navigationBestEffort(goal_pose);
        }
        else {   // planner_utils service unavailable
          std::string service_name = make_reachable_plan_client_.getService();
          if (p_normal_nav_if_best_effort_unavailable_) {
            ROS_WARN("[%s] Service %s is not available, using normal navigation", name_.c_str(), service_name.c_str());
            navigationDirect(goal_pose);
          }
          else {
            ROS_ERROR("[%s] Service %s is not available", name_.c_str(), service_name.c_str());
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

}  // namespace task_supervisor
