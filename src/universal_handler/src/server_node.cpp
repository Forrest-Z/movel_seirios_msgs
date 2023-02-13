#include <universal_handler/server_node.h>
#include <movel_common_libs/json.hpp>
#include <ros_utils/ros_utils.h>
#include <std_msgs/String.h>
#include <actionlib_msgs/GoalID.h>

using json = nlohmann::json;

UniversalHandlerNode::UniversalHandlerNode(std::string name)
  : nh_private_("~")
  , as_(nh_, name, boost::bind(&UniversalHandlerNode::executeCb, this, _1), false)
  , server_name_(name)
  , paused_(false)
  , cancelled_(false)
  , navigation_state_("idle")
{
  ros::Time::waitForValid();
  if (!loadParams())
  {
    ROS_FATAL("[%s] Error during parameter loading. Shutting down.", server_name_.c_str());
    return;
  }

  ROS_INFO("[%s] All parameters loaded. Launching.", server_name_.c_str());

  heartbeat_pub_ = nh_.advertise<std_msgs::Bool>("universal_handler/heartbeat", 10);
  pause_status_pub_ = nh_.advertise<std_msgs::Bool>("universal_handler/pause_status", 1, true);

  pause_sub_ = nh_private_.subscribe("pause", 1, &UniversalHandlerNode::pauseCb, this);
  cancel_sub_ = nh_private_.subscribe("cancel", 1, &UniversalHandlerNode::cancelCb, this);

  heartbeat_timer_ = nh_.createTimer(ros::Duration(1.0 / p_loop_rate_), &UniversalHandlerNode::heartbeat, this);

  redis_client_ = nh_.serviceClient<movel_seirios_msgs::StringTrigger>("/movel_redis/set");

  // services topics
  ts_pause_pub_ = nh_.advertise<std_msgs::Bool>("task_supervisor/pause", 1);
  flexbe_pause_pub_ = nh_.advertise<std_msgs::Bool>(p_flexbe_cmd_ns_ + "/pause", 1);
  ts_cancel_pub_ = nh_.advertise<actionlib_msgs::GoalID>("task_supervisor/cancel", 1);
  flexbe_cancel_pub_ = nh_.advertise<actionlib_msgs::GoalID>(p_flexbe_server_ + "/cancel", 1);

  as_.start();

  // start task supervisor action client
  ROS_INFO("[%s] Establishing connection to task supervisor action server.", server_name_.c_str());

  ts_ac_ptr_ =
    std::make_shared<actionlib::SimpleActionClient<movel_seirios_msgs::RunTaskListAction>>(p_ts_server_, true);
  
  // start flexbe action client
  if (p_use_flexbe_)
  {
    ROS_INFO("[%s] Establishing connection to flexbe action server.", server_name_.c_str());

    flexbe_ac_ptr_ =
      std::make_shared<actionlib::SimpleActionClient<flexbe_msgs::BehaviorExecutionAction>>(p_flexbe_server_, true);
  }

  // publishPauseStatus();
}

bool UniversalHandlerNode::loadParams()
{
  ros_utils::ParamLoader loader(nh_private_);

  loader.get_required("loop_rate", p_loop_rate_);
  loader.get_required("task_supervisor/server", p_ts_server_);
  loader.get_required("task_supervisor/timeout", p_ts_server_timeout_);
  loader.get_required("flexbe/server", p_flexbe_server_);
  loader.get_required("flexbe/command_namespace", p_flexbe_cmd_ns_);
  loader.get_required("flexbe/timeout", p_flexbe_server_timeout_);
  
  loader.get_optional("use_flexbe", p_use_flexbe_, true);

  return loader.params_valid();
}

void UniversalHandlerNode::executeCb(const movel_seirios_msgs::UnifiedTaskGoalConstPtr& goal)
{
  if (paused_)
    paused_ = false;
  
  if (cancelled_)
    cancelled_ = false;
  
  unified_task_target_ = goal->target_service;

  if (unified_task_target_ == movel_seirios_msgs::UnifiedTaskGoal::TASK_SUPERVISOR)
  {
    ROS_INFO("[%s] Received goal for task supervisor", server_name_.c_str());

    if (!ts_ac_ptr_->waitForServer(ros::Duration(p_ts_server_timeout_)))
    {
      std::string msg = "Could not communicate with task supervisor server after waiting for "
        + std::to_string(p_ts_server_timeout_) + " seconds.";

      ROS_FATAL("[%s] %s", server_name_.c_str(), msg.c_str());

      completed_task_list_id_ = goal->task_list.id;
      resultReturnFailure(msg);
      return;
    }

    // construct and send goal
    movel_seirios_msgs::RunTaskListGoal ts_msg;
    ts_msg.task_list = goal->task_list;

    ts_ac_ptr_->sendGoal(ts_msg,
                         boost::bind(&UniversalHandlerNode::tsDoneCb, this, _1, _2),
                         boost::bind(&UniversalHandlerNode::tsActiveCb, this),
                         boost::bind(&UniversalHandlerNode::tsFeedbackCb, this, _1));

    while (current_goal_state_ != actionlib::SimpleClientGoalState::ACTIVE)
      ;
    
    bool update_navigation_state = false;
    if (ts_msg.task_list.tasks[0].type == 3 || ts_msg.task_list.tasks[0].type == 6)
    {
      navigation_state_ = "navigating";
      update_navigation_state = true;
    }
    else if (ts_msg.task_list.tasks[0].type == 2)
    {
      navigation_state_ = "mapping";
      update_navigation_state = true;
    }
    
    if (update_navigation_state)
    {
      movel_seirios_msgs::StringTrigger redis_srv;
      json redis_pair;
      redis_pair["navigation_state"]= navigation_state_;
      redis_srv.request.input = redis_pair.dump();

      ros::service::waitForService("/movel_redis/set", ros::Duration(10));
      if (!redis_client_.call(redis_srv))
      {
        ROS_WARN("[%s] Failed setting navigation state to redis.", server_name_.c_str());
      }
    }

  }
  else if (unified_task_target_ == movel_seirios_msgs::UnifiedTaskGoal::FLEXBE_STATE_MACHINE)
  {
    ROS_INFO("[%s] Received goal for flexbe", server_name_.c_str());

    if (!p_use_flexbe_)
    {
      std::string msg = "Param p_use_flexbe_ is false. Flexbe client is not set up";

      ROS_WARN("[%s] %s. Will not send goal to flexbe server.", server_name_.c_str(), msg.c_str());

      completed_task_list_id_ = 0; // flexbe has no task list ID
      resultReturnFailure(msg);
      return;
    }

    if (!flexbe_ac_ptr_->waitForServer(ros::Duration(p_flexbe_server_timeout_)))
    {
      std::string msg = "Could not communicate with flexbe server after waiting for "
        + std::to_string(p_flexbe_server_timeout_) + " seconds.";

      ROS_FATAL("[%s] %s", server_name_.c_str(), msg.c_str());

      completed_task_list_id_ = 0;
      resultReturnFailure(msg);
      return;
    }

    // construct and send goal
    json flexbe_payload = json::parse(goal->goal_payload);
    if (flexbe_payload.find("behavior_name") != flexbe_payload.end())
    {
      flexbe_msgs::BehaviorExecutionGoal behavior_msg;
      behavior_msg.behavior_name = flexbe_payload["behavior_name"];
      behavior_msg.arg_keys = flexbe_payload["arg_keys"].get<std::vector<std::string>>();
      behavior_msg.arg_values = flexbe_payload["arg_values"].get<std::vector<std::string>>();
      behavior_msg.input_keys = flexbe_payload["input_keys"].get<std::vector<std::string>>();
      behavior_msg.input_values = flexbe_payload["input_values"].get<std::vector<std::string>>();
      
      flexbe_ac_ptr_->sendGoal(behavior_msg,
                               boost::bind(&UniversalHandlerNode::flexbeDoneCb, this, _1, _2),
                               boost::bind(&UniversalHandlerNode::flexbeActiveCb, this),
                               boost::bind(&UniversalHandlerNode::flexbeFeedbackCb, this, _1));

      while (current_goal_state_ != actionlib::SimpleClientGoalState::ACTIVE)
        ;
    }
    else
    {
      std::string msg = "Malformed payload";

      completed_task_list_id_ = 0; // flexbe has no ID
      resultReturnFailure(msg);
      return;
    }
  }

  ros::Rate r(p_loop_rate_);

  // wait until goal is completed or preempted
  while (ros::ok() && !isGoalDone())
  {
    if (isGoalCancelled() && !cancelled_)
    {
      ROS_INFO("[%s] Cancel received from user", server_name_.c_str());

      // actionlib_msgs::GoalID all_goals;

      // if (unified_task_target_ == movel_seirios_msgs::UnifiedTaskGoal::TASK_SUPERVISOR)
      // {
      //   ts_cancel_pub_.publish(all_goals);
      //   if (p_use_flexbe_)
      //     flexbe_cancel_pub_.publish(all_goals);
      // }
      // else if (unified_task_target_ == movel_seirios_msgs::UnifiedTaskGoal::FLEXBE_STATE_MACHINE)
      // {
      //   flexbe_cancel_pub_.publish(all_goals);
      //   ts_cancel_pub_.publish(all_goals);
      // }

      cancelled_ = true;
    }

    r.sleep();
  }

  // set universal handler goal status and return result
  switch (current_goal_state_)
  {
    case actionlib::SimpleClientGoalState::SUCCEEDED:
      resultReturnSuccess(result_message_);
      break;
    case actionlib::SimpleClientGoalState::PREEMPTED:
      resultReturnPreempted(result_message_);
      break;
    case actionlib::SimpleClientGoalState::ABORTED:
    case actionlib::SimpleClientGoalState::RECALLED:
    case actionlib::SimpleClientGoalState::REJECTED:
    case actionlib::SimpleClientGoalState::LOST:
      resultReturnFailure(result_message_);
      break;
  }

  return;
}

bool UniversalHandlerNode::isGoalDone()
{
  switch (current_goal_state_)
  {
    case actionlib::SimpleClientGoalState::SUCCEEDED:
    case actionlib::SimpleClientGoalState::PREEMPTED:
    case actionlib::SimpleClientGoalState::ABORTED:
    case actionlib::SimpleClientGoalState::RECALLED:
    case actionlib::SimpleClientGoalState::REJECTED:
    case actionlib::SimpleClientGoalState::LOST:
      return true;
    default:
      return false;
  }
}

bool UniversalHandlerNode::isGoalCancelled()
{
  return (as_.isPreemptRequested() || !ros::ok());
}

void UniversalHandlerNode::resultReturnPreempted(std::string cancellation_msg)
{
  ROS_WARN("[%s] Task with tasklist ID %d preempted with message: %s", server_name_.c_str(), completed_task_list_id_, cancellation_msg.c_str());

  result_.success = false;
  result_.id = completed_task_list_id_;
  std_msgs::String msg;
  msg.data = cancellation_msg;
  result_.message = msg;
  as_.setPreempted(result_);

  current_goal_state_ = actionlib::SimpleClientGoalState::PREEMPTED;

  if (paused_)
  {
    paused_ = false;
    // publishPauseStatus();
  }
}

void UniversalHandlerNode::resultReturnFailure(std::string failure_message)
{
  ROS_ERROR("[%s] Task with tasklist ID %d failed with message: %s", server_name_.c_str(), completed_task_list_id_, failure_message.c_str());

  result_.success = false;
  result_.id = completed_task_list_id_;
  std_msgs::String msg;
  msg.data = failure_message;
  result_.message = msg;
  as_.setAborted(result_);

  current_goal_state_ = actionlib::SimpleClientGoalState::ABORTED;
  
  if (paused_)
  {
    paused_ = false;
    // publishPauseStatus();
  }
}

void UniversalHandlerNode::resultReturnSuccess(std::string success_message)
{
  ROS_INFO("[%s] Task with tasklist ID %d succeeded with message: %s", server_name_.c_str(), completed_task_list_id_, success_message.c_str());

  result_.success = true;
  result_.id = completed_task_list_id_;
  std_msgs::String msg;
  msg.data = success_message;
  result_.message = msg;
  as_.setSucceeded(result_);

  current_goal_state_ = actionlib::SimpleClientGoalState::SUCCEEDED;

  if (paused_)
  {
    paused_ = false;
    // publishPauseStatus();
  }
}

void UniversalHandlerNode::pauseCb(const std_msgs::Bool::ConstPtr& msg)
{
  if (current_goal_state_ == actionlib::SimpleClientGoalState::ACTIVE)
  {
    if (msg->data && !paused_)
    {
      ROS_INFO("[%s] Pausing current task", server_name_.c_str());
      
      paused_ = true;
      std_msgs::Bool pause;
      pause.data = paused_;
      
      if (p_use_flexbe_)
        flexbe_pause_pub_.publish(pause);
      ts_pause_pub_.publish(pause);
    }
    else if (!msg->data && paused_)
    {
      ROS_INFO("[%s] Resuming current task", server_name_.c_str());
      
      paused_ = false;
      std_msgs::Bool pause;
      pause.data = paused_;

      if (p_use_flexbe_)
        flexbe_pause_pub_.publish(pause);
      ts_pause_pub_.publish(pause);
    }

    publishPauseStatus();
  }
  else
    ROS_WARN("[%s] No active task to pause/resume", server_name_.c_str());
}

void UniversalHandlerNode::cancelCb(const actionlib_msgs::GoalID::ConstPtr& msg)
{
  ROS_INFO("[%s] Received cancel command from user.", server_name_.c_str());
  actionlib_msgs::GoalID cancel_msg;
  cancel_msg = latest_ts_goal_id_;
  ts_cancel_pub_.publish(cancel_msg);
  if (p_use_flexbe_)
  {
    cancel_msg = latest_flexbe_goal_id_;
    flexbe_cancel_pub_.publish(cancel_msg);
  }

  cancelled_ = true;
}

void UniversalHandlerNode::publishPauseStatus()
{
  std_msgs::Bool pause_status;
  pause_status.data = paused_;
  pause_status_pub_.publish(pause_status);
}

void UniversalHandlerNode::heartbeat(const ros::TimerEvent& e)
{
  std_msgs::Bool hb;
  hb.data = false;
  if (ros::ok())
    hb.data = true;
  
  heartbeat_pub_.publish(hb);
}

void UniversalHandlerNode::tsDoneCb(const actionlib::SimpleClientGoalState& state,
                                    const movel_seirios_msgs::RunTaskListResultConstPtr& result)
{
  current_goal_state_ = state.state_;
  completed_task_list_id_ = result->id;
  result_message_ = result->message.data;
}

void UniversalHandlerNode::tsActiveCb()
{
  current_goal_state_ = actionlib::SimpleClientGoalState::ACTIVE;
}

void UniversalHandlerNode::tsFeedbackCb(const movel_seirios_msgs::RunTaskListFeedbackConstPtr& feedback)
{
  ROS_INFO_STREAM("[" << server_name_ << "] Received feedback:" << *feedback);

  feedback_.completed_task = feedback->completed_task;
  as_.publishFeedback(feedback_);
}

void UniversalHandlerNode::tsStatusCb(const actionlib_msgs::GoalStatusArrayConstPtr& msg)
{
  if (msg->status_list.size() > 0)
  {
    latest_ts_goal_id_ = msg->status_list[0].goal_id;
  }
}

void UniversalHandlerNode::flexbeDoneCb(const actionlib::SimpleClientGoalState& state,
                                        const flexbe_msgs::BehaviorExecutionResultConstPtr& result)
{
  if (result->outcome == "preempted")
    current_goal_state_ = actionlib::SimpleClientGoalState::PREEMPTED;
  else
    current_goal_state_ = state.state_;

  completed_task_list_id_ = 0; // flexbe has no ID
  result_message_ = result->outcome;
}

void UniversalHandlerNode::flexbeActiveCb()
{
  current_goal_state_ = actionlib::SimpleClientGoalState::ACTIVE;
}

void UniversalHandlerNode::flexbeFeedbackCb(const flexbe_msgs::BehaviorExecutionFeedbackConstPtr& feedback)
{
  ROS_INFO_STREAM("[" << server_name_ << "] Received feedback:" << *feedback);

  feedback_.current_state.data = feedback->current_state;
  as_.publishFeedback(feedback_);
}

void UniversalHandlerNode::flexbeStatusCb(const actionlib_msgs::GoalStatusArrayConstPtr& msg)
{
  if (msg->status_list.size() > 0)
  {
    latest_flexbe_goal_id_ = msg->status_list[0].goal_id;
  }
}

/* MAIN */
int main(int argc, char** argv)
{
  ros::init(argc, argv, "universal_handler");

  std::string nodeName(ros::this_node::getName(), ros::this_node::getNamespace().length(), std::string::npos);
  UniversalHandlerNode h(nodeName);

  ros::spin();

  return 0;
}