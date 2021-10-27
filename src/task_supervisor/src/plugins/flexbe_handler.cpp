#include <vector>
#include <task_supervisor/plugins/flexbe_handler.h>
#include <task_supervisor/json.hpp>
#include <pluginlib/class_list_macros.h>
#include <ros_utils/ros_utils.h>

PLUGINLIB_EXPORT_CLASS(task_supervisor::FlexbeHandler, task_supervisor::TaskHandler);

using json = nlohmann::json;

namespace task_supervisor
{

FlexbeHandler::FlexbeHandler()
 : is_healthy_(true)
{
}


bool FlexbeHandler::setupHandler()
{
  if (!loadParams())
  {
    ROS_FATAL("[%s] Error during parameter loading. Shutting down.", name_.c_str());
    return false;
  }

  flexbe_repeat_srv_ = nh_handler_.advertiseService("/flexbe_handler/repeat", &FlexbeHandler::flexbeRepeatCommandCb, this);

  flexbe_logs_sub_ = nh_handler_.subscribe(p_flexbe_topics_prefix_ + "/log", 1, &FlexbeHandler::flexbeLogsCb, this);
  flexbe_status_sub_ = nh_handler_.subscribe(p_flexbe_topics_prefix_ + "/status", 1, &FlexbeHandler::flexbeStatusCb, this);
  flexbe_current_state_sub_ = nh_handler_.subscribe(p_flexbe_topics_prefix_ + "/behavior_update", 1, &FlexbeHandler::flexbeCurrentStateCb, this);

  flexbe_pause_pub_ = nh_handler_.advertise<std_msgs::Bool>(p_flexbe_topics_prefix_ + "/command/pause", 1);
  flexbe_repeat_pub_ = nh_handler_.advertise<std_msgs::Empty>(p_flexbe_topics_prefix_ + "/command/repeat", 1);

  return true;
}


bool FlexbeHandler::loadParams()
{
  ros_utils::ParamLoader param_loader(nh_handler_);

  std::string default_prefix = "flexbe";
  std::string default_server = "flexbe/execute_behavior";

  param_loader.get_optional("enable_flexbe_logs", p_enable_flexbe_logs_, true);
  param_loader.get_optional("flexbe_topics_prefix", p_flexbe_topics_prefix_, default_prefix);
  param_loader.get_optional("flexbe_server", p_flexbe_server_, default_server);
  param_loader.get_optional("flexbe_server_timeout", p_flexbe_server_timeout_, 2.0);

  if(p_flexbe_topics_prefix_.at(0) != '/')
    p_flexbe_topics_prefix_ = "/" + p_flexbe_topics_prefix_;
  
  return param_loader.params_valid();
}


void FlexbeHandler::flexbeLogsCb(const flexbe_msgs::BehaviorLog::ConstPtr& msg)
{
  std::string message = msg->text;
  uint8_t code = msg->status_code;

  if (p_enable_flexbe_logs_)
  {
    if (code == 0 || code == 2)
      ROS_INFO("[%s] [FlexBE Log] %s", name_.c_str(), message.c_str());
    if (code == 1)
      ROS_WARN("[%s] [FlexBE Log] %s", name_.c_str(), message.c_str());
    if (code == 3)
      ROS_ERROR("[%s] [FlexBE Log] %s", name_.c_str(), message.c_str());
    if (code == 10)
      ROS_DEBUG("[%s] [FlexBE Log] %s", name_.c_str(), message.c_str());
  }
}


void FlexbeHandler::flexbeStatusCb(const flexbe_msgs::BEStatus::ConstPtr& msg)
{
  if (behavior_status_ != msg->code)
  {
    behavior_status_ = msg->code;
    ROS_INFO("[%s] [FlexBE Behavior] Behavior status updated: %d", name_.c_str(), behavior_status_);
  }
}


void FlexbeHandler::flexbeCurrentStateCb(const std_msgs::String::ConstPtr& msg)
{
  current_active_state_ = msg->data;
}


bool FlexbeHandler::flexbeRepeatCommandCb(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res)
{
  if (behavior_status_ == flexbe_msgs::BEStatus::STARTED)
  {
    std_msgs::Empty repeat_trigger;
    flexbe_repeat_pub_.publish(repeat_trigger);

    res.message = "Repeat command sent.";
    res.success = true;
  }
  else
  {
    ROS_WARN("[%s] No flexbe behavior is active", name_.c_str());
    res.message = "No flexbe behavior is active.";
    res.success = false;
  }

  return true;
}


bool FlexbeHandler::startActionClient()
{
  flexbe_ac_ptr_ =
    std::make_shared<actionlib::SimpleActionClient<flexbe_msgs::BehaviorExecutionAction>>(p_flexbe_server_, true);
  
  if (!flexbe_ac_ptr_->waitForServer(ros::Duration(p_flexbe_server_timeout_)))
  {
    ROS_ERROR("[%s] Could not communicate with flexbe server after waiting for %f seconds. Shutting down.",
              name_.c_str(), p_flexbe_server_timeout_);
    message_ = "Could not communicate with flexbe server after waiting for "
      + std::to_string(p_flexbe_server_timeout_) + " seconds.";
    
    return false;
  }

  ROS_INFO("[%s] Communication with flexbe action server started", name_.c_str());

  return true;
}

ReturnCode FlexbeHandler::runTask(movel_seirios_msgs::Task& task, std::string& error_message)
{
  task_active_ = true;
  task_parsed_ = false;
  start_ = ros::Time::now();

  if (startActionClient())
  {
    ROS_INFO("[%s] Task payload %s", name_.c_str(), task.payload.c_str());

    // parse payload
    json payload = json::parse(task.payload);
    if (payload.find("behavior_name") != payload.end())
    {
      // input goal
      flexbe_msgs::BehaviorExecutionGoal behavior_msg;
      behavior_msg.behavior_name = payload["behavior_name"];
      behavior_msg.arg_keys = payload["arg_keys"].get<std::vector<std::string>>();
      behavior_msg.arg_values = payload["arg_values"].get<std::vector<std::string>>();
      behavior_msg.input_keys = payload["input_keys"].get<std::vector<std::string>>();
      behavior_msg.input_values = payload["input_values"].get<std::vector<std::string>>();

      bool success = behaviorLoop(behavior_msg);
      setTaskResult(success);
    }
    else
    {
      setMessage("Malformed payload\n"
                 "Example:\n"
                 "{\n"
                 "  \"behavior_name\": \"example_behavior\"\n"
                 "  \"arg_keys\": [\"param_1\", \"param_2\", \"param_3\"]\n"
                 "  \"arg_values\": [\"true\", \"1.0\", \"some string\"]\n"
                 "  \"input_keys\": [\"userdata_1\", \"userdata_2\"]\n"
                 "  \"input_keys\": [\"userdata_1_value\", \"userdata_2_value\"]\n"
                 "}");
      error_message = message_;
      setTaskResult(false);
    }
  }
  else
  {
    setMessage("Unable to talk to flexbe action server on " + p_flexbe_server_);
    error_message = message_;
    setTaskResult(false);
  }

  return code_;
}


bool FlexbeHandler::behaviorLoop(const flexbe_msgs::BehaviorExecutionGoal& behavior_goal)
{
  // send goal to action server
  flexbe_ac_ptr_->sendGoal(behavior_goal);

  // wait for behavior to start
  while (behavior_status_ != flexbe_msgs::BEStatus::STARTED)
  {
  }

  while (behavior_status_ == flexbe_msgs::BEStatus::STARTED)
  {
  }

  if (behavior_status_ == flexbe_msgs::BEStatus::FINISHED)
  {
    return true;
  }

  if (behavior_status_ == flexbe_msgs::BEStatus::FAILED)
  {
    return false;
  }

  // any other behavior
  return false;
}


void FlexbeHandler::cancelTask()
{
  if (flexbe_ac_ptr_->isServerConnected())
  {
    flexbe_ac_ptr_->cancelGoal();

    while (flexbe_ac_ptr_->getState() == actionlib::SimpleClientGoalState::ACTIVE)
    {
    }

    setMessage("Flexbe behavior was cancelled.");
    setTaskResult(false);
  }
  else
  {
    setMessage("Server disconnected before flexbe behavior was cancelled.");
    setTaskResult(false);
  }

  task_parsed_ = true;
  task_active_ = false;
  task_paused_ = false;
}

void FlexbeHandler::pauseTask()
{
  if (behavior_status_ == flexbe_msgs::BEStatus::STARTED)
  {
    std_msgs::Bool pause_trigger;
    pause_trigger.data = true;
    flexbe_pause_pub_.publish(pause_trigger);
  }
  else
  {
    ROS_WARN("[%s] No flexbe behavior is active", name_.c_str());
  }
}

void FlexbeHandler::resumeTask()
{
  if (behavior_status_ == flexbe_msgs::BEStatus::STARTED)
  {
    std_msgs::Bool pause_trigger;
    pause_trigger.data = false;
    flexbe_pause_pub_.publish(pause_trigger);
  }
  else
  {
    ROS_WARN("[%s] No flexbe behavior is active", name_.c_str());
  }
}

} // namespace task_supervisor