#include <universal_handler/task_router_server.h>
#include <universal_handler/json.hpp>
#include <ros_utils/ros_utils.h>

using json = nlohmann::json;

namespace UniversalHandler
{

TaskRouterServer::TaskRouterServer(std::string name)
  : nh_private_("~")
  , server_name_(name)
  , ts_paused_(false)
  , fb_paused_(false)
  , cancelled_(false)
{
  ros::Time::waitForValid();
  if (!loadParams())
  {
    ROS_FATAL("[%s] Error during parameter loading. Shutting down.", server_name_.c_str());
    return;
  }

  ROS_INFO("[%s] All parameters loaded. Launching.", server_name_.c_str());

  // temporary prefix to prevent conflict
  run_trail_srv_ = nh_.advertiseService("/task_router/run_trail", &TaskRouterServer::runTrailCb, this);
  run_waypoint_srv_ = nh_.advertiseService("/task_router/run_waypoint", &TaskRouterServer::runWaypointCb, this);
  run_flexbe_srv_ = nh_.advertiseService("/task_router/run_flexbe", &TaskRouterServer::runFlexbeCb, this);

  cancel_sub_ = nh_.subscribe("/task_router/cancel", 1, &TaskRouterServer::cancelCb, this);
  pause_sub_ = nh_.subscribe("/task_router/pause", 1, &TaskRouterServer::pauseCb, this);

  // redis
  redis_client_ = nh_.serviceClient<movel_seirios_msgs::StringTrigger>("/movel_redis/set");

  pause_status_pub_ = nh_.advertise<std_msgs::Bool>("/task_router/pause_status", 1);

  task_feedback_pub_ = nh_.advertise<movel_seirios_msgs::TaskFeedback>("/task_router/task_feedback", 1);

  subtask_feedback_pubs_[3] = nh_.advertise<movel_seirios_msgs::SubtaskFeedback>(
    "/task_router/subtask_feedback/type_3", 1);
  subtask_feedback_pubs_[6] = nh_.advertise<movel_seirios_msgs::SubtaskFeedback>(
    "/task_router/subtask_feedback/type_6", 1);

  // task supervisor
  ts_run_task_pub_ = nh_.advertise<movel_seirios_msgs::RunTaskListActionGoal>("/task_supervisor/goal", 1);
  ts_cancel_pub_ = nh_.advertise<actionlib_msgs::GoalID>("/task_supervisor/cancel", 1);
  ts_pause_pub_ = nh_.advertise<std_msgs::Bool>("/task_supervisor/pause", 1);
  ts_status_sub_ = nh_.subscribe("/task_supervisor/status", 1, &TaskRouterServer::tsStatusCb, this);
  ts_result_sub_ = nh_.subscribe("/task_supervisor/result", 1, &TaskRouterServer::tsResultCb, this);
  ts_subtask_feedback_sub_ = nh_.subscribe(
    "/task_supervisor/handler_feedback", 1, &TaskRouterServer::tsSubtaskFeedbackCb, this);

  // flexbe
  fb_run_task_pub_ = nh_.advertise<flexbe_msgs::BehaviorExecutionActionGoal>("/flexbe/execute_behavior/goal", 1);
  fb_cancel_pub_ = nh_.advertise<actionlib_msgs::GoalID>("/flexbe/execute_behavior/cancel", 1);
  fb_pause_pub_ = nh_.advertise<std_msgs::Bool>("/flexbe/command/pause", 1);
  fb_status_sub_ = nh_.subscribe("/flexbe/execute_behavior/status", 1, &TaskRouterServer::fbStatusCb, this);
  fb_result_sub_ = nh_.subscribe("/flexbe/execute_behavior/result", 1, &TaskRouterServer::fbResultCb, this);

  publishPauseStatus();
}

bool TaskRouterServer::loadParams()
{
  ROS_INFO("[%s] Loading params.", server_name_.c_str());
  return true;
}

bool TaskRouterServer::runTrailCb(movel_seirios_msgs::UniversalHandlerRunTask::Request &req,
                                  movel_seirios_msgs::UniversalHandlerRunTask::Response &res)
{
  resetStates();
  current_ts_task_.second = movel_seirios_msgs::TaskFeedback::IDLE;

  json payload = json::parse(req.payload);
  movel_seirios_msgs::RunTaskListActionGoal msg;
  std::string error_msg;
  ROS_INFO("[%s] [/universal_handler/run_trail] Received request with payload:\n%s",
    server_name_.c_str(), payload.dump().c_str());

  if (constructTrailTaskMsg(payload, msg, error_msg))
  {
    ts_run_task_pub_.publish(msg);
    res.success = true;
    res.message = "";

    updateNavigationState("navigating");
  }
  else
  {
    res.success = false;
    res.message = error_msg;
  }

  return true;
}

bool TaskRouterServer::runWaypointCb(movel_seirios_msgs::UniversalHandlerRunTask::Request &req,
                                     movel_seirios_msgs::UniversalHandlerRunTask::Response &res)
{
  resetStates();
  current_ts_task_.second = movel_seirios_msgs::TaskFeedback::IDLE;

  json payload = json::parse(req.payload);
  movel_seirios_msgs::RunTaskListActionGoal msg;
  std::string error_msg;
  ROS_INFO("[%s] [/universal_handler/run_waypoint] Received request with payload:\n%s",
    server_name_.c_str(), payload.dump().c_str());

  if (constructWaypointTaskMsg(payload, msg, error_msg))
  {
    ts_run_task_pub_.publish(msg);
    res.success = true;
    res.message = "";

    updateNavigationState("navigating");
  }
  else
  {
    res.success = false;
    res.message = error_msg;
  }

  return true;
}

bool TaskRouterServer::runFlexbeCb(movel_seirios_msgs::UniversalHandlerRunTask::Request &req,
                                   movel_seirios_msgs::UniversalHandlerRunTask::Response &res)
{
  resetStates();
  current_fb_task_.second = movel_seirios_msgs::TaskFeedback::IDLE;

  json payload = json::parse(req.payload);
  flexbe_msgs::BehaviorExecutionActionGoal msg;
  std::string error_msg;
  ROS_INFO("[%s] [/universal_handler/run_flexbe] Received request with payload:\n%s",
    server_name_.c_str(), payload.dump().c_str());

  if (constructFlexbeTaskMsg(payload, msg, error_msg))
  {
    fb_run_task_pub_.publish(msg);
    res.success = true;
    res.message = "";
  }
  else
  {
    res.success = false;
    res.message = error_msg;
  }

  return true;
}

void TaskRouterServer::cancelCb(const actionlib_msgs::GoalID::ConstPtr& msg)
{
  ROS_INFO("[%s] Received cancel command from user.", server_name_.c_str());
  // TODO: cancel by ID. Currently UH receives and sends empty GoalID (means cancel all).
  actionlib_msgs::GoalID cancel_msg;
  cancel_msg.stamp = msg->stamp;
  cancel_msg.id = msg->id;

  ts_cancel_pub_.publish(cancel_msg);
  fb_cancel_pub_.publish(cancel_msg);

  cancelled_ = true;
}

void TaskRouterServer::pauseCb(const std_msgs::Bool::ConstPtr& msg)
{
  ROS_INFO("[%s] Received pause command from user.", server_name_.c_str());
  if (current_fb_task_.second == movel_seirios_msgs::TaskFeedback::ACTIVE)
  {
    if (msg->data && !fb_paused_)
    {
      ROS_INFO("[%s] Pausing current flexbe task", server_name_.c_str());

      fb_paused_ = true;
      std_msgs::Bool pause;
      pause.data = fb_paused_;
      fb_pause_pub_.publish(pause);
    }
    else if (!msg->data && fb_paused_)
    {
      fb_paused_ = false;
      std_msgs::Bool pause;
      pause.data = fb_paused_;
      fb_pause_pub_.publish(pause);
    }
  }

  if (current_ts_task_.second == movel_seirios_msgs::TaskFeedback::ACTIVE)
  {
    if (msg->data && !ts_paused_)
    {
      ROS_INFO("[%s] Pausing current TS task", server_name_.c_str());

      ts_paused_ = true;
      std_msgs::Bool pause;
      pause.data = ts_paused_;
      ts_pause_pub_.publish(pause);
    }
    else if (!msg->data && ts_paused_)
    {
      ts_paused_ = false;
      std_msgs::Bool pause;
      pause.data = ts_paused_;
      ts_pause_pub_.publish(pause);
    }
  }

  publishPauseStatus();
}

bool TaskRouterServer::constructTrailTaskMsg(const json& payload,
                                             movel_seirios_msgs::RunTaskListActionGoal& msg,
                                             std::string& err_msg)
{
  try
  {
    // checking fields
    current_ts_task_.first = payload.at("taskId").get<std::string>();

    msg.goal_id.id = current_ts_task_.first;

    bool start_at_nearest;
    start_at_nearest = payload.at("startAtNearestPoint").get<bool>();

    std::string map_id;
    map_id = payload.at("mapId").get<std::string>();

    // subtask 1: type 3
    if (!start_at_nearest)
    {
      movel_seirios_msgs::Task type_3_task;
      type_3_task.id = 0;
      type_3_task.name = "Goto";
      type_3_task.type = 3;
      type_3_task.mapId = map_id;
      type_3_task.linear_velocity = payload.at("velocity").at("linear").get<double>();
      type_3_task.angular_velocity = payload.at("velocity").at("angular").get<double>();

      json type_3_payload = {
        {"path", {}},
        {"start_at_nearest_point", start_at_nearest}
      };

      json type_3_path = {
        {"position", {
          {"x", payload.at("points").at(0).at("position").at("x").get<double>()},
          {"y", payload.at("points").at(0).at("position").at("y").get<double>()},
          {"z", payload.at("points").at(0).at("position").at("z").get<double>()}
        }},
        {"orientation", {
          {"x", payload.at("points").at(0).at("orientation").at("x").get<double>()},
          {"y", payload.at("points").at(0).at("orientation").at("y").get<double>()},
          {"z", payload.at("points").at(0).at("orientation").at("z").get<double>()},
          {"w", payload.at("points").at(0).at("orientation").at("w").get<double>()}
        }},
        {"from_map", map_id},
        {"to_map", map_id},
        {"velocity", {
          {"linear_velocity", payload.at("velocity").at("linear").get<double>()},
          {"angular_velocity", payload.at("velocity").at("angular").get<double>()}
        }}
      };

      type_3_payload["path"].push_back(type_3_path);

      type_3_task.payload = type_3_payload.dump();

      msg.goal.task_list.tasks.push_back(type_3_task);
    }

    // subtask 2: type 6
    movel_seirios_msgs::Task type_6_task;
    type_6_task.id = 0;
    type_6_task.name = "Path";
    type_6_task.type = 6;
    type_6_task.mapId = map_id;
    type_6_task.linear_velocity = payload.at("velocity").at("linear").get<double>();
    type_6_task.angular_velocity = payload.at("velocity").at("angular").get<double>();

    json type_6_payload = {
      {"path", {}},
      {"start_at_nearest_point", start_at_nearest}
    };

    for (auto& elem : payload["points"])
    {
      json type_6_path = {
        {"position", {
          {"x", elem.at("position").at("x").get<double>()},
          {"y", elem.at("position").at("y").get<double>()},
          {"z", elem.at("position").at("z").get<double>()}
        }},
        {"orientation", {
          {"x", elem.at("orientation").at("x").get<double>()},
          {"y", elem.at("orientation").at("y").get<double>()},
          {"z", elem.at("orientation").at("z").get<double>()},
          {"w", elem.at("orientation").at("w").get<double>()}
        }},
        {"from_map", map_id},
        {"to_map", map_id},
        {"velocity", {
          {"linear_velocity", payload.at("velocity").at("linear").get<double>()},
          {"angular_velocity", payload.at("velocity").at("angular").get<double>()}
        }}
      };

      type_6_payload["path"].push_back(type_6_path);
    }

    type_6_task.payload = type_6_payload.dump();

    msg.goal.task_list.tasks.push_back(type_6_task);

    return true;
  }
  catch (json::exception& e)
  {
    ROS_FATAL("[%s] %s.", server_name_.c_str(), std::string(e.what()).c_str());
    err_msg = std::string(e.what());
    return false;
  }
}

bool TaskRouterServer::constructWaypointTaskMsg(const json& payload,
                                                movel_seirios_msgs::RunTaskListActionGoal& msg,
                                                std::string& err_msg)
{
  try
  {
    // checking fields
    current_ts_task_.first = payload.at("taskId").get<std::string>();

    msg.goal_id.id = current_ts_task_.first;

    bool start_at_nearest;
    start_at_nearest = payload.at("startAtNearestPoint").get<bool>();

    std::string map_id;
    map_id = payload.at("mapId").get<std::string>();

    // subtask 1: type 3
    movel_seirios_msgs::Task type_3_task;
    type_3_task.id = 0;
    type_3_task.name = "Goto";
    type_3_task.type = 3;
    type_3_task.mapId = map_id;
    type_3_task.linear_velocity = payload.at("velocity").at("linear").get<double>();
    type_3_task.angular_velocity = payload.at("velocity").at("angular").get<double>();

    json type_3_payload = {
      {"path", {}},
      {"start_at_nearest_point", start_at_nearest}
    };

    for (auto& elem : payload["points"])
    {
      json type_3_path = {
        {"position", {
          {"x", elem.at("position").at("x").get<double>()},
          {"y", elem.at("position").at("y").get<double>()},
          {"z", elem.at("position").at("z").get<double>()}
        }},
        {"orientation", {
          {"x", elem.at("orientation").at("x").get<double>()},
          {"y", elem.at("orientation").at("y").get<double>()},
          {"z", elem.at("orientation").at("z").get<double>()},
          {"w", elem.at("orientation").at("w").get<double>()}
        }},
        {"from_map", map_id},
        {"to_map", map_id},
        {"velocity", {
          {"linear_velocity", payload.at("velocity").at("linear").get<double>()},
          {"angular_velocity", payload.at("velocity").at("angular").get<double>()}
        }}
      };

      type_3_payload["path"].push_back(type_3_path);
    }

    type_3_task.payload = type_3_payload.dump();

    msg.goal.task_list.tasks.push_back(type_3_task);

    return true;
  }
  catch (const json::exception& e)
  {
    ROS_FATAL("[%s] %s.", server_name_.c_str(), std::string(e.what()).c_str());
    err_msg = std::string(e.what());
    return false;
  }
}

bool TaskRouterServer::constructFlexbeTaskMsg(const json& payload,
                                              flexbe_msgs::BehaviorExecutionActionGoal& msg,
                                              std::string& err_msg)
{
  try
  {
    // checking fields
    current_fb_task_.first = payload.at("taskId").get<std::string>();

    msg.goal_id.id = current_fb_task_.first;

    msg.goal.behavior_name = payload.at("behaviorName").get<std::string>();
    msg.goal.arg_keys = payload.at("argKeys").get<std::vector<std::string>>();
    msg.goal.arg_values = payload.at("argValues").get<std::vector<std::string>>();
    msg.goal.input_keys = payload.at("inputKeys").get<std::vector<std::string>>();
    msg.goal.input_values = payload.at("inputValues").get<std::vector<std::string>>();

    return true;
  }
  catch(const json::exception& e)
  {
    ROS_FATAL("[%s] %s.", server_name_.c_str(), std::string(e.what()).c_str());
    err_msg = std::string(e.what());
    return false;
  }
}

void TaskRouterServer::publishPauseStatus()
{
  std_msgs::Bool pause_status;
  pause_status.data = ts_paused_ || fb_paused_;
  pause_status_pub_.publish(pause_status);
}

void TaskRouterServer::resetStates()
{
  if (ts_paused_)
    ts_paused_ = false;
  if (fb_paused_)
    fb_paused_ = false;
  if (cancelled_)
    cancelled_ = false;
}

void TaskRouterServer::updateNavigationState(std::string navigation_state)
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

void TaskRouterServer::tsStatusCb(const actionlib_msgs::GoalStatusArray::ConstPtr& msg)
{
  if (msg->status_list.size() == 0)
    return;

  for (auto& s : msg->status_list)
  {
    if (s.goal_id.id == current_ts_task_.first && getTaskStatus(s.status) == movel_seirios_msgs::TaskFeedback::ACTIVE)
    {
      if (current_ts_task_.second != movel_seirios_msgs::TaskFeedback::ACTIVE)
      {
        current_ts_task_.second = movel_seirios_msgs::TaskFeedback::ACTIVE;
        
        movel_seirios_msgs::TaskFeedback task_feedback_msg;
        task_feedback_msg.task_id = current_ts_task_.first;
        task_feedback_msg.success = false;
        task_feedback_msg.status = current_ts_task_.second;
        task_feedback_msg.message = "Task " + current_ts_task_.first + " is running.";
        task_feedback_pub_.publish(task_feedback_msg);

        resetStates();
        publishPauseStatus();

        ROS_INFO("[%s] Task %s is running.", server_name_.c_str(), current_ts_task_.first.c_str());
      }

      break;
    }
  }
}

void TaskRouterServer::tsResultCb(const movel_seirios_msgs::RunTaskListActionResult::ConstPtr& msg)
{
  if (msg->status.goal_id.id == current_ts_task_.first)
  {
    current_ts_task_.second = getTaskStatus(msg->status.status);

    movel_seirios_msgs::TaskFeedback task_feedback_msg;
    task_feedback_msg.task_id = current_ts_task_.first;
    task_feedback_msg.status = current_ts_task_.second;
    task_feedback_msg.success = msg->result.success;
    task_feedback_msg.message = msg->result.message.data;
    task_feedback_pub_.publish(task_feedback_msg);

    resetStates();
    publishPauseStatus();

    ROS_INFO("[%s] Task %s terminates with status: %d and message: %s.",
      server_name_.c_str(), current_ts_task_.first.c_str(), current_ts_task_.second, msg->result.message.data.c_str());
  }
}

void TaskRouterServer::tsSubtaskFeedbackCb(const movel_seirios_msgs::TaskHandlerFeedback::ConstPtr& msg)
{
  ROS_INFO("[%s] tsSubtaskFeedbackCb.", server_name_.c_str());
  /* 
   * TODO: associate feedback message with task id. This is necessary in order to have a proper task
   * queue functionality. This can be done by changing the semantics of RunTaskListAction feedback message
   * and how the Task Supervisor action server feedback behaves. Currently for backwards compatibility
   * reason we want the changes to Task Supervisor to be minimal so we aren't doing it now.
  */
  std::map<uint8_t, ros::Publisher>::iterator pub_it = subtask_feedback_pubs_.find(msg->task_type);
  if (pub_it == subtask_feedback_pubs_.end())
  {
    std::string topic = "/task_router/subtask_feedback/type_" + std::to_string(msg->task_type);
    subtask_feedback_pubs_[msg->task_type] = nh_.advertise<movel_seirios_msgs::SubtaskFeedback>(topic, 1);
  }

  json msg_payload = {
    {"taskId", current_ts_task_.first},
    {"feedback", json::parse(msg->message)}
  };
  movel_seirios_msgs::SubtaskFeedback subtask_feedback_msg;
  subtask_feedback_msg.message = msg_payload.dump();
  subtask_feedback_pubs_[msg->task_type].publish(subtask_feedback_msg);
}

void TaskRouterServer::fbStatusCb(const actionlib_msgs::GoalStatusArray::ConstPtr& msg)
{
  ROS_INFO("[%s] fbStatusCb.", server_name_.c_str());
  for (auto& s : msg->status_list)
  {
    if (s.goal_id.id == current_fb_task_.first && getTaskStatus(s.status) == movel_seirios_msgs::TaskFeedback::ACTIVE)
    {
      if (current_fb_task_.second != movel_seirios_msgs::TaskFeedback::ACTIVE)
      {
        current_fb_task_.second = movel_seirios_msgs::TaskFeedback::ACTIVE;
        
        movel_seirios_msgs::TaskFeedback task_feedback_msg;
        task_feedback_msg.task_id = current_fb_task_.first;
        task_feedback_msg.success = false;
        task_feedback_msg.status = current_fb_task_.second;
        task_feedback_msg.message = "Task " + current_fb_task_.first + " is running.";
        task_feedback_pub_.publish(task_feedback_msg);

        resetStates();
        publishPauseStatus();

        ROS_INFO("[%s] Task %s is running.", server_name_.c_str(), current_fb_task_.first.c_str());
      }

      break;
    }
  }
}

void TaskRouterServer::fbResultCb(const flexbe_msgs::BehaviorExecutionActionResult::ConstPtr& msg)
{
  ROS_INFO("[%s] fbResultCb.", server_name_.c_str());
  if (msg->status.goal_id.id == current_fb_task_.first)
  {
    current_fb_task_.second = getTaskStatus(msg->status.status);

    movel_seirios_msgs::TaskFeedback task_feedback_msg;
    task_feedback_msg.task_id = current_fb_task_.first;
    task_feedback_msg.status = current_fb_task_.second;
    task_feedback_msg.success = task_feedback_msg.status == movel_seirios_msgs::TaskFeedback::SUCCEEDED ? true : false;
    task_feedback_msg.message = msg->result.outcome;
    task_feedback_pub_.publish(task_feedback_msg);

    resetStates();
    publishPauseStatus();

    ROS_INFO("[%s] Task %s terminates with status: %d and message: %s.",
      server_name_.c_str(), current_fb_task_.first.c_str(), current_fb_task_.second, msg->result.outcome.c_str());
  }
}

uint8_t TaskRouterServer::getTaskStatus(const uint8_t& actionlib_status)
{
  switch (actionlib_status)
  {
    case actionlib_msgs::GoalStatus::ACTIVE:
      return movel_seirios_msgs::TaskFeedback::ACTIVE;
      break;
    case actionlib_msgs::GoalStatus::SUCCEEDED:
      return movel_seirios_msgs::TaskFeedback::SUCCEEDED;
      break;
    case actionlib_msgs::GoalStatus::PREEMPTED:
      return movel_seirios_msgs::TaskFeedback::PREEMPTED;
      break;
    case actionlib_msgs::GoalStatus::REJECTED:
      return movel_seirios_msgs::TaskFeedback::REJECTED;
    default:
      return movel_seirios_msgs::TaskFeedback::ABORTED;
  }
}

} // namespace UniversalHandler

int main(int argc, char** argv)
{
  ros::init(argc, argv, "task_router");
  std::string nodeName(ros::this_node::getName(), ros::this_node::getNamespace().length(), std::string::npos);
  UniversalHandler::TaskRouterServer s(nodeName);

  ros::spin();

  return 0;
}