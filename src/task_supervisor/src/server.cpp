#include <task_supervisor/server.h>
#include <ros_utils/ros_utils.h>
#include <future>

namespace task_supervisor
{
TaskSupervisor::TaskSupervisor(std::string name, Params param_list)
  : server_name_(name), node_name_(name), stopped_(true), supervisor_state_(STOPPED)
  , active_task_type_(0)
{
  plugin_list_ = param_list.plugin_list;
  p_loop_rate_ = param_list.p_loop_rate;
  p_transition_timeout_ = param_list.p_transition_timeout;
  server_nh_ = param_list.server_nh;
  p_default_linear_velocity_ = param_list.p_default_linear_velocity;
  p_default_angular_velocity_ = param_list.p_default_angular_velocity;
}

TaskSupervisor::~TaskSupervisor()
{
  handler_ptr_.reset();
  plugins_.clear();
  plugin_loader_ptr_.reset();
}

LOADER_RETURN TaskSupervisor::loadPlugins()
{
  plugin_loader_ptr_ = std::make_shared<pluginlib::ClassLoader<task_supervisor::TaskHandler>>(
      node_name_, "task_supervisor::TaskHandler");

  // Check that plugin list is array type
  if (plugin_list_.getType() == XmlRpc::XmlRpcValue::TypeArray)
  {
    for (int i = 0; i < plugin_list_.size(); ++i)
    {
      std::string name = static_cast<std::string>(plugin_list_[i]["name"]);
      uint8_t task_type = static_cast<int>(plugin_list_[i]["type"]);
      std::string class_type = static_cast<std::string>(plugin_list_[i]["class"]);
      ROS_INFO("[%s] Loading plugin %s from class %s with task type %i", server_name_.c_str(), name.c_str(),
               class_type.c_str(), task_type);

      try
      {
        boost::shared_ptr<task_supervisor::TaskHandler> plugin_ptr = plugin_loader_ptr_->createInstance(class_type);
        if (!plugin_ptr->initialize(server_nh_, name))
        {
          ROS_ERROR("[%s] Failed to initialize plugin", server_name_.c_str());
          return INITIALIZE_FAIL;
        }

        if (getPlugin(task_type) != NULL)
        {
          ROS_ERROR("[%s] Another plugin with this task type has already been loaded.", server_name_.c_str());
          return ALREADY_EXISTS;
        }

        plugins_.emplace(task_type, plugin_ptr);
      }
      catch (pluginlib::PluginlibException& ex)
      {
        ROS_ERROR("[%s] The plugin failed to load: %s", server_name_.c_str(), ex.what());
        return LOAD_FAIL;
      }
    }
  }
  return LOAD_OK;
}

boost::shared_ptr<task_supervisor::TaskHandler> TaskSupervisor::getPlugin(uint16_t task_type)
{
  std::map<uint16_t, boost::shared_ptr<task_supervisor::TaskHandler>>::iterator it = plugins_.find(task_type);
  if (it != plugins_.end())
    return it->second;
  else
    return NULL;
}

void TaskSupervisor::startTaskList(const movel_seirios_msgs::RunTaskListGoalConstPtr& goal)
{
  supervisor_state_ = RUNNING;

  movel_seirios_msgs::TaskList task_list = goal->task_list;

  for (movel_seirios_msgs::Task& task : task_list.tasks)
  {
    ROS_INFO("[%s] Starting task %d of type %i", server_name_.c_str(), task.id, task.type);

    // Check if error occurred or cancel is called by server
    if (runTask(task) || supervisor_state_ == STOPPING)
    {
      supervisor_state_ = STOPPED;  // Set supervisor stopped as error occurred or cancelled
      return;
    }

    // One task completed, return feedback to server_node
    // Feedback might be missed if server node processes feedback slower than completion of tasks
    feedback_.completed_task = task;
    feedback_updated_ = true;
  }

  supervisor_state_ = COMPLETED;
}

RUN_TASK_RETURN TaskSupervisor::runTask(movel_seirios_msgs::Task task)
{
  ros::Time last_time_update = ros::Time::now();
  bool proceed = false;

  // Wait (p_transition_timeout_) amount of time for vehicle to stop before exiting as fail
  while ((ros::Time::now() - last_time_update).toSec() < p_transition_timeout_)
  {
    if (stopped_)
    {
      proceed = true;
      break;
    }
  }

  if (!proceed)
  {
    error_code_ = ReturnCode::FAILED;
    error_msg_ = "Task " + std::to_string(task.id) + " of type " + std::to_string(task.type) +
                 " will not proceed until AGV is still. Time waited: " +
                 std::to_string((ros::Time::now() - last_time_update).toSec());
    ROS_WARN("%s", error_msg_.c_str());
    return VEHICLE_NOT_STOPPED;
  }

  // step one: get the corresponding task handler
  handler_ptr_ = getPlugin(task.type);
  if (handler_ptr_ == NULL)
  {
    error_code_ = ReturnCode::FAILED;
    error_msg_ = "Task " + std::to_string(task.id) + " of type " + std::to_string(task.type) + " does not correspond "
                                                                                               "to any known tasks";
    ROS_WARN("%s", error_msg_.c_str());
    return NO_PLUGIN_FOUND;
  }
  active_task_type_ = task.type;

  bool velocity_changed = setVelocities(task.linear_velocity, task.angular_velocity);

  // start task asynchronously
  std::future<ReturnCode> task_status =
      std::async(std::launch::async, &TaskHandler::runTask, handler_ptr_, std::ref(task), std::ref(error_msg_));

  while (!handler_ptr_->isTaskActive())
    ;

  // Loop to wait for results and check for cancellation
  ros::Rate r(p_loop_rate_);
  while (ros::ok())
  {
    if (supervisor_state_ == STOPPING)  // Check for cancellation from outside
    {
      handler_ptr_->cancelTask();
      while (handler_ptr_->isTaskActive())
        ;  // Block until task handler has stopped
      ROS_WARN("[%s] Task %d cancelled", server_name_.c_str(), task.id);
      active_task_type_ = 0;
      if (velocity_changed)
        revertVelocities();
      return RUN_STOPPED;
    }

    // Check if task is done and that future is valid
    if (handler_ptr_->isTaskParsed() && task_status.valid())
    {
      // Get return from runTask()
      error_code_ = task_status.get();
      if (error_code_ != ReturnCode::SUCCESS)
      {
        ROS_ERROR("[%s] Task %d error", server_name_.c_str(), task.id);
        active_task_type_ = 0;
        if (velocity_changed)
          revertVelocities();
        return RUN_ERROR;
      }
      // Task complete
      else
        break;
    }

    r.sleep();
  }

  if (velocity_changed)
    revertVelocities();
  error_code_ = handler_ptr_->getTaskResult(error_msg_);
  if (error_code_ != ReturnCode::SUCCESS)
  {
    ROS_INFO("[%s] Task %d result error", server_name_.c_str(), task.id);
    active_task_type_ = 0;
    return RUN_RESULT_ERROR;
  }

  ROS_INFO("[%s] Task %d complete", server_name_.c_str(), task.id);
  active_task_type_ = 0;
  return RUN_OK;
}

void TaskSupervisor::cancelSingleTask()
{
  // Catch exceptions? Will handler_ptr_ be invalid?
  handler_ptr_->cancelTask();
}

void TaskSupervisor::cancelAllTask()
{
  supervisor_state_ = STOPPING;
}

bool TaskSupervisor::isActive()
{
  return supervisor_state_ == RUNNING;
}

bool TaskSupervisor::isPreempted()
{
  return supervisor_state_ == STOPPED;
}

bool TaskSupervisor::isStopped()
{
  return supervisor_state_ == STOPPED;
}

bool TaskSupervisor::isDone()
{
  return supervisor_state_ == COMPLETED;
}

bool TaskSupervisor::getFeedback(movel_seirios_msgs::RunTaskListFeedback& feedback)
{
  if (feedback_updated_)
  {
    feedback = feedback_;
    feedback_updated_ = false;
    return true;
  }

  // No update
  else
    return false;
}

ReturnCode TaskSupervisor::getErrorCode()
{
  task_supervisor::ReturnCode code_ = error_code_;
  error_code_ = ReturnCode::SUCCESS;
  return code_;
}

std::string TaskSupervisor::getMsg()
{
  std::string msg_ = error_msg_;
  if (error_msg_.empty())
    return "No error message found";
  else
  {
    //error_msg_ = "";
    return msg_;
  }
}

std::string TaskSupervisor::errorMessage(RUN_TASK_RETURN error)
{
  switch (error)
  {
    case VEHICLE_NOT_STOPPED:
      return "Task will not proceed until vehicle is stopped. Waited past threshold time";

    case NO_PLUGIN_FOUND:
      return "Task does not correspond to any imported plugins from param file";

    case RUN_ERROR:
      return "Error occurred while running latest task";

    case RUN_RESULT_ERROR:
      return "Error returned when retrieving the result of latest task";

    case RUN_STOPPED:
      return "Task was stopped";

    default:
      return "Error code does not correspond to any message";
  }
}

std::string TaskSupervisor::errorMessage(EXPAND_TASK_RETURN error)
{
  switch (error)
  {
    default:
      return "Error code does not correspond to any message";
  }
}

std::string TaskSupervisor::errorMessage(LOADER_RETURN error)
{
  switch (error)
  {
    case INITIALIZE_FAIL:
      return "Failed to initialize plugin";

    case ALREADY_EXISTS:
      return "Plugin already exists and is loaded";

    case LOAD_FAIL:
      return "Failed to load plugin";

    default:
      return "Error code does not correspond to any message";
  }
}

uint16_t TaskSupervisor::getActiveTaskType()
{
  return active_task_type_;
}

bool TaskSupervisor::setVelocities(double linear_velocity, double angular_velocity)
{
  // Set velocity with velocity_setter service if received non-zero velocity values
  if (linear_velocity != 0 || angular_velocity != 0)
  {
    ROS_INFO("[%s] Received velocity value(s) to be set", server_name_.c_str());
    ros::ServiceClient velocity_client =
        server_nh_.serviceClient<movel_seirios_msgs::SetSpeed>("/velocity_setter_node/set_speed");
    movel_seirios_msgs::SetSpeed set_speed;

    if (linear_velocity != 0)
      set_speed.request.linear = linear_velocity;
    else
      set_speed.request.linear = p_default_linear_velocity_;

    if (angular_velocity != 0)
      set_speed.request.angular = angular_velocity;
    else
      set_speed.request.angular = p_default_angular_velocity_;

    if (!velocity_client.call(set_speed))
      ROS_ERROR("[%s] Failed to call /velocity_setter_node/set_speed service", server_name_.c_str());
    else if (set_speed.response.success)
      ROS_INFO("[%s] Linear velocity set to: %f, angular velocity set to: %f", server_name_.c_str(),
               set_speed.request.linear, set_speed.request.angular);
    else if (!set_speed.response.success)
      ROS_ERROR("[%s] Failed to set new linear and angular velocities", server_name_.c_str());
    return true;
  }
  else
    return false;
}

void TaskSupervisor::revertVelocities()
{
  // Revert to default velocities
  ROS_INFO("[%s] Reverting to default velocities", server_name_.c_str());
  ros::ServiceClient velocity_client =
      server_nh_.serviceClient<movel_seirios_msgs::SetSpeed>("/velocity_setter_node/set_speed");
  movel_seirios_msgs::SetSpeed set_speed;

  set_speed.request.linear = p_default_linear_velocity_;
  set_speed.request.angular = p_default_angular_velocity_;

  if (!velocity_client.call(set_speed))
    ROS_ERROR("[%s] Failed to call /velocity_setter_node/set_speed service", server_name_.c_str());
  else if (set_speed.response.success)
    ROS_INFO("[%s] Reverted to default velocities", server_name_.c_str());
  else if (!set_speed.response.success)
    ROS_ERROR("[%s] Failed to revert to default velocities", server_name_.c_str());
}

void TaskSupervisor::pauseTask()
{
  handler_ptr_->pauseTask();
}

void TaskSupervisor::resumeTask()
{
  handler_ptr_->resumeTask();
}
}  // namespace task_supervisor
