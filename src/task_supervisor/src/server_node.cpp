#include <task_supervisor/server_node.h>
#include <ros_utils/ros_utils.h>
#include <movel_hasp_vendor/license.h>

#include <future>

TaskSupervisorNode::TaskSupervisorNode(std::string name)
  : nh_private_("~")
  , as_(nh_, name, boost::bind(&TaskSupervisorNode::executeCB, this, _1), false)
  , server_name_(name)
  , paused_(false)
{
  ros::Time::waitForValid();
  if (!loadParams())
  {
    ROS_FATAL("[%s] Error during parameter loading. Shutting down.", server_name_.c_str());
    return;
  }

  task_supervisor_ptr_ = std::make_shared<task_supervisor::TaskSupervisor>(name, task_supervisor_params_);

  ROS_INFO("[%s] All parameters loaded. Launching.", server_name_.c_str());

  if (task_supervisor_ptr_->loadPlugins())
  {
      ROS_FATAL("[%s] Error during plugin loading. Shutting down.", server_name_.c_str());
      return;
  }

  ROS_INFO("[%s] Starting tasks server", server_name_.c_str());
  pub_heartbeat_ = nh_.advertise<std_msgs::Bool>("task_supervisor/heartbeat", 10);
  pause_pub_ = nh_.advertise<std_msgs::Bool>("task_supervisor/pause_status", 1, true);
  heartbeat_timer_ =
      nh_.createTimer(ros::Duration(1.0 / task_supervisor_params_.p_loop_rate), &TaskSupervisorNode::HeartBeat, this);
  get_task_type_service_server_ =
      nh_private_.advertiseService("get_task_type", &TaskSupervisorNode::getTaskTypeServiceCb, this);
  pause_sub_ = nh_private_.subscribe("pause", 1, &TaskSupervisorNode::pauseCb, this);
  as_.start();

  std_msgs::Bool pause;
  pause.data = paused_;
  pause_pub_.publish(pause);
}

TaskSupervisorNode::~TaskSupervisorNode()
{
}

bool TaskSupervisorNode::loadParams()
{
  ros_utils::ParamLoader loader(nh_private_);

  // Load parameters
  loader.get_required("loop_rate", task_supervisor_params_.p_loop_rate);
  loader.get_required("transition_timeout", task_supervisor_params_.p_transition_timeout);
  loader.get_required("default_linear_velocity", task_supervisor_params_.p_default_linear_velocity);
  loader.get_required("default_angular_velocity", task_supervisor_params_.p_default_angular_velocity);

  loader.get_optional("plugins", task_supervisor_params_.plugin_list, XmlRpc::XmlRpcValue());

  // Give server node object to task supervisor object
  task_supervisor_params_.server_nh = nh_private_;

  return loader.params_valid();
}

void TaskSupervisorNode::executeCB(const movel_seirios_msgs::RunTaskListGoalConstPtr& goal)
{
  if (paused_)
  {
    paused_ = false;
    std_msgs::Bool pause;
    pause.data = paused_;
    pause_pub_.publish(pause);
  }
  action_id_ = goal->task_list.id;

  // Forward goal list to task_supervisor asynchronously
  std::future<void> task_list_status = std::async(std::launch::async, &task_supervisor::TaskSupervisor::startTaskList,
                                                  task_supervisor_ptr_, std::ref(goal));
  while (!task_supervisor_ptr_->isActive())
    ;

  ros::Rate r(task_supervisor_params_.p_loop_rate);

  // Wait for task_supervisor to complete task lists and process callbacks
  while (ros::ok())
  {
    // Publish feedback if a task has been completed
    if (task_supervisor_ptr_->getFeedback(feedback_))
      as_.publishFeedback(feedback_);

    if (isActionCancelled())
    {
      ROS_INFO("[%s] Cancel received from user", server_name_.c_str());

      // Send cancel to task supervisor and then block until cancelled
      task_supervisor_ptr_->cancelAllTask();
      ROS_INFO("[%s] Waiting for supervisor to kill handlers", server_name_.c_str());
      while (!(task_supervisor_ptr_->isStopped() || task_supervisor_ptr_->isPreempted()))
        ;

      cancelAction("Preempted by client");
      return;
    }

    // Check if task supervisor is done with task
    else if (task_supervisor_ptr_->isDone())
    {
      completeAction();
      return;
    }

    // Check if task supervisor stopped on error
    else if (task_supervisor_ptr_->isStopped() || task_supervisor_ptr_->isPreempted())
    {
      failAction(task_supervisor_ptr_->getMsg());
      return;
    }

    r.sleep();
  }
}

bool TaskSupervisorNode::isActionCancelled()
{
  return as_.isPreemptRequested() || !ros::ok();
}

void TaskSupervisorNode::cancelAction(std::string cancellation_message)
{
  ROS_WARN_STREAM(server_name_ << ": " << cancellation_message);

  result_.success = false;
  result_.id = action_id_;
  std_msgs::String cancellation_message_ros;
  cancellation_message_ros.data = cancellation_message;
  result_.message = cancellation_message_ros;
  as_.setPreempted(result_);
  if (paused_)
  {
    paused_ = false;
    std_msgs::Bool pause;
    pause.data = paused_;
    pause_pub_.publish(pause);
  }
}

void TaskSupervisorNode::completeAction()
{
  ROS_INFO("[%s] Succeeded", server_name_.c_str());

  result_.success = true;
  result_.id = action_id_;
  std_msgs::String success_msg;
  success_msg.data = task_supervisor_ptr_->getMsg();
  result_.message = success_msg;
  as_.setSucceeded(result_);
  if (paused_)
  {
    paused_ = false;
    std_msgs::Bool pause;
    pause.data = paused_;
    pause_pub_.publish(pause);
  }
}

void TaskSupervisorNode::failAction(std::string failure_message)
{
  ROS_ERROR_STREAM(server_name_ << ": " << failure_message);
  ROS_WARN("[%s] Failed", server_name_.c_str());

  result_.success = false;
  result_.id = action_id_;
  std_msgs::String failure_message_ros;
  failure_message_ros.data = failure_message;
  result_.message = failure_message_ros;
  as_.setAborted(result_);
  if (paused_)
  {
    paused_ = false;
    std_msgs::Bool pause;
    pause.data = paused_;
    pause_pub_.publish(pause);
  }
}

void TaskSupervisorNode::HeartBeat(const ros::TimerEvent& e)
{
  std_msgs::Bool hb;
  hb.data = false;
  if (ros::ok())
  {
    hb.data = true;
  }
  pub_heartbeat_.publish(hb);
}

void TaskSupervisorNode::pauseCb(const std_msgs::Bool::ConstPtr& msg)
{
  if (task_supervisor_ptr_->isActive())
  {
    if (msg->data && !paused_)
    {
      ROS_INFO("[%s] Pausing current task", server_name_.c_str());
      paused_ = true;
      task_supervisor_ptr_->pauseTask();
      std_msgs::Bool pause;
      pause.data = paused_;
      pause_pub_.publish(pause);
    }
    else if (!msg->data && paused_)
    {
      ROS_INFO("[%s] Resuming current task", server_name_.c_str());
      paused_ = false;
      task_supervisor_ptr_->resumeTask();
      std_msgs::Bool pause;
      pause.data = paused_;
      pause_pub_.publish(pause);
    }
  }
  else
    ROS_WARN("[%s] No active task to pause/resume", server_name_.c_str());
}

int main(int argc, char** argv)
{
#ifdef MOVEL_LICENSE
  MovelLicense ml(2);
  if (!ml.login())
    return 1;
#endif

  ros::init(argc, argv, "task_supervisor");

  // Set server name to be same as node name, remove "namespace/"
  std::string nodeName(ros::this_node::getName(), ros::this_node::getNamespace().length(), std::string::npos);
  TaskSupervisorNode s(nodeName);

  ros::spin();

#ifdef MOVEL_LICENSE
  ml.logout();
#endif

  return 0;
}
