#include <move_base_state/move_base_state.h>
#include <movel_hasp_vendor/license.h>

MoveBaseState::MoveBaseState()
  : nh_private_("~"), prev_state_(State::NORMAL), current_state_(State::NORMAL), init_(true)
{
  initialize();
}

void MoveBaseState::initialize()
{
  if (!loadParams())
  {
    ROS_FATAL("[move_base_state] Error during parameter loading. Shutting down.");
    return;
  }
  ROS_INFO("[move_base_state] All parameters loaded. Launching.");

  setupTopics();

  ros::spin();
}

// Load ROS params
bool MoveBaseState::loadParams()
{
  ros_utils::ParamLoader loader(nh_private_);
  loader.get_required("loop_rate", loop_rate_);
  loader.get_required("failed_state_timeout", failed_state_timeout_);
  loader.get_required("recovery_state_timeout", recovery_state_timeout_);
  loader.get_required("aborted_state_timeout", aborted_state_timeout_);
  loader.get_required("failed_msg", failed_msg_);
  loader.get_required("recovery_msg", recovery_msg_);
  loader.get_required("aborted_msg", aborted_msg_);
  return loader.params_valid();
}

// Setup callbacks
void MoveBaseState::setupTopics()
{
  log_sub_ = nh_.subscribe("rosout", 10, &MoveBaseState::logCallback, this);
  state_pub_ = nh_private_.advertise<std_msgs::UInt8>("state", 1, true);
  timer_ = nh_.createTimer(ros::Duration(1.0 / loop_rate_), &MoveBaseState::timerCallback, this);
}

// Get log messages from /rosout
void MoveBaseState::logCallback(const rosgraph_msgs::Log::ConstPtr& log)
{
  // Check move_base log messages for relevant keywords
  if (log->name == "/move_base")
  {
    std::size_t found;

    if (log->level == 8)
    {
      found = log->msg.find(failed_msg_);
      if (found != std::string::npos)
      {
        current_state_ = State::FAILED;
        start_ = ros::Time::now();
      }
      else
      {
        found = log->msg.find(aborted_msg_);
        if (found != std::string::npos)
        {
          current_state_ = State::ABORTED;
          start_ = ros::Time::now();
        }
      }
    }
    else if (log->level == 4)
    {
      found = log->msg.find(recovery_msg_);
      if (found != std::string::npos)
      {
        current_state_ = State::RECOVERY;
        start_ = ros::Time::now();
      }
    }
  }
}

// Periodically publish state or reset state if necessary
void MoveBaseState::timerCallback(const ros::TimerEvent& e)
{
  // Initial publish
  if (init_)
  {
    std_msgs::UInt8 state;
    state.data = current_state_;
    state_pub_.publish(state);
    init_ = false;
    return;
  }

  // Reset state after timeout
  switch (current_state_)
  {
    case State::NORMAL:
      break;
    case State::FAILED:
      if (ros::Time::now().toSec() - start_.toSec() > failed_state_timeout_)
        current_state_ = State::NORMAL;
      break;
    case State::RECOVERY:
      if (ros::Time::now().toSec() - start_.toSec() > recovery_state_timeout_)
        current_state_ = State::NORMAL;
      break;
    case State::ABORTED:
      if (ros::Time::now().toSec() - start_.toSec() > aborted_state_timeout_)
        current_state_ = State::NORMAL;
      break;
  }

  // Publish state on state change
  if (current_state_ != prev_state_)
  {
    std_msgs::UInt8 state;
    state.data = current_state_;
    state_pub_.publish(state);
    prev_state_ = current_state_;
  }
}

int main(int argc, char** argv)
{
#ifdef MOVEL_LICENSE
  MovelLicense ml;
  if (!ml.login())
    return 1;
#endif

  ros::init(argc, argv, "move_base_state");
  MoveBaseState state;

#ifdef MOVEL_LICENSE
  ml.logout();
#endif
};
