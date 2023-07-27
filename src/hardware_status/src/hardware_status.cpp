#include <hardware_status/hardware_status.h>
#include <movel_hasp_vendor/license.h>

HardwareStatus::HardwareStatus(std::string name)
  : nh_private_("~")
  , name_(name)
{
  initAndRun();
}

HardwareStatus::~HardwareStatus()
{
}

void HardwareStatus::initAndRun()
{
  if (!loadParams())
  {
    ROS_FATAL("[%s] Error during parameter loading. Shutting down.", name_.c_str());
    return;
  }
  ROS_INFO("[%s] All parameters loaded. Launching.", name_.c_str());

  setupRosInterfaces();

  ros::spin();
}

bool HardwareStatus::loadParams()
{
  ros_utils::ParamLoader loader(nh_private_);
  loader.get_required("loop_rate", loop_rate_);
  loader.get_required("reset_timeout", reset_timeout_);
  loader.get_required("odom_topic", odom_topic_);
  
  if (!nh_private_.getParam("lidar_2d_topics", lidar_2d_topics_))
    return false;

  if (!nh_private_.getParam("lidar_3d_topics", lidar_3d_topics_))
    return false;
    
  if (!nh_private_.getParam("camera_topics", camera_topics_))
    return false;
    
  if (!nh_private_.getParam("other_hardware_topics", other_hardware_topics_))
    return false;

  return loader.params_valid();
}

void HardwareStatus::setupRosInterfaces()
{
  odom_state_ = initTopicState(odom_topic_);
  initTopicStateMap(lidar_2d_topics_, lidar_states_);
  initTopicStateMap(lidar_3d_topics_, lidar_states_);
  initTopicStateMap(camera_topics_, camera_states_);
  initTopicStateMap(other_hardware_topics_, other_hardware_states_);

  timer_ = nh_.createTimer(ros::Duration(1.0 / loop_rate_), &HardwareStatus::timerCb, this);
  get_srv_ = nh_private_.advertiseService("get_status", &HardwareStatus::getStatusCb, this);
}

void HardwareStatus::initTopicStateMap(
  std::vector<std::string> topic_names,
  std::map<std::string, TopicState>& topic_states)
{
  for (const auto& topic_name : topic_names)
  {
    topic_states[topic_name] = initTopicState(topic_name);
  }
}

TopicState HardwareStatus::initTopicState(std::string topic_name)
{
  TopicState topic_state;
  topic_state.topic_name = topic_name;
  topic_state.topic_type = "";
  topic_state.last_timestamp = ros::Time::now();
  topic_state.state = State::INACTIVE;
  topic_state.subscriber = nh_.subscribe<topic_tools::ShapeShifter>(
    topic_name, 10, boost::bind(&HardwareStatus::topicCb, this, _1, topic_name));
  
  return topic_state;
}

void HardwareStatus::updateTimeoutStatus(std::map<std::string, TopicState>& states)
{
  for (auto& state : states)
  {
    if (ros::Time::now().toSec() - state.second.last_timestamp.toSec() > reset_timeout_)
    {
      state.second.state = State::INACTIVE;
    }
  }
}

void HardwareStatus::timerCb(const ros::TimerEvent& e)
{
  if (ros::Time::now().toSec() - odom_state_.last_timestamp.toSec() > reset_timeout_)
    odom_state_.state = State::INACTIVE;

  updateTimeoutStatus(lidar_states_);
  updateTimeoutStatus(camera_states_);
  updateTimeoutStatus(other_hardware_states_);
}

std::vector<movel_seirios_msgs::HardwareState> getHardwareStateMsgs(
  std::map<std::string, TopicState> states)
{
  std::vector<movel_seirios_msgs::HardwareState> state_msgs;
  for (const auto& state : states)
  {
    movel_seirios_msgs::HardwareState msg;
    msg.name = state.first;
    msg.state = state.second.state;
    state_msgs.push_back(msg);
  }

  return state_msgs;
}

bool HardwareStatus::getStatusCb(
  movel_seirios_msgs::HardwareStates::Request &req, 
  movel_seirios_msgs::HardwareStates::Response &res)
{
  movel_seirios_msgs::HardwareState motor_state;
  motor_state.name = odom_topic_;
  motor_state.state = odom_state_.state;

  res.base_motor_state = motor_state;
  res.lidar_states = getHardwareStateMsgs(lidar_states_);
  res.camera_states = getHardwareStateMsgs(camera_states_);
  res.other_hardware_states = getHardwareStateMsgs(other_hardware_states_);

  return true;
}

void HardwareStatus::topicCb(
  const topic_tools::ShapeShifter::ConstPtr& msg, 
  const std::string& cb_topic_name)
{ 
  // Determine which topic state to update
  TopicState* state_to_update_ptr = nullptr;
  
  if (odom_topic_ == cb_topic_name)
    state_to_update_ptr = &odom_state_;
  else if (std::find(lidar_2d_topics_.begin(), lidar_2d_topics_.end(), cb_topic_name) != lidar_2d_topics_.end()
    || std::find(lidar_3d_topics_.begin(), lidar_3d_topics_.end(), cb_topic_name) != lidar_3d_topics_.end())
    state_to_update_ptr = &lidar_states_[cb_topic_name];
  else if (std::find(camera_topics_.begin(), camera_topics_.end(), cb_topic_name) != camera_topics_.end())
    state_to_update_ptr = &camera_states_[cb_topic_name];
  else
    state_to_update_ptr = &other_hardware_states_[cb_topic_name];
  
  // Mark this topic as active and update last timestamp
  state_to_update_ptr->state = State::ACTIVE;
  state_to_update_ptr->topic_type = msg->getDataType();
  state_to_update_ptr->last_timestamp = ros::Time::now();
}

int main(int argc, char** argv)
{
  #ifdef MOVEL_LICENSE
  MovelLicense ml;
  if (!ml.login())
    return 1;
  #endif
  
  ros::init(argc, argv, "hardware_status");
  
  std::string node_name(ros::this_node::getName(), ros::this_node::getNamespace().length(), std::string::npos);
  HardwareStatus hardware_status(node_name);

  #ifdef MOVEL_LICENSE
  ml.logout();
  #endif
};
