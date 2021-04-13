#include <hardware_status/hardware_status.h>
#include <movel_hasp_vendor/license.h>

HardwareStatus::HardwareStatus()
  : nh_private_("~")
{
  initialize();
}

void HardwareStatus::initialize()
{
  if (!loadParams())
  {
    ROS_FATAL("[hardware_status] Error during parameter loading. Shutting down.");
    return;
  }
  ROS_INFO("[hardware_status] All parameters loaded. Launching.");

  setupTopics();

  ros::spin();
}

// Load ROS params
bool HardwareStatus::loadParams()
{
  ros_utils::ParamLoader loader(nh_private_);
  loader.get_required("loop_rate", loop_rate_);
  loader.get_required("reset_timeout", reset_timeout_);
  loader.get_required("odom_topic", odom_topic_);
  
  if (nh_private_.hasParam("lidar_2d_topics"))
    nh_private_.getParam("lidar_2d_topics", lidar_2d_topics_);
  else
    return false;

  if (nh_private_.hasParam("lidar_3d_topics"))
    nh_private_.getParam("lidar_3d_topics", lidar_3d_topics_);
  else
    return false;
    
  if (nh_private_.hasParam("camera_topics"))
    nh_private_.getParam("camera_topics", camera_topics_);
  else
    return false;
    
  if (nh_private_.hasParam("other_hardware"))
    nh_private_.getParam("other_hardware", other_hardware_);
  else
    return false;

  for (const auto& topic_name : lidar_2d_topics_)
  {
    lidar_states_[topic_name] = State::INACTIVE;
  }

  for (const auto& topic_name : lidar_3d_topics_)
  {
    lidar_states_[topic_name] = State::INACTIVE;
  }

  for (const auto& topic_name : camera_topics_)
  {
    camera_states_[topic_name] = State::INACTIVE;
  }

  for (const auto& sensor : other_hardware_)
  {
    other_hardware_states_[sensor] = State::INACTIVE;
  }

  return loader.params_valid();
}

// Setup callbacks
void HardwareStatus::setupTopics()
{
  odom_sub_ = nh_.subscribe(odom_topic_, 10, &HardwareStatus::odomCallback, this);
  subscribeLidarTopics();
  subscribeCameraTopics();
  timer_ = nh_.createTimer(ros::Duration(1.0 / loop_rate_), &HardwareStatus::timerCallback, this);
  get_srv_ = nh_private_.advertiseService("get_status", &HardwareStatus::getStatus, this);
}

// Subscribe to list of lidar topics
void HardwareStatus::subscribeLidarTopics()
{
  for (const auto& topic_name : lidar_2d_topics_)
  {
    lidar_sub_map_[topic_name] = nh_.subscribe<sensor_msgs::LaserScan> (topic_name.c_str(), 1, boost::bind(&HardwareStatus::scanCallback, this, _1, topic_name));
  }

  for (const auto& topic_name : lidar_3d_topics_)
  {
    lidar_sub_map_[topic_name] = nh_.subscribe<sensor_msgs::PointCloud2> (topic_name.c_str(), 1, boost::bind(&HardwareStatus::cloudCallback, this, _1, topic_name));
  }
}

// Subscribe to list of camera topics
void HardwareStatus::subscribeCameraTopics()
{
  for (const auto& topic_name : camera_topics_)
  {
    camera_sub_map_[topic_name] = nh_.subscribe<sensor_msgs::Image> (topic_name.c_str(), 1, boost::bind(&HardwareStatus::cameraCallback, this, _1, topic_name));
  }
}

// Set active state for motor
void HardwareStatus::odomCallback(const nav_msgs::Odometry::ConstPtr& odom)
{
  motor_state_ = State::ACTIVE;
  motor_state_time_ = odom->header.stamp;
}

// Set active state for 2d lidar(s)
void HardwareStatus::scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan, std::string topic_name)
{
  lidar_states_[topic_name] = State::ACTIVE;
  lidar_states_time_[topic_name] = scan->header.stamp;
}

// Set active state for 3d lidar(s)
void HardwareStatus::cloudCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud, std::string topic_name)
{
  lidar_states_[topic_name] = State::ACTIVE;
  lidar_states_time_[topic_name] = cloud->header.stamp;
}

// Set active state for camera(s)
void HardwareStatus::cameraCallback(const sensor_msgs::Image::ConstPtr& image, std::string topic_name)
{
  camera_states_[topic_name] = State::ACTIVE;
  camera_states_time_[topic_name] = image->header.stamp;
}

// Periodically reset to inactive state
void HardwareStatus::timerCallback(const ros::TimerEvent& e)
{
  if (ros::Time::now().toSec() - motor_state_time_.toSec() > reset_timeout_)
    motor_state_ = State::INACTIVE;

  for (auto const& lidar : lidar_states_)
  {
    if (lidar_states_time_.find(lidar.first) != lidar_states_time_.end())
    {
      if (ros::Time::now().toSec() - lidar_states_time_[lidar.first].toSec() > reset_timeout_)
        lidar_states_[lidar.first] = State::INACTIVE;
    }
  }

  for (auto const& camera : camera_states_)
  {
    if (camera_states_time_.find(camera.first) != camera_states_time_.end())
    {
      if (ros::Time::now().toSec() - camera_states_time_[camera.first].toSec() > reset_timeout_)
        camera_states_[camera.first] = State::INACTIVE;
    }
  }

  if (other_hardware_.size() > 0)
    checkNodes();
}

// Return state of all hardware
bool HardwareStatus::getStatus(movel_seirios_msgs::HardwareStates::Request &req, movel_seirios_msgs::HardwareStates::Response &res)
{
  movel_seirios_msgs::HardwareState motor;
  motor.name = odom_topic_;
  motor.state = motor_state_;
  res.base_motor_state = motor;

  for (const auto& state : lidar_states_)
  {
    movel_seirios_msgs::HardwareState lidar;
    lidar.name = state.first;
    lidar.state = state.second;
    res.lidar_states.push_back(lidar);
  }

  for (const auto& state : camera_states_)
  {
    movel_seirios_msgs::HardwareState camera;
    camera.name = state.first;
    camera.state = state.second;
    res.camera_states.push_back(camera);
  }
  
  for (const auto& state : other_hardware_states_)
  {
    movel_seirios_msgs::HardwareState other;
    other.name = state.first;
    other.state = state.second;
    res.other_hardware_states.push_back(other);
  }
  return true;
}

// Set state of other hardware based on existence of nodes
void HardwareStatus::checkNodes()
{
  std::vector<std::string> nodes;
  if (ros::master::getNodes(nodes))
  {
    for (const auto& sensor : other_hardware_)
    {
      if (std::find(nodes.begin(), nodes.end(), sensor) != nodes.end())
        other_hardware_states_[sensor] = State::ACTIVE;
      else
        other_hardware_states_[sensor] = State::INACTIVE;
    }
  }
}

int main(int argc, char** argv)
{
  #ifdef MOVEL_LICENSE
  MovelLicense ml();
  if (!ml.login())
    return 1;
  #endif

  ros::init(argc, argv, "hardware_status");
  HardwareStatus status;

  #ifdef MOVEL_LICENSE
  ml.logout();
  #endif
};
