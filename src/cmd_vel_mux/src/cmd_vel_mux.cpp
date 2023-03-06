#include <cmd_vel_mux/cmd_vel_mux.h>
#include <movel_hasp_vendor/license.h>

CmdVelMux::CmdVelMux() : nh_private_("~")
{
  initialize();
}

void CmdVelMux::initialize()
{
  ros::Time::waitForValid();
  if (!loadParams())
  {
    ROS_FATAL("Error during parameter loading. Shutting down.");
    return;
  }
  ROS_INFO("All parameters loaded. Launching.");
  setupTopics();

  speed_sources_.push_back(
      { .topic = speed_subs_[0].getTopic(),
        .state = { .timestamp = ros::Time::now(), .speed = stop_speed_, .timeout = p_timeout_safety_ } });
  speed_sources_.push_back(
      { .topic = speed_subs_[1].getTopic(),
        .state = { .timestamp = ros::Time::now(), .speed = stop_speed_, .timeout = p_timeout_teleop_ } });
  speed_sources_.push_back(
      { .topic = speed_subs_[2].getTopic(),
        .state = { .timestamp = ros::Time::now(), .speed = stop_speed_, .timeout = p_timeout_teleop_ } });
  speed_sources_.push_back(
      { .topic = speed_subs_[3].getTopic(),
        .state = { .timestamp = ros::Time::now(), .speed = stop_speed_, .timeout = p_timeout_autonomous_ } });

  main_timer_ = nh_.createTimer(ros::Duration(1.0 / p_loop_rate_), &CmdVelMux::run, this);
  ros::spin();
}
bool CmdVelMux::loadParams()
{
  ros_utils::ParamLoader loader(nh_private_);

  loader.get_required("loop_rate", p_loop_rate_);
  loader.get_required("timeout_safety", p_timeout_safety_);
  loader.get_required("timeout_teleop", p_timeout_teleop_);
  loader.get_required("timeout_autonomous", p_timeout_autonomous_);
  loader.get_required("pub_zero_on_idle", p_pub_zero_on_idle_);

  return loader.params_valid();
}

void CmdVelMux::setupTopics()
{
  speed_subs_.push_back(nh_private_.subscribe("autonomous", 1, &CmdVelMux::onSpeedAutonomous, this));
  speed_subs_.push_back(nh_private_.subscribe("teleop/joystick", 1, &CmdVelMux::onSpeedJoystick, this));
  speed_subs_.push_back(nh_private_.subscribe("teleop/keyboard", 1, &CmdVelMux::onSpeedKeyboard, this));
  speed_subs_.push_back(nh_private_.subscribe("safety", 1, &CmdVelMux::onSpeedSafety, this));

  speed_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 10);

  // topics for soft estop
  estop_serv_ = nh_.advertiseService("stop_robot", &CmdVelMux::onStopRobot, this);
  is_estop_ = false;
  // pub zeros on idle
  pub_zero_serv_ = nh_private_.advertiseService("toggle_pub_zero_on_idle", &CmdVelMux::togglePubZero, this);

  // add/remove cmd vel inputs
  add_cmd_vel_input_srv_ = nh_private_.advertiseService("add_cmd_vel_input", &CmdVelMux::onAddCmdVelInput, this);
  remove_cmd_vel_input_srv_ =
      nh_private_.advertiseService("remove_cmd_vel_input", &CmdVelMux::onRemoveCmdVelInput, this);
}

bool CmdVelMux::onStopRobot(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res)
{
  is_estop_ = req.data;
  res.success = true;
  if (is_estop_)
  {
    res.message = "onStopRobot: true";
  }
  else
  {
    res.message = "onStopRobot: false";
  }
  return true;
}

bool CmdVelMux::togglePubZero(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res)
{
  p_pub_zero_on_idle_ = req.data;
  res.success = true;
  if (p_pub_zero_on_idle_)
    res.message = "togglePubZeroOnIdle: true";
  else
    res.message = "togglePubZeroOnIdle: false";
  return true;
}

bool CmdVelMux::onAddCmdVelInput(movel_seirios_msgs::CmdVelInput::Request& req,
                                 movel_seirios_msgs::CmdVelInput::Response& res)
{
  for (int i = 0; i < speed_sources_.size(); ++i)
  {
    if (req.topic == speed_sources_[i].topic)
    {
      res.success = false;
      res.message = "Topic " + req.topic + " is already registered as a cmd vel input.";
      return true;
    }
  }

  std::string full_topic;
  if (req.topic[0] == '/')
    full_topic = req.topic;  // topic is absolute path
  else
    full_topic = nh_private_.getNamespace() + "/" + req.topic;  // topic is relative, append nh namespace

  auto callback = [&](const geometry_msgs::Twist::ConstPtr& speed) -> void {
    // repeating topic-building logic inside lambda because I can't capture full_topic variable from outside properly
    // TODO: fix if possible
    std::string _topic;
    if (req.topic[0] == '/')
      _topic = req.topic;
    else
      _topic = nh_private_.getNamespace() + "/" + req.topic;

    for (int i = 0; i < speed_sources_.size(); ++i)
    {
      if (_topic == speed_sources_[i].topic)
      {
        struct SpeedSourceState new_state = { .timestamp = ros::Time::now(), .speed = *speed, .timeout = req.timeout };
        speed_sources_[i].state = new_state;
        return;
      }
    }
  };
  ros::Subscriber speed_sub = nh_private_.subscribe<geometry_msgs::Twist>(req.topic, 1, callback);
  speed_subs_.push_back(speed_sub);
  speed_sources_.push_back(
      { .topic = full_topic,
        .state = { .timestamp = ros::Time::now(), .speed = stop_speed_, .timeout = req.timeout } });

  res.success = true;
  res.message = "Topic " + full_topic + " is successfully registered as a cmd vel input.";
  return true;
}

bool CmdVelMux::onRemoveCmdVelInput(movel_seirios_msgs::CmdVelInput::Request& req,
                                    movel_seirios_msgs::CmdVelInput::Response& res)
{
  for (int i = 0; i < speed_sources_.size(); ++i)
  {
    if (req.topic == speed_sources_[i].topic)
    {
      if (i > 3)
      {
        speed_sources_.erase(speed_sources_.begin() + i);
        speed_subs_.erase(speed_subs_.begin() + i);

        res.success = true;
        res.message = "Topic " + req.topic + " has been removed from cmd vel input.";
        return true;
      }
      else
      {
        // index 0 to 3 (safety, keyboard, teleop, autonomous) may not be removed
        res.success = false;
        res.message = "Topic " + req.topic + " is not allowed to be removed from cmd_vel_input.";
        return true;
      }
    }
  }

  res.success = false;
  res.message = "Topic " + req.topic + " is not registered as a cmd vel input.";
  return true;
}

void CmdVelMux::onSpeedAutonomous(const geometry_msgs::Twist::ConstPtr& speed)
{
  struct SpeedSourceState new_state = { .timestamp = ros::Time::now(),
                                        .speed = *speed,
                                        .timeout = p_timeout_autonomous_ };
  speed_sources_[3].state = new_state;
}

void CmdVelMux::onSpeedJoystick(const geometry_msgs::Twist::ConstPtr& speed)
{
  struct SpeedSourceState new_state = { .timestamp = ros::Time::now(), .speed = *speed, .timeout = p_timeout_teleop_ };
  speed_sources_[2].state = new_state;
}

void CmdVelMux::onSpeedKeyboard(const geometry_msgs::Twist::ConstPtr& speed)
{
  struct SpeedSourceState new_state = { .timestamp = ros::Time::now(), .speed = *speed, .timeout = p_timeout_teleop_ };
  speed_sources_[1].state = new_state;
}

void CmdVelMux::onSpeedSafety(const geometry_msgs::Twist::ConstPtr& speed)
{
  struct SpeedSourceState new_state = { .timestamp = ros::Time::now(), .speed = *speed, .timeout = p_timeout_safety_ };
  speed_sources_[0].state = new_state;
}

void CmdVelMux::run(const ros::TimerEvent& e)
{
  if (is_estop_)
  {
    geometry_msgs::Twist estopped;
    estopped.linear.x = 0;
    estopped.linear.y = 0;
    estopped.linear.z = 0;
    estopped.angular.x = 0;
    estopped.angular.y = 0;
    estopped.angular.z = 0;

    speed_pub_.publish(estopped);
  }
  else
  {
    geometry_msgs::Twist selected_speed = stop_speed_;
    ros::Time now = ros::Time::now();
    for (int i = 0; i < speed_sources_.size(); i++)
    {
      struct SpeedSourceState candidate_source = speed_sources_[i].state;

      if ((now.toSec() - candidate_source.timestamp.toSec()) < candidate_source.timeout)
      {
        selected_speed = candidate_source.speed;
        for (int j = i + 1; j < speed_sources_.size(); j++)
          speed_sources_[j].state.timestamp = now - ros::Duration(candidate_source.timeout * 1.2);
        speed_pub_.publish(selected_speed);
        return;
      }
    }
    if (p_pub_zero_on_idle_)
      speed_pub_.publish(stop_speed_);
  }
}

int main(int argc, char** argv)
{
#ifdef MOVEL_LICENSE
  MovelLicense ml;
  if (!ml.login())
    return 1;
#endif

  ros::init(argc, argv, "cmd_vel_mux");
  CmdVelMux scm;

#ifdef MOVEL_LICENSE
  ml.logout();
#endif
}
