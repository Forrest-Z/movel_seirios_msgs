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

  speed_store_[0] = { .timestamp = ros::Time::now(), .speed = stop_speed_, .timeout = p_timeout_safety_ };
  speed_store_[1] = { .timestamp = ros::Time::now(), .speed = stop_speed_, .timeout = p_timeout_teleop_ };
  speed_store_[2] = { .timestamp = ros::Time::now(), .speed = stop_speed_, .timeout = p_timeout_teleop_ };
  speed_store_[3] = { .timestamp = ros::Time::now(), .speed = stop_speed_, .timeout = p_timeout_autonomous_ };

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

  return loader.params_valid();
}

void CmdVelMux::setupTopics()
{
  sub1_ = nh_private_.subscribe("autonomous", 1, &CmdVelMux::onSpeedAutonomous, this);
  sub2_ = nh_private_.subscribe("teleop/joystick", 1, &CmdVelMux::onSpeedJoystick, this);
  sub3_ = nh_private_.subscribe("teleop/keyboard", 1, &CmdVelMux::onSpeedKeyboard, this);
  sub4_ = nh_private_.subscribe("safety", 1, &CmdVelMux::onSpeedSafety, this);

  speed_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 10);

  estop_serv_ = nh_.advertiseService("stop_robot", &CmdVelMux::onStopRobot, this);
  is_estop_ = false;
}

bool CmdVelMux::onStopRobot(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res) {
  is_estop_ = req.data;
  res.success = true;
  if(is_estop_){
    res.message = "onStopRobot: true";
  }
  else {
    res.message = "onStopRobot: false";
  }
  return true;
}

void CmdVelMux::onSpeedAutonomous(const geometry_msgs::Twist::ConstPtr& speed)
{
  struct SpeedSourceState new_state = { .timestamp = ros::Time::now(),
                                        .speed = *speed,
                                        .timeout = p_timeout_autonomous_ };
  speed_store_[3] = new_state;
}

void CmdVelMux::onSpeedSafety(const geometry_msgs::Twist::ConstPtr& speed)
{
  struct SpeedSourceState new_state = { .timestamp = ros::Time::now(), .speed = *speed, .timeout = p_timeout_safety_ };
  speed_store_[2] = new_state;
}

void CmdVelMux::onSpeedJoystick(const geometry_msgs::Twist::ConstPtr& speed)
{
  struct SpeedSourceState new_state = { .timestamp = ros::Time::now(), .speed = *speed, .timeout = p_timeout_teleop_ };
  speed_store_[1] = new_state;
}

void CmdVelMux::onSpeedKeyboard(const geometry_msgs::Twist::ConstPtr& speed)
{
  struct SpeedSourceState new_state = { .timestamp = ros::Time::now(), .speed = *speed, .timeout = p_timeout_teleop_ };
  speed_store_[0] = new_state;
}

void CmdVelMux::run(const ros::TimerEvent& e)
{
  if(is_estop_) {
    geometry_msgs::Twist estopped;
    estopped.linear.x = 0;
    estopped.linear.y = 0;
    estopped.linear.z = 0;
    estopped.angular.x = 0;
    estopped.angular.y = 0;
    estopped.angular.z = 0;

    speed_pub_.publish(estopped);
  }
  else {
    geometry_msgs::Twist selected_speed = stop_speed_;
    ros::Time now = ros::Time::now();
    for (int i = 0; i < 4; i++)
    {
      struct SpeedSourceState candidate_source = speed_store_[i];

      if ((now.toSec() - candidate_source.timestamp.toSec()) < candidate_source.timeout)
      {
        selected_speed = candidate_source.speed;
        for (int j = i + 1; j < 4; j++)
          speed_store_[j].timestamp = now - ros::Duration(candidate_source.timeout * 1.2);
        break;
      }
    }
    speed_pub_.publish(selected_speed);
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
