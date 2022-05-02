#ifndef CMD_VEL_MUX_H
#define CMD_VEL_MUX_H

#include <boost/thread/mutex.hpp>

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <std_srvs/SetBool.h>
#include <geometry_msgs/Twist.h>
#include <ros_utils/ros_utils.h>

struct SpeedSourceState
{
  ros::Time timestamp;
  geometry_msgs::Twist speed;
  double timeout;
};

class CmdVelMux
{
private:
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  ros::Publisher speed_pub_;
  ros::Publisher cancel_pub_;

  ros::Subscriber sub1_;
  ros::Subscriber sub2_;
  ros::Subscriber sub3_;
  ros::Subscriber sub4_;

  ros::Timer main_timer_;

  SpeedSourceState speed_store_[4];
  std_msgs::Empty empty_msgs_;
  geometry_msgs::Twist stop_speed_;
  
  double p_loop_rate_;
  double p_timeout_safety_;
  double p_timeout_teleop_;
  double p_timeout_autonomous_;
  bool p_pub_zero_on_idle_;

  // soft-estop stuff
  bool is_estop_;
  ros::ServiceServer estop_serv_;
  bool onStopRobot(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res);

  // pub zeros on idle
  ros::ServiceServer pub_zero_serv_;
  bool togglePubZero(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res);

  void initialize();
  bool loadParams();
  void setupTopics();

  void onSpeedSafety(const geometry_msgs::Twist::ConstPtr& speed);
  void onSpeedAutonomous(const geometry_msgs::Twist::ConstPtr& speed);
  void onSpeedKeyboard(const geometry_msgs::Twist::ConstPtr& speed);
  void onSpeedJoystick(const geometry_msgs::Twist::ConstPtr& speed);

  void run(const ros::TimerEvent& e);

public:
  CmdVelMux();
};

#endif  // CMD_VEL_MUX_H
