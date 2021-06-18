#ifndef map_swapper_h
#define map_swapper_h

#include <geometry_msgs/Pose.h>
#include <movel_seirios_msgs/StringTrigger.h>
#include <nav_msgs/LoadMap.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <yaml-cpp/yaml.h>

class MapSwapper
{
public:
  MapSwapper();
  ~MapSwapper(){}

  // utilities
  void loadParams();
  void setupTopics();

  bool checkInBounds(geometry_msgs::Pose pose, std::string piece_id);

private:
  // parameters
  double rate_;
  std::string robot_frame_;
  std::string map_frame_;

  // bookkeeping
  YAML::Node transitions_;
  bool have_transitions_;
  geometry_msgs::Pose prev_pose_;
  std::string map_dir_;

  // ROS infrastructure
  ros::NodeHandle nh_;
  ros::Publisher transitions_pub_;
  ros::ServiceServer load_map_srv_;
  ros::ServiceClient swap_map_clt_;
  ros::Timer transition_timer_;

  // callbacks
  void transitionTimerCb(const ros::TimerEvent &te);
  bool loadMapSrvCb(movel_seirios_msgs::StringTrigger::Request &req, movel_seirios_msgs::StringTrigger::Response &res);
};

#endif
