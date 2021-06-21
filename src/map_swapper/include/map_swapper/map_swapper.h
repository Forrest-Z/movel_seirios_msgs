#ifndef map_swapper_h
#define map_swapper_h

#include <geometry_msgs/Pose.h>
#include <iostream>
#include <movel_seirios_msgs/StringTrigger.h>
#include <nav_msgs/LoadMap.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <yaml-cpp/yaml.h>

struct Pt2D
{
  double x;
  double y;
};

class MapSwapper
{
public:
  MapSwapper();
  ~MapSwapper(){}

  // utilities
  void loadParams();
  void setupTopics();
  bool checkInBounds(geometry_msgs::Pose pose, std::string piece_id);
  void publishTransitions();
  bool findInBoundPiece(geometry_msgs::Pose pose);
  bool loadMapPiece(std::string piece_id);

  double sideCheck(Pt2D pt, Pt2D ln0, Pt2D ln1); // which side of the line is the test point?
  double alongCheck(Pt2D pt, Pt2D ln0, Pt2D ln1); // how far along the line *segment* is the point *projection*?

private:
  // parameters
  double rate_;
  std::string robot_frame_;
  std::string map_frame_;

  // bookkeeping
  YAML::Node transitions_;
  bool have_transitions_;
  bool pose_inited_;
  geometry_msgs::Pose prev_pose_;
  std::string map_dir_;
  std::string piece_id_;

  // ROS infrastructure
  ros::NodeHandle nh_;
  ros::Publisher transitions_pub_;
  ros::ServiceServer load_map_srv_;
  ros::ServiceClient swap_map_clt_;
  ros::Timer transition_timer_;
  tf2_ros::TransformListener tf_ear_;
  tf2_ros::Buffer tf_buffer_;

  // callbacks
  void transitionTimerCb(const ros::TimerEvent &te);
  bool loadMapSrvCb(movel_seirios_msgs::StringTrigger::Request &req, movel_seirios_msgs::StringTrigger::Response &res);
};

#endif
