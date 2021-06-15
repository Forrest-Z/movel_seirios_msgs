#ifndef map_splitter_h
#define map_splitter_h

#include <ros/ros.h>
#include <movel_seirios_msgs/StringTrigger.h>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <yaml-cpp/yaml.h>

class MapSplitter
{
public:
  MapSplitter();
  ~MapSplitter(){};

  void loadParams();
  void setupTopics();

private:
  // bookkeeping
  ros::NodeHandle nh_;
  ros::ServiceServer split_map_srv_;

  // parameters
  int width_;
  int height_;
  int overlap_;

  // service callbacks
  bool splitMapSrvCb(movel_seirios_msgs::StringTrigger::Request &req,
                     movel_seirios_msgs::StringTrigger::Response &res);
};

#endif