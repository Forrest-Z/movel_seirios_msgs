#ifndef speed_limit_zones_hpp
#define speed_limit_zones_hpp

#include <ros/ros.h>

class SpeedLimitZones 
{
  public:
    SpeedLimitZones();
    ~SpeedLimitZones(){};
    bool setupTopics();

  private:
    ros::NodeHandle nl;
};

#endif