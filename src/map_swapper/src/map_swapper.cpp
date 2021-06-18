#include "map_swapper/map_swapper.h"

MapSwapper::MapSwapper() 
: rate_(2), have_transitions_(false)
, robot_frame_("base_link"), map_frame_("map")
{
  loadParams();
  setupTopics();
}

void MapSwapper::loadParams()
{

}

void MapSwapper::setupTopics()
{

}

bool MapSwapper::checkInBounds(geometry_msgs::Pose pose, std::string piece_id)
{
  return false;
}

void MapSwapper::transitionTimerCb(const ros::TimerEvent &te)
{
  // get current pose

  // check if out of bounds

  // if out of bounds, find piece where robot is within bounds

  // check for transition

}

bool MapSwapper::loadMapSrvCb(movel_seirios_msgs::StringTrigger::Request &req, 
                              movel_seirios_msgs::StringTrigger::Response &res)
{

}