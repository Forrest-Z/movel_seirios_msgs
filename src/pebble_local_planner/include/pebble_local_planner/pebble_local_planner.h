#ifndef pebble_local_planner_h
#define pebble_local_planner_h

#include <nav_core/base_local_planner.h>
#include <ros/ros.h>

namespace pebble_local_planner
{

class PebbleLocalPlanner : public nav_core::BaseLocalPlanner
{
public:
  PebbleLocalPlanner();
  ~PebbleLocalPlanner(){}

  bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel);
  bool isGoalReached();
  bool setPlan(const std::vector<geometry_msgs::PoseStamped>& plan);
  void initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros);

private:
  std::vector<geometry_msgs::PoseStamped> global_plan_;
  std::string name_;
};

}

#endif