#include <pluginlib/class_list_macros.h>
#include "pebble_local_planner/pebble_local_planner.h"

PLUGINLIB_EXPORT_CLASS(pebble_local_planner::PebbleLocalPlanner, nav_core::BaseLocalPlanner)

namespace pebble_local_planner
{
  PebbleLocalPlanner::PebbleLocalPlanner()
  {
    ROS_INFO("pebble constructed");
  }

  bool PebbleLocalPlanner::computeVelocityCommands(geometry_msgs::Twist& cmd_vel)
  {
    ROS_INFO("[%s] calcul velo", name_.c_str());
    cmd_vel.linear.x = 0.;
    cmd_vel.linear.y = 0.;
    cmd_vel.linear.z = 0.;
    cmd_vel.angular.x = 0.;
    cmd_vel.angular.y = 0.;
    cmd_vel.angular.z = 0.;

    return true;
  }


  bool PebbleLocalPlanner::isGoalReached()
  {
    return false;
  }


  bool PebbleLocalPlanner::setPlan(const std::vector<geometry_msgs::PoseStamped>& plan)
  {
    ROS_INFO("[%s] set plan called, size %lu", name_.c_str(), plan.size());
    global_plan_ = plan;
    for (int i = 0; i < plan.size(); i++)
    {
      double th_deg = 180. / M_PI * 2. * acos(plan[i].pose.orientation.w);
      ROS_INFO("wp %d/%lu: (%5.2f, %5.2f, %5.2f)", i, plan.size(), plan[i].pose.position.x, plan[i].pose.position.y, th_deg);
    }
    return true;
  }


  void PebbleLocalPlanner::initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros)
  {
    ROS_INFO("Pebble Local Planner inited with name %s", name.c_str());
    name_ = name;
  }
}