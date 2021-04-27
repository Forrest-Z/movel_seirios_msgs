#ifndef planner_utils_hpp
#define planner_utils_hpp

#include <ros/ros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <global_planner/planner_core.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>

class PlannerUtils
{
private:
  ros::NodeHandle nh_;
  std::shared_ptr<costmap_2d::Costmap2DROS> clean_costmap_ptr_;
  std::shared_ptr<costmap_2d::Costmap2DROS> sync_costmap_ptr_;
  std::shared_ptr<global_planner::GlobalPlanner> global_planner_ptr_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_ear_;

  double safety_radius_;

public:
  PlannerUtils();
  ~PlannerUtils(){};

  bool makePlanToReachable(geometry_msgs::PoseStamped start, geometry_msgs::PoseStamped goal,
                           std::vector<geometry_msgs::PoseStamped>& plan);
};

#endif