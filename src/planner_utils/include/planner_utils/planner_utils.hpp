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
  std::string name_{"planner_utils"};
  ros::NodeHandle nh_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_ear_;
  std::shared_ptr<costmap_2d::Costmap2DROS> clean_costmap_ptr_;
  std::shared_ptr<costmap_2d::Costmap2DROS> sync_costmap_ptr_;
  std::shared_ptr<global_planner::GlobalPlanner> global_planner_ptr_;
  double safety_radius_;

public:
  double reachable_plan_stop_distance_;

  PlannerUtils();
  ~PlannerUtils(){};

  bool makeCleanPlan(geometry_msgs::PoseStamped start, 
                     geometry_msgs::PoseStamped goal,
                     std::vector<geometry_msgs::PoseStamped>& plan);

  bool calcReachableSubplan(const std::vector<geometry_msgs::PoseStamped>& plan, 
                            const int start_from_idx, 
                            int& reachable_idx, int& blocked_idx);

  bool makePlanToReachable(const geometry_msgs::PoseStamped& start, 
                           const geometry_msgs::PoseStamped& goal,
                           std::vector<geometry_msgs::PoseStamped>& plan);

};

#endif