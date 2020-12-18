#ifndef metaplanner_h
#define metaplanner_h

#include <ros/ros.h>
#include <nav_core/base_global_planner.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PoseArray.h>
#include <angles/angles.h>

#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <base_local_planner/world_model.h>
#include <base_local_planner/costmap_model.h>

#include <graphmap/graphmap_planner.hpp>
#include <navfn/navfn_ros.h>

namespace metaplanner
{

enum t_planner {graph, grid};

class MetaPlanner : public nav_core::BaseGlobalPlanner
{
public:
  MetaPlanner();
  MetaPlanner(std::string name, costmap_2d::Costmap2DROS *costmap_ros);

  void initialize(std::string name, costmap_2d::Costmap2DROS *costmap_ros);

  bool makePlan(const geometry_msgs::PoseStamped &start,
                const geometry_msgs::PoseStamped &goal,
                std::vector<geometry_msgs::PoseStamped> &plan);

private:
  ros::NodeHandle nh_;
  ros::Subscriber switch_planner_sub_;
  bool inited_;
  graph_planner::GraphPlanner graph_planner_;
  navfn::NavfnROS navfn_planner_;
  t_planner active_planner_;

  void switchPlannerCb(std_msgs::Empty msg);
};

}; // namespace metaplanner

#endif
