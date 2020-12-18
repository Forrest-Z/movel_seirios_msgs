#ifndef graphmap_planner_h
#define graphmap_planner_h

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
#include <navfn/navfn_ros.h>

#include <graphmap/graphmap.hpp>
#include <movel_license/license_utils.h>

namespace graph_planner 
{

class GraphPlanner : public nav_core::BaseGlobalPlanner
{
public:
  GraphPlanner();
  GraphPlanner(std::string name, costmap_2d::Costmap2DROS *costmap_ros);

  void initialize(std::string name, costmap_2d::Costmap2DROS *costmap_ros);
  bool makePlan(const geometry_msgs::PoseStamped &start,
                const geometry_msgs::PoseStamped &goal,
                std::vector<geometry_msgs::PoseStamped> &plan);

  void initGraph();

  void visGraph();

  void visPath(std::vector<geometry_msgs::PoseStamped> &plan);

  bool checkLOS(geometry_msgs::PoseStamped &src,
                geometry_msgs::PoseStamped &tgt,
                float los_factor=1.5);

  bool addEdgeLOS(Vx src, float los_factor=1.5);

private:
  ros::NodeHandle nh_;
  bool inited_;
  GraphMap gm_;
  ros::Publisher vis_graph_pub_;
  ros::Publisher vis_plan_pub_;
  float goal_threshold_;
  float k_connect_;
  std::vector<geometry_msgs::PoseStamped> active_plan_;
  bool have_active_goal_;
  navfn::NavfnROS navfn_planner_;
};

};

#endif