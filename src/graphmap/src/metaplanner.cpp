#include <pluginlib/class_list_macros.h>
#include <graphmap/metaplanner.hpp>
#include <graphmap/graphmap_planner.hpp>
#include <navfn/navfn_ros.h>

PLUGINLIB_EXPORT_CLASS(metaplanner::MetaPlanner, nav_core::BaseGlobalPlanner)

namespace metaplanner
{
  MetaPlanner::MetaPlanner() : inited_(false), graph_planner_(), navfn_planner_()
  {
  }

  MetaPlanner::MetaPlanner(std::string name, costmap_2d::Costmap2DROS *costmap_ros)
  : inited_(false), graph_planner_("GraphPlanner", costmap_ros)
  , navfn_planner_("GlobalPlaner", costmap_ros)
  {
    initialize(name, costmap_ros);
  }

  void MetaPlanner::initialize(std::string name, costmap_2d::Costmap2DROS *costmap_ros)
  {
    if (!inited_)
    {
      inited_ = true;
      navfn_planner_.initialize("GlobalPlanner", costmap_ros);
      graph_planner_.initialize("GraphPlanner", costmap_ros);
      active_planner_ = graph;
      ROS_INFO("Active planner is %d, graph", active_planner_);

      switch_planner_sub_ = nh_.subscribe("switch_planner", 1,
                                          &MetaPlanner::switchPlannerCb, this);
    }

  }

  bool MetaPlanner::makePlan(const geometry_msgs::PoseStamped &start,
                             const geometry_msgs::PoseStamped &goal,
                             std::vector<geometry_msgs::PoseStamped> &plan)
  {
    bool success;
    if (active_planner_ == graph)
    {
      success = graph_planner_.makePlan(start, goal, plan);
    }
    else
    {
      success = navfn_planner_.makePlan(start, goal, plan);
    }

    return success;
  }

  void MetaPlanner::switchPlannerCb(std_msgs::Empty msg)
  {
    if (active_planner_ == graph)
    {
      active_planner_ = grid;
      ROS_INFO("Active planner is %d, grid", active_planner_);
    }
    else if (active_planner_ == grid)
    {
      active_planner_ = graph;
      ROS_INFO("Active planner is %d, graph", active_planner_);
    }
  }
};
