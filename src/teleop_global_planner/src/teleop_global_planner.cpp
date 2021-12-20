#include "teleop_global_planner/teleop_global_planner.h"
#include <pluginlib/class_list_macros.h>
#include <tf/tf.h>
#include <nav_msgs/Path.h>


PLUGINLIB_EXPORT_CLASS(teleop_global_planner::TeleopGlobalPlanner, nav_core::BaseGlobalPlanner)

namespace teleop_global_planner
{

TeleopGlobalPlanner::TeleopGlobalPlanner() : costmap_ros_(NULL), initialized_(false), clear_waypoints_(false)
{
}


TeleopGlobalPlanner::TeleopGlobalPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
{
  initialize(name, costmap_ros);
}


TeleopGlobalPlanner::~TeleopGlobalPlanner()
{
}


void TeleopGlobalPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
{
  if(!initialized_)
  {
    // get the costmap
    costmap_ros_ = costmap_ros;
    costmap_ = costmap_ros_->getCostmap();
    world_model_ = new base_local_planner::CostmapModel(*costmap_);

    ros::NodeHandle nh;
    ros::NodeHandle pnh("~" + name);

    // load parameters
    // pnh.param("epsilon", epsilon_, 1e-1);
    pnh.param("waypoints_per_meter", waypoints_per_meter_, 20);

    // initialize publishers and subscribers
    // waypoint_sub_ = pnh.subscribe("/clicked_point", 100, &TeleopGlobalPlanner::waypointCallback, this);
    external_path_sub_ = pnh.advertiseService("teleop_plan", &TeleopGlobalPlanner::externalPathCallback, this);
    //external_path_sub_ = pnh.subscribe("external_path", 1, &TeleopGlobalPlanner::externalPathCallback, this);
    waypoint_marker_pub_ = pnh.advertise<visualization_msgs::MarkerArray>("waypoints", 1);
    goal_pub_ = nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1);
    plan_pub_ = pnh.advertise<nav_msgs::Path>("global_plan", 1);

    initialized_ = true;
    ROS_INFO("Planner has been initialized");
  }
  else
  {
    ROS_WARN("This planner has already been initialized");
  }
}


bool TeleopGlobalPlanner::makePlan(const geometry_msgs::PoseStamped& start_pose,
  const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan)
{
  // path_.poses.insert(path_.poses.begin(), start_pose);
  // interpolatePath(path_);
  plan_pub_.publish(path_);
  plan = path_.poses;
  ROS_INFO("Published global plan");
  return true;
}




void TeleopGlobalPlanner::interpolatePath(nav_msgs::Path& path)
{
  std::vector<geometry_msgs::PoseStamped> temp_path;
  for (int i = 0; i < static_cast<int>(path.poses.size()-1); i++)
  {
    // calculate distance between two consecutive waypoints
    double x1 = path.poses[i].pose.position.x;
    double y1 = path.poses[i].pose.position.y;
    double x2 = path.poses[i+1].pose.position.x;
    double y2 = path.poses[i+1].pose.position.y;
    double dist =  hypot(x1-x2, y1-y2);
    int num_wpts = dist * waypoints_per_meter_;

    temp_path.push_back(path.poses[i]);
    geometry_msgs::PoseStamped p = path.poses[i];
    for (int j = 0; j < num_wpts - 2; j++)
    {
      p.pose.position.x = x1 + static_cast<double>(j) / num_wpts * (x2 - x1);
      p.pose.position.y = y1 + static_cast<double>(j) / num_wpts * (y2 - y1);
      temp_path.push_back(p);
    }
  }

  // update sequence of poses
  for (size_t i = 0; i < temp_path.size(); i++)
    temp_path[i].header.seq = static_cast<int>(i);

  temp_path.push_back(path.poses.back());
  path.poses = temp_path;
}

bool TeleopGlobalPlanner::externalPathCallback(movel_seirios_msgs::TeleopPlanner::Request &req, movel_seirios_msgs::TeleopPlanner::Response &res )
{
  nav_msgs::Path plan = req.teleop_path;
  path_.poses.clear();
  clear_waypoints_ = true;
  path_.header = plan.header;
  path_.poses = plan.poses;
  ROS_INFO("[Teleop Planner] Got global plan");
  // createAndPublishMarkersFromPath(path_.poses);
  goal_pub_.publish(path_.poses.back());
  res.success = true;
  return true;
}


}  // namespace teleop_global_planner
