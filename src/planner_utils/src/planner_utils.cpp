#include "planner_utils/planner_utils.hpp"


double calcDistance(geometry_msgs::Pose a, geometry_msgs::Pose b)
{
  double dx, dy;
  dx = a.position.x - b.position.x;
  dy = a.position.y - b.position.y;
  return sqrt(dx*dx + dy*dy);
}


PlannerUtils::PlannerUtils()
: tf_ear_(tf_buffer_), safety_radius_(1.0), reachable_plan_stop_distance_(1.0)
{
  clean_costmap_ptr_ = std::make_shared<costmap_2d::Costmap2DROS>("aux_clean_map", tf_buffer_);
  sync_costmap_ptr_ = std::make_shared<costmap_2d::Costmap2DROS>("aux_sync_map", tf_buffer_);
  global_planner_ptr_ = std::make_shared<global_planner::GlobalPlanner>();

  global_planner_ptr_->initialize("aux_planner", clean_costmap_ptr_.get());

  std::vector<geometry_msgs::Point> footprint = clean_costmap_ptr_->getRobotFootprint();
  double max_x = 0.0;
  double max_y = 0.0;
  for (int i = 0; i < footprint.size(); i++)
  {
    if (fabs(footprint[i].x) > max_x)
      max_x = fabs(footprint[i].x);

    if (fabs(footprint[i].y) > max_y)
      max_y = fabs(footprint[i].y);

    safety_radius_ = sqrt(max_x*max_x + max_y*max_y);
  }
}


bool PlannerUtils::makeCleanPlan(geometry_msgs::PoseStamped start, 
                                 geometry_msgs::PoseStamped goal,
                                 std::vector<geometry_msgs::PoseStamped>& plan)
{
  // Verify frames are transformable
  ROS_INFO("[%s] frames start %s, goal %s, costmap %s", 
    name_.c_str(), start.header.frame_id.c_str(), 
    goal.header.frame_id.c_str(), clean_costmap_ptr_->getGlobalFrameID().c_str()
  );
  if (start.header.frame_id != clean_costmap_ptr_->getGlobalFrameID() || 
      goal.header.frame_id != clean_costmap_ptr_->getGlobalFrameID()) {
    try {
      tf_buffer_.transform(start, start, clean_costmap_ptr_->getGlobalFrameID(), ros::Duration(1.0));
      tf_buffer_.transform(goal, goal, clean_costmap_ptr_->getGlobalFrameID(), ros::Duration(1.0));
    }
    catch(const tf2::TransformException& e) {
      ROS_INFO("[%s] can't transform starting or goal point to map frame", name_.c_str());
      return false;
    }
  }
  ROS_INFO("[%s] transforms OK", name_.c_str());
  // make plan
  bool success = global_planner_ptr_->makePlan(start, goal, plan);
  return success;
}


bool PlannerUtils::calcReachableSubplan(const std::vector<geometry_msgs::PoseStamped>& plan, 
                                        const int start_from_idx,
                                        int& reachable_idx, int& blocked_idx)
{
  // Based on plan, march forward until blocked, march backward to allow space for robot footprint
  // sanity check
  if (plan.size() == 0) { return false; }
  ROS_INFO("[%s] Input plan OK, size %lu", name_.c_str(), plan.size());
  // create reachable subplan
  int final_idx = start_from_idx;
  costmap_2d::Costmap2D* sync_costmap = sync_costmap_ptr_->getCostmap();  
  // forward march loop
  for (int i = start_from_idx; i < plan.size(); i++)
  {
    unsigned int mx, my;
    double wx = plan[i].pose.position.x;
    double wy = plan[i].pose.position.y;
    
    // check for obstacles
    if (sync_costmap->worldToMap(wx, wy, mx, my)) {
      unsigned char cost_i = sync_costmap->getCost(mx, my);
      if (cost_i == costmap_2d::LETHAL_OBSTACLE) {
        ROS_INFO("[%s] forward march, found obstruction", name_.c_str());
        break;
      }
    }
    // out of bounds
    else {
      ROS_INFO("[%s] forward march, out of bounds", name_.c_str());
      break;
    }
    final_idx = i;
  }
  blocked_idx = final_idx;
  ROS_INFO("[%s] forward march idx %d", name_.c_str(), final_idx);
  // backward march loop
  if (final_idx == plan.size()-1) {
    ROS_INFO("[%s] no obstacles found, backward march not required", name_.c_str());
  }
  else {  
    bool success = false;
    for (int i = final_idx-1; i >= start_from_idx; i--)
    {
      double dee = calcDistance(plan[final_idx].pose, plan[i].pose);
      double max_stop_distance_ = std::max(safety_radius_, reachable_plan_stop_distance_);
      if (dee >= max_stop_distance_) {
        success = true;
        final_idx = i;
        break;
      }
    }
    if (!success) {
      ROS_INFO("[%s] backward march failed. Could not find a subplan adhering to safety radius %f", name_.c_str(), max_stop_distance_);  
      return false;
    }
    ROS_INFO("[%s] backward march idx %d", name_.c_str(), final_idx);
  }
  // set output idx value
  reachable_idx = final_idx;
  ROS_INFO("[%s] final subplan idx %d", name_.c_str(), final_idx);
  return true;
}


bool PlannerUtils::makePlanToReachable(const geometry_msgs::PoseStamped& start, 
                                       const geometry_msgs::PoseStamped& goal,
                                       std::vector<geometry_msgs::PoseStamped>& plan)
{
  ROS_INFO("[%s] new reachable plan request", name_.c_str());
  // get clean plan
  if (!makeCleanPlan(start, goal, plan)) {
    return false;
  }
  // find reachable subplan
  int reachable_idx, blocked_idx = 0;
  int start_from_idx = 0;
  if (!calcReachableSubplan(plan, start_from_idx, reachable_idx, blocked_idx)) {
    return false;
  }
  // truncate plan
  if (reachable_idx < plan.size()-1) {
    plan.erase(plan.begin()+reachable_idx, plan.end());
  }
  return true;
}