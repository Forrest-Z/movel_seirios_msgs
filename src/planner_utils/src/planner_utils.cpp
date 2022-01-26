#include "planner_utils/planner_utils.hpp"


double calcDistance(geometry_msgs::Pose a, geometry_msgs::Pose b)
{
  double dx, dy;
  dx = a.position.x - b.position.x;
  dy = a.position.y - b.position.y;
  return sqrt(dx*dx + dy*dy);
}


PlannerUtils::PlannerUtils()
: tf_ear_(tf_buffer_), extra_safety_buffer_(0.1)
{
  // costmaps
  clean_costmap_ptr_ = std::make_shared<costmap_2d::Costmap2DROS>("aux_clean_map", tf_buffer_);
  sync_costmap_ptr_ = std::make_shared<costmap_2d::Costmap2DROS>("aux_sync_map", tf_buffer_);
  // global planners
  clean_global_planner_ptr_ = std::make_shared<global_planner::GlobalPlanner>();
  sync_global_planner_ptr_ = std::make_shared<global_planner::GlobalPlanner>();
  clean_global_planner_ptr_->initialize("aux_clean_planner", clean_costmap_ptr_.get());
  sync_global_planner_ptr_->initialize("aux_sync_planner", sync_costmap_ptr_.get());
  // calculate robot footprint circumscribed radius
  std::vector<geometry_msgs::Point> footprint = clean_costmap_ptr_->getRobotFootprint();
  double max_x = 0.0;
  double max_y = 0.0;
  for (int i = 0; i < footprint.size(); i++)
  {
    if (fabs(footprint[i].x) > max_x) { max_x = fabs(footprint[i].x); }
    if (fabs(footprint[i].y) > max_y) { max_y = fabs(footprint[i].y); }
    footprint_circumscribed_radius_ = sqrt(max_x*max_x + max_y*max_y);
  }
}


bool PlannerUtils::makeCleanPlan(const geometry_msgs::PoseStamped& start, 
                                 const geometry_msgs::PoseStamped& goal,
                                 std::vector<geometry_msgs::PoseStamped>& plan)
{
  bool success = PlannerUtils::makePlan(start, goal, plan, clean_costmap_ptr_, clean_global_planner_ptr_);
  return success;
}


bool PlannerUtils::makeSyncPlan(const geometry_msgs::PoseStamped& start, 
                                const geometry_msgs::PoseStamped& goal,
                                std::vector<geometry_msgs::PoseStamped>& plan)
{
  bool success = PlannerUtils::makePlan(start, goal, plan, sync_costmap_ptr_, sync_global_planner_ptr_);
  return success;
}


bool PlannerUtils::makePlan(geometry_msgs::PoseStamped start,   // copy
                            geometry_msgs::PoseStamped goal,   // copy
                            std::vector<geometry_msgs::PoseStamped>& plan,
                            const std::shared_ptr<costmap_2d::Costmap2DROS>& costmap_ptr_,
                            const std::shared_ptr<global_planner::GlobalPlanner>& global_planner_ptr_)
{
  // Verify frames are transformable
  ROS_INFO("[%s] frames start %s, goal %s, costmap %s", 
    name_.c_str(), start.header.frame_id.c_str(), 
    goal.header.frame_id.c_str(), costmap_ptr_->getGlobalFrameID().c_str()
  );
  if (start.header.frame_id != costmap_ptr_->getGlobalFrameID() || 
      goal.header.frame_id != costmap_ptr_->getGlobalFrameID()) {
    try {
      tf_buffer_.transform(start, start, costmap_ptr_->getGlobalFrameID(), ros::Duration(1.0));
      tf_buffer_.transform(goal, goal, costmap_ptr_->getGlobalFrameID(), ros::Duration(1.0));
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
  enum ObstructionType { LETHAL, INSCRIBED_INFLATED };
  ObstructionType obstruction_type = LETHAL;
  costmap_2d::Costmap2D* sync_costmap = sync_costmap_ptr_->getCostmap();  
  // forward march loop
  blocked_idx = plan.size();   // index out of bounds
  for (int i = start_from_idx; i < plan.size(); i++)
  {
    unsigned int mx, my;
    double wx = plan[i].pose.position.x;
    double wy = plan[i].pose.position.y;
    // check for obstacles
    if (sync_costmap->worldToMap(wx, wy, mx, my)) {
      unsigned char cost_i = sync_costmap->getCost(mx, my);
      if (cost_i == costmap_2d::INSCRIBED_INFLATED_OBSTACLE) {
        ROS_INFO("[%s] forward march, found obstruction on C-space inscribed inflation", name_.c_str());
        obstruction_type = INSCRIBED_INFLATED;
        blocked_idx = i;
        break;
      }
      if (cost_i == costmap_2d::LETHAL_OBSTACLE) {
        ROS_INFO("[%s] forward march, found obstruction on C-space lethal obstacle", name_.c_str());
        obstruction_type = LETHAL;
        blocked_idx = i;
        break;
      }
    }
    // out of bounds
    else {
      ROS_INFO("[%s] forward march, out of bounds", name_.c_str());
      obstruction_type = LETHAL;
      blocked_idx = i;
      break;
    }
  }
  reachable_idx = blocked_idx - 1;
  ROS_INFO("[%s] forward march idx %d", name_.c_str(), reachable_idx);
  // backward march loop
  if (reachable_idx == plan.size()-1) {
    ROS_INFO("[%s] no obstacles found, backward march not required", name_.c_str());
  }
  else {  
    // move back reachable_idx to adhere to safety distance
    double safety_dist;
    if (obstruction_type == INSCRIBED_INFLATED) { safety_dist = extra_safety_buffer_; }
    if (obstruction_type == LETHAL) { safety_dist = footprint_circumscribed_radius_ + extra_safety_buffer_; }
    geometry_msgs::Pose reachable_pose = plan[reachable_idx].pose;
    for (int i = reachable_idx; i >= 0; i--)
    {
      double dee = calcDistance(reachable_pose, plan[i].pose);
      if (dee >= safety_dist) {
        reachable_idx = i;
        break;
      }
    }
    // no safe reachable_idx
    if (reachable_idx < start_from_idx) {
      ROS_INFO("[%s] backward march failed. Could not find a subplan adhering to extra safety distance %f", name_.c_str(), safety_dist);  
      ROS_INFO("[%s] reachable_idx %d, start_from_idx %d", name_.c_str(), reachable_idx, start_from_idx); 
      return false;
    }
    ROS_INFO("[%s] backward march idx %d", name_.c_str(), reachable_idx);
  }
  // set output idx value
  ROS_INFO("[%s] final subplan idx %d", name_.c_str(), reachable_idx);  
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