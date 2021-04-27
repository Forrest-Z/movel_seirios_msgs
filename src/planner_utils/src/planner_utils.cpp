#include "planner_utils/planner_utils.hpp"

double calcDistance(geometry_msgs::Pose a, geometry_msgs::Pose b)
{
  double dx, dy;
  dx = a.position.x - b.position.x;
  dy = a.position.y - b.position.y;
  return sqrt(dx*dx + dy*dy);
}

PlannerUtils::PlannerUtils()
: tf_ear_(tf_buffer_)
, safety_radius_(1.0)
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

bool PlannerUtils::makePlanToReachable(geometry_msgs::PoseStamped start, geometry_msgs::PoseStamped goal,
                                       std::vector<geometry_msgs::PoseStamped>& plan)
{
  ROS_INFO("new reachable plan request");
  // Verify frames are transformable
  ROS_INFO("frames start %s, goal %s, costmap %s", start.header.frame_id.c_str(), goal.header.frame_id.c_str(), clean_costmap_ptr_->getGlobalFrameID().c_str());
  if (start.header.frame_id != clean_costmap_ptr_->getGlobalFrameID() || 
      goal.header.frame_id != clean_costmap_ptr_->getGlobalFrameID())
  {
    try
    {
      tf_buffer_.transform(start, start, clean_costmap_ptr_->getGlobalFrameID(), ros::Duration(1.0));
      tf_buffer_.transform(goal, goal, clean_costmap_ptr_->getGlobalFrameID(), ros::Duration(1.0));
    }
    catch(const tf2::TransformException& e)
    {
      ROS_INFO("can't transform starting or goal point to map frame");
      return false;
    }
  }
  ROS_INFO("transforms OK");

  // Get plan in clean map, march forward until blocked, march backward to allow space for robot footprint
  if(global_planner_ptr_->makePlan(start, goal, plan))
  {
    ROS_INFO("clean plan OK, size %lu", plan.size());
    int final_idx = 0;
    costmap_2d::Costmap2D* sync_costmap = sync_costmap_ptr_->getCostmap();
    
    // forward march loop
    for (int i = 0; i < plan.size(); i++)
    {
      unsigned int mx, my;
      double wx = plan[i].pose.position.x;
      double wy = plan[i].pose.position.y;

      if (sync_costmap->worldToMap(wx, wy, mx, my))
      {
        unsigned char cost_i = sync_costmap->getCost(mx, my);
        if (cost_i == costmap_2d::LETHAL_OBSTACLE || cost_i == costmap_2d::INSCRIBED_INFLATED_OBSTACLE)
        {
          ROS_INFO("forward march, found obstruction");
          final_idx = std::max(0, i-1);
          break;
        }
      }
      else
      {
        ROS_INFO("forward march, out of bounds");
        final_idx = std::max(0, i-1);
        break;
      }
      final_idx = i;
    }
    ROS_INFO("forward march idx %d", final_idx);

    // backward march loop
    if (final_idx < plan.size()-1)
    {  
      for (int i = final_idx-1; i >= 0; i--)
      {
        double dee = calcDistance(plan[final_idx].pose, plan[i].pose);
        if (dee >= safety_radius_ || i == 0)
        {
          final_idx = i;
          break;
        }
      }
      ROS_INFO("backward march idx %d", final_idx);
    }

    // truncate plan to final_idx
    plan.erase(plan.begin()+final_idx, plan.end());
    ROS_INFO("final plan size %lu", plan.size());
    return true;
  }
  else
    return false;
}