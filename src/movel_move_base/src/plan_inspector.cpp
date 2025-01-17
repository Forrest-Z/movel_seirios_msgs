#include <movel_move_base/plan_inspector.h>

PlanInspector::PlanInspector(tf2_ros::Buffer* tf) : have_plan_(false), current_plan_(NULL)
{
  tf_buffer_ = tf;
  current_plan_ = new std::vector<geometry_msgs::PoseStamped>();
}

PlanInspector::PlanInspector(tf2_ros::Buffer* tf, int obstruction_thresh, double partial_blockage_path_length_thresh)
  : have_plan_(false)
  , current_plan_(NULL)
  , obstruction_threshold_(obstruction_thresh)
  , partial_blockage_path_length_threshold_(partial_blockage_path_length_thresh)
{
  tf_buffer_ = tf;
  current_plan_ = new std::vector<geometry_msgs::PoseStamped>();
}

PlanInspector::BlockageType PlanInspector::processNewPlan(const std::vector<geometry_msgs::PoseStamped>& new_plan,
                                                          const geometry_msgs::PoseStamped& active_goal)
{
  using VecPS = std::vector<geometry_msgs::PoseStamped>;

  if (have_plan_)
  {
    if (current_goal_ == active_goal)
    {
      // find remainder of current plan
      int nearest_idx = 0;
      double nearest_dist = std::numeric_limits<double>::infinity();
      VecPS& current_plan = *current_plan_;
      for (int i = 0; i < current_plan_->size(); i++)
      {
        double dist = calculateDistance(new_plan[0].pose, current_plan[i].pose);
        if (dist < nearest_dist)
        {
          nearest_dist = dist;
          nearest_idx = i;
        }
      }

      VecPS current_plan_remainder(current_plan.begin() + nearest_idx, current_plan.end());

      if (current_plan_remainder.size() < 2 || new_plan.size() < 2)
      {
        ROS_WARN("[plan_inspector] processNewPlan no path found");
        return BlockageType::FAILED;
      }

      // compare with new plan
      auto f_plan_dist = [&, this](const VecPS& plan) -> double {
        double dist = 0.0;
        for (int i = 0; i < plan.size() - 1; i++)
          dist += calculateDistance(plan[i].pose, plan[i + 1].pose);
        return dist;
      };
      double dist_current_plan_remainder = f_plan_dist(current_plan_remainder);
      double dist_new_plan = f_plan_dist(new_plan);
      double diff = abs(dist_current_plan_remainder - dist_new_plan);
      // ROS_INFO("[plan_inspector] processNewPlan plan diff: %f", diff);
      bool is_partial_blockage = diff < partial_blockage_path_length_threshold_;

      if (allow_partial_blockage_replan_ && is_partial_blockage)
        *current_plan_ = new_plan;

      return is_partial_blockage ? BlockageType::PARTIAL : BlockageType::FULL;
    }
    else
    {
      have_plan_ = false;
    }
  }

  if (!have_plan_)
  {
    current_goal_ = active_goal;
    VecPS temp_plan = new_plan;
    *current_plan_ = temp_plan;
    have_plan_ = true;
    return BlockageType::NEW_GOAL;
  }
}

PlanInspector::BlockageType PlanInspector::comparePlans(const std::vector<geometry_msgs::PoseStamped>& new_plan,
                                                        const std::vector<geometry_msgs::PoseStamped>& current_plan)
{
  using VecPS = std::vector<geometry_msgs::PoseStamped>;
  // find remainder of current plan
  int nearest_idx = 0;
  double nearest_dist = std::numeric_limits<double>::infinity();
  for (int i = 0; i < current_plan.size(); i++)
  {
    double dist = calculateDistance(new_plan[0].pose, current_plan[i].pose);
    if (dist < nearest_dist)
    {
      nearest_dist = dist;
      nearest_idx = i;
    }
  }

  VecPS current_plan_remainder(current_plan.begin() + nearest_idx, current_plan.end());

  if (current_plan_remainder.size() < 2 || new_plan.size() < 2)
  {
    ROS_WARN("[plan_inspector] processNewPlan no path found");
    return BlockageType::FAILED;
  }

  // compare with new plan
  auto f_plan_dist = [&, this](const VecPS& plan) -> double {
    double dist = 0.0;
    for (int i = 0; i < plan.size() - 1; i++)
      dist += calculateDistance(plan[i].pose, plan[i + 1].pose);
    return dist;
  };
  double dist_current_plan_remainder = f_plan_dist(current_plan_remainder);
  double dist_new_plan = f_plan_dist(new_plan);
  double diff = abs(dist_current_plan_remainder - dist_new_plan);
  // ROS_INFO("[plan_inspector] comparePlans plan diff: %f", diff);
  bool is_partial_blockage = diff < partial_blockage_path_length_threshold_;

  return is_partial_blockage ? BlockageType::PARTIAL : BlockageType::FULL;
}

bool PlanInspector::checkObstruction(const std::vector<geometry_msgs::PoseStamped>& plan,
                                     const costmap_2d::Costmap2DROS& costmap,
                                     const geometry_msgs::PoseStamped& robot_pose,
                                     geometry_msgs::PoseStamped& obstruction_pos)
{
  int max_occupancy = 0;
  bool first = true;

  // Find index of nearest waypoint in path plan
  int current_index = 0;
  double nearest_distance = sqrt(pow((robot_pose.pose.position.x - plan[0].pose.position.x), 2) +
                                 pow((robot_pose.pose.position.y - plan[0].pose.position.y), 2));

  for (int i = 1; i < plan.size(); i++)
  {
    double distance = sqrt(pow((robot_pose.pose.position.x - plan[i].pose.position.x), 2) +
                           pow((robot_pose.pose.position.y - plan[i].pose.position.y), 2));
    if (distance < nearest_distance)
    {
      nearest_distance = distance;
      current_index = i;
    }
  }

  // march through the plan
  for (int i = current_index; i < plan.size(); i++)
  {
    geometry_msgs::PoseStamped pose_i = plan[i];
    geometry_msgs::PoseStamped pose_costmap, pose_costmap_local;
    // ROS_INFO_STREAM(i << " original plan pose " << pose_i.pose.position);

    // transform plan pose to costmap frame
    geometry_msgs::TransformStamped transform =
        tf_buffer_->lookupTransform(costmap.getGlobalFrameID(), pose_i.header.frame_id, ros::Time(0));

    tf2::doTransform(pose_i.pose, pose_costmap.pose, transform);

    int row, col;
    costmap.getCostmap()->worldToMapNoBounds(pose_costmap.pose.position.x, pose_costmap.pose.position.y, row, col);

    // ROS_INFO_STREAM("dimensions " << latest_costmap_.info.height << "/" << latest_costmap_.info.width);

    // update maximum occupancy
    int costmap_width = costmap.getCostmap()->getSizeInCellsX();
    int costmap_height = costmap.getCostmap()->getSizeInCellsY();
    if (row < costmap_width && col < costmap_height && row >= 0 && col >= 0)
    {
      int occupancy = costmap.getCostmap()->getCost(row, col);

      if ((occupancy >= obstruction_threshold_) && first)
      {
        first = false;
        first_path_obs_ = pose_i;
        geometry_msgs::TransformStamped transform1 =
            tf_buffer_->lookupTransform("map", first_path_obs_.header.frame_id, ros::Time(0));

        tf2::doTransform(first_path_obs_.pose, first_path_map_.pose, transform1);
        obstruction_pos = first_path_map_;
      }
      if (occupancy > max_occupancy)
      {
        max_occupancy = occupancy;  // TODO: can have early return if obstruction detected?
      }
    }
  }
  // ROS_INFO("max occupancy %d/%d", max_occupancy, obstruction_threshold_);
  if (max_occupancy >= obstruction_threshold_)
  {
    // ROS_INFO("obstruction %d/%d", max_occupancy, obstruction_threshold_);
    return true;
  }

  return false;
}

double PlanInspector::calculateDistance(geometry_msgs::Pose a, geometry_msgs::Pose b)
{
  double dx = b.position.x - a.position.x;
  double dy = b.position.y - a.position.y;

  return sqrt(dx * dx + dy * dy);
}