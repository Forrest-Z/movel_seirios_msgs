#include "movel_fms_utils/plan_measurer.hpp"

PlanMeasurer::PlanMeasurer() : tf_ear_(tf_buffer_)
{
}

bool PlanMeasurer::measQrySimpleCb(movel_seirios_msgs::GetPlanLengthSimple::Request &req,
                                   movel_seirios_msgs::GetPlanLengthSimple::Response &res)
{
  geometry_msgs::Pose robot_pose;
  if (getRobotPose(robot_pose))
  {
    nav_msgs::Path plan = getPlan(robot_pose, req.goal);
    if (plan.poses.size() < 2)
    {
      res.length = -1.0;
      return true;
    }

    res.length = measurePlan(plan);
    return true;
  }
  else
  {
    res.length = -1.0;
    return true;
  }
}

bool PlanMeasurer::measQryCb(movel_seirios_msgs::GetPlanLength::Request &req,
                             movel_seirios_msgs::GetPlanLength::Response &res)
{
  if (req.goals.size() < 1)
  {
    res.length = -1.0;
    return true;
  }

  double plan_length = 0.0;
  geometry_msgs::Pose robot_pose;
  if (getRobotPose(robot_pose))
  {
    // incrementally add up plan length between waypoints, 
    // if a waypoint is unreachable, skip it (length 0), except for the last one
    geometry_msgs::Pose wp_a, wp_b;
    wp_a = robot_pose;
    
    for (int i = 0; i < req.goals.size()-1; i++)
    {
      wp_b = req.goals[i];
    
      nav_msgs::Path plan_i = getPlan(wp_a, wp_b);
      plan_length += measurePlan(plan_i);

      wp_a = wp_b;
    }

    // special case for final waypoint: if it is unreachable, then return invalid
    wp_b = req.goals[req.goals.size()-1];
    nav_msgs::Path plan_i = getPlan(wp_a, wp_b);
    if (plan_i.poses.size() < 1)
    {
      res.length = -1.0;
      return true;
    }
    plan_length += measurePlan(plan_i);
    res.length = plan_length;
    return true;
  }
  else
  {
    res.length = -1.0;
    return true;
  }
}

bool PlanMeasurer::getRobotPose(geometry_msgs::Pose &robot_pose)
{
  geometry_msgs::TransformStamped transform;
  try
  {
    transform = tf_buffer_.lookupTransform(map_frame_, robot_frame_, ros::Time(0));
  }
  catch(const tf2::TransformException &e)
  {
    ROS_ERROR("failed to get transform %s", e.what());
    return false;
  }

  robot_pose.position.x = transform.transform.translation.x;
  robot_pose.position.y = transform.transform.translation.y;
  robot_pose.position.z = transform.transform.translation.z;
  robot_pose.orientation = transform.transform.rotation;

  return true;  
}

nav_msgs::Path PlanMeasurer::getPlan(geometry_msgs::Pose start, geometry_msgs::Pose goal)
{
  geometry_msgs::PoseStamped start_stamped;
  start_stamped.header.frame_id = map_frame_;
  start_stamped.header.stamp = ros::Time::now();
  start_stamped.pose = start;

  geometry_msgs::PoseStamped goal_stamped;
  goal_stamped.header.frame_id = map_frame_;
  goal_stamped.header.stamp = ros::Time::now();
  goal_stamped.pose = goal;

  nav_msgs::GetPlan srv;
  srv.request.start = start_stamped;
  srv.request.goal = goal_stamped;

  planner_service_.call(srv);

  return srv.response.plan;
}

double PlanMeasurer::measurePlan(nav_msgs::Path plan)
{
  double length = 0.0;
  if (plan.poses.size() < 2)
    return length;

  for (int i = 1; i < plan.poses.size(); i++)
  {
    double dx, dy, dz;
    dx = plan.poses[i].pose.position.x - plan.poses[i-1].pose.position.x;
    dy = plan.poses[i].pose.position.y - plan.poses[i-1].pose.position.y;
    dz = plan.poses[i].pose.position.z - plan.poses[i-1].pose.position.z;

    length += sqrt(dx*dx + dy*dy + dz*dz);
  }

  return length;
}