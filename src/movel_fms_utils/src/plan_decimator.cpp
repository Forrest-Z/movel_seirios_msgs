#include <movel_fms_utils/plan_decimator.hpp>

PlanDecimator::PlanDecimator()
{
}

void PlanDecimator::globalPlanCb(nav_msgs::Path msg)
{
  // ROS_INFO("new global plan, size %lu", msg.poses.size());
  latest_path_raw_ = msg;
  decimatePlan(msg, latest_path_decimated_, d_decimation_);
  // ROS_INFO("decimation complete, new size %lu", latest_path_decimated_.poses.size());

  decimated_plan_pub_.publish(latest_path_decimated_);
}

bool PlanDecimator::getDecimatedPlanCb(movel_seirios_msgs::GetDecimatedPlan::Request &req,
                                       movel_seirios_msgs::GetDecimatedPlan::Response &res)
{
  if (latest_path_raw_.poses.size() == 0 || req.decimation <= 0.0)
  {
    res.plan = latest_path_raw_;
    return false;
  }

  if (fabs(req.decimation - d_decimation_) > 1.0e-2)
  {
    nav_msgs::Path decimated_plan;
    decimatePlan(latest_path_raw_, decimated_plan, req.decimation);
    res.plan = decimated_plan;
    return true;
  }
  else
  {
    res.plan = latest_path_decimated_;
    return true;
  }
  
}

bool PlanDecimator::setDecimationCb(movel_seirios_msgs::SetPlanDecimation::Request &req,
                                    movel_seirios_msgs::SetPlanDecimation::Response &res)
{
  if (req.decimation > 0.0)
  {
    d_decimation_ = req.decimation;
    res.ack = true;
    return true;
  }
  res.ack = false;
  return true;
}

void PlanDecimator::decimatePlan(nav_msgs::Path &in_plan, nav_msgs::Path &out_plan, double decimation)
{
  out_plan.poses.clear();
  out_plan.header = in_plan.header;

  geometry_msgs::PoseStamped pose_ref = in_plan.poses[0];
  out_plan.poses.push_back(in_plan.poses[0]);

  for (int i = 1; i < in_plan.poses.size(); i++)
  {
    double d = calcDistance(in_plan.poses[i].pose, pose_ref.pose);
    if (d > decimation || i == in_plan.poses.size()-1)
    {
      double theta = calcAngle(pose_ref.pose, in_plan.poses[i].pose);
      pose_ref = in_plan.poses[i];
      out_plan.poses.push_back(in_plan.poses[i]);

      // set the orientation of the waypoint before this one
      int prev_idx = out_plan.poses.size() - 2;
      out_plan.poses[prev_idx].pose.orientation.w = cos(0.5*theta);
      out_plan.poses[prev_idx].pose.orientation.z = sin(0.5*theta);
    }
  }

  // set the orientation of the final waypoint
  if (out_plan.poses.size() > 1)
  {
    int last_idx = out_plan.poses.size() - 1;
    out_plan.poses[last_idx].pose.orientation = out_plan.poses[last_idx-1].pose.orientation;
  }
}

double PlanDecimator::calcDistance(geometry_msgs::Pose a, geometry_msgs::Pose b)
{
  double dx, dy, dz;
  dx = a.position.x - b.position.x;
  dy = a.position.y - b.position.y;
  dz = a.position.z - b.position.z;

  return sqrt(dx*dx + dy*dy + dz*dz);
}

double PlanDecimator::calcAngle(geometry_msgs::Pose a, geometry_msgs::Pose b)
{
  double dx, dy;
  dx = b.position.x - a.position.x;
  dy = b.position.y - a.position.y;

  return atan2(dy, dx);
}