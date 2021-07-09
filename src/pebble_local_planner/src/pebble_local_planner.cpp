#include <algorithm>
#include <pluginlib/class_list_macros.h>
#include "pebble_local_planner/pebble_local_planner.h"

PLUGINLIB_EXPORT_CLASS(pebble_local_planner::PebbleLocalPlanner, nav_core::BaseLocalPlanner)

void quaternionToRPY(geometry_msgs::Quaternion q, double &r, double &p, double &y)
{
  r = atan2(2*(q.w*q.x + q.y*q.z), 1-2*(q.x*q.x + q.y*q.y));
  p = asin(2*(q.w*q.y - q.z*q.x));
  y = atan2(2*(q.w*q.z + q.x*q.y), 1-2*(q.y*q.y + q.z*q.z));
}

double calcPoseDistance(geometry_msgs::PoseStamped a, geometry_msgs::PoseStamped b)
{
  double dx, dy, dee;
  dx = a.pose.position.x - b.pose.position.x;
  dy = a.pose.position.y - b.pose.position.y;
  dee = sqrt(dx*dx + dy*dy);
  return dee;
}

double calcAlongScore(Pt p, Pt l0, Pt l1)
{
  // line vector
  double lx = l1.x - l0.x;
  double ly = l1.y - l0.y; 

  // point vector
  double px = p.x - l0.x;
  double py = p.y - l0.y;
  double dotprod = lx*px + ly*py;
  double linemag = lx*lx + ly*ly;

  return dotprod/linemag;
}

namespace pebble_local_planner
{
  PebbleLocalPlanner::PebbleLocalPlanner()
  {
    ROS_INFO("pebble constructed");
  }

  bool PebbleLocalPlanner::computeVelocityCommands(geometry_msgs::Twist& cmd_vel)
  {
    if (goal_reached_)
      return false;
    static ros::Time prev_t = ros::Time::now();
    // ROS_INFO("[%s] calcul velo, got %5.2f, %5.2f", name_.c_str(), cmd_vel.linear.x, cmd_vel.angular.z);
    cmd_vel.linear.x = 0.;
    cmd_vel.linear.y = 0.;
    cmd_vel.linear.z = 0.;
    cmd_vel.angular.x = 0.;
    cmd_vel.angular.y = 0.;
    cmd_vel.angular.z = 0.;

    // attempt to advance plan index
    geometry_msgs::PoseStamped robot_pose;
    if(!getRobotPose(robot_pose))
      return false;
    // find correct index in case of overshoot or other nudges
    idx_plan_ = findIdxAlongPlan(robot_pose, decimated_global_plan_, idx_plan_);
    // eval if we should advance the index
    double dee = calcPoseDistance(robot_pose, decimated_global_plan_[idx_plan_]);
    if (dee < 0.5*d_min_)
    {
      ++idx_plan_;
      idx_plan_ = std::min(idx_plan_, (int)decimated_global_plan_.size()-1);
    }
    // ROS_INFO("index OK %d/%lu", idx_plan_, decimated_global_plan_.size());
    // double rr, pp, yy;
    // quaternionToRPY(robot_pose.pose.orientation, rr, pp, yy);
    // ROS_INFO_STREAM("robot pose OK " << robot_pose.pose.position.x << ", " << robot_pose.pose.position.y <<
    //                 ", " << robot_pose.pose.orientation.w << "/" << yy << " at " << robot_pose.header.stamp);

    // update pid
    // transform goal to robot frame
    geometry_msgs::PoseStamped goal_rframe, goal_i;
    try
    {
      goal_i = decimated_global_plan_[idx_plan_];
      goal_i.header.stamp = robot_pose.header.stamp;
      goal_rframe = tf_buffer_->transform(goal_i, robot_frame_);
      // if (fabs(goal_rframe.pose.position.x) < 0.5*d_min_)
      // {
      //   ++idx_plan_;
      //   idx_plan_ = std::min(idx_plan_, (int)decimated_global_plan_.size()-1);

      //   goal_i = decimated_global_plan_[idx_plan_];
      //   goal_i.header.stamp = robot_pose.header.stamp;
      //   goal_rframe = tf_buffer_->transform(goal_i, robot_frame_);
      // }
    }
    catch (const std::exception& e)
    {
      ROS_INFO("failed to transform goal to robot frame %s", e.what());
      return false;
    }
    waypoint_pub_.publish(decimated_global_plan_[idx_plan_]);

    double r, p, th_ref;
    quaternionToRPY(goal_rframe.pose.orientation, r, p, th_ref);
    double dt = (ros::Time::now() - prev_t).toSec();
    // call update
    double vx, wz;
    // quaternionToRPY(decimated_global_plan_[idx_plan_].pose.orientation, rr, pp, yy);
    // ROS_INFO("goal OK %5.3f, %5.3f, %8.6f", 
    //          decimated_global_plan_[idx_plan_].pose.position.x, decimated_global_plan_[idx_plan_].pose.position.y, yy);
    // ROS_INFO("goal OK %5.3f, %5.3f, %8.6f", 
    //          goal_rframe.pose.position.x, goal_rframe.pose.position.y, th_ref);

    // pid_.update(goal_rframe.pose.position.x, goal_rframe.pose.position.y, th_ref, 0., 0., 0., dt, vx, wz);
    // ROS_INFO("pid update ok %5.2f, %5.2f", vx, wz);
    calcVeloSimple(goal_rframe.pose.position.x, goal_rframe.pose.position.y, th_ref, dt, vx, wz);
    // ROS_INFO("velo update OK %5.2f, %5.2f", vx, wz);

    prev_vx_ = vx;
    prev_wz_ = wz;

    // prep cmd_vel
    cmd_vel.linear.x = vx;
    cmd_vel.angular.z = wz;

    // completion check
    if (idx_plan_ == decimated_global_plan_.size()-1 && 
        calcPoseDistance(decimated_global_plan_[decimated_global_plan_.size()-1], robot_pose) < xy_tolerance_ && 
        fabs(th_ref) < th_tolerance_)
    {
      goal_reached_ = true;
    }

    prev_t = ros::Time::now();
    return true;
  }


  bool PebbleLocalPlanner::isGoalReached()
  {
    if (goal_reached_)
      ROS_INFO("PEBBLE GOAL!");
    return goal_reached_;
  }


  bool PebbleLocalPlanner::setPlan(const std::vector<geometry_msgs::PoseStamped>& plan)
  {
    ROS_INFO("[%s] set plan called, size %lu", name_.c_str(), plan.size());
    global_plan_ = plan;
    // for (int i = 0; i < plan.size(); i++)
    // {
    //   double th_deg = 180. / M_PI * 2. * acos(plan[i].pose.orientation.w);
    //   ROS_INFO("wp %d/%lu: (%5.2f, %5.2f, %5.2f)", i, plan.size(), plan[i].pose.position.x, plan[i].pose.position.y, th_deg);
    // }

    geometry_msgs::PoseStamped robot_pose;
    if (!getRobotPose(robot_pose))
    {
      ROS_INFO("[%s] failed to get robot pose", name_.c_str());
      return false;
    }

    // decimate plan
    decimatePlan(plan, decimated_global_plan_);

    // find index closest to robot position in plan
    idx_plan_ = findIdxAlongPlan(robot_pose, decimated_global_plan_, 1);

    // publish path, waypoint
    nav_msgs::Path path;
    path.header = decimated_global_plan_[0].header;
    path.poses = decimated_global_plan_;
    decimated_path_pub_.publish(path);
    waypoint_pub_.publish(decimated_global_plan_[idx_plan_]);
    pid_.reset();

    goal_reached_ = false;

    return true;
  }


  void PebbleLocalPlanner::initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros)
  {
    ROS_INFO("Pebble Local Planner inited with name %s", name.c_str());
    name_ = name;
    tf_buffer_ = tf;
    loadParams();

    ros::NodeHandle nh;
    decimated_path_pub_ = nh.advertise<nav_msgs::Path>("pebble_path", 1);
    waypoint_pub_ = nh.advertise<geometry_msgs::PoseStamped>("pebble", 1);
  }

  bool PebbleLocalPlanner::loadParams()
  {
    ros::NodeHandle nl("~"+name_);
    // std::string resolved = nl.resolveName("d_min");
    // ROS_INFO("resolved dmin %s", resolved.c_str());
    // planner related
    d_min_ = 0.1;
    if (nl.hasParam("d_min"))
      nl.getParam("d_min", d_min_);

    robot_frame_ = "base_link";
    if (nl.hasParam("robot_frame"))
      nl.getParam("robot_frame", robot_frame_);

    map_frame_ = "map";
    if (nl.hasParam("map_frame"))
      nl.getParam("map_frame", map_frame_);

    xy_tolerance_ = 0.30;
    if (nl.hasParam("xy_tolerance"))
      nl.getParam("xy_tolerance", xy_tolerance_);

    th_tolerance_ = 0.785;
    if (nl.hasParam("th_tolerance"))
      nl.getParam("th_tolerance", th_tolerance_);

    // PID needs linear tolerance smaller than decimation or you get Bad Time
    // angular tolerance is more arbitrary, but let's set it smaller than planner tolerance
    // so the planner will increment the index before the robot can stop at a waypoint
    pid_.setTolerances(0.25*d_min_, 0.75*th_tolerance_);

    // pid related
    double kpl = 1.;
    double kil = 0.;
    double kdl = 0.;
    if (nl.hasParam("kp_linear"))
      nl.getParam("kp_linear", kpl);

    if (nl.hasParam("ki_linear"))
      nl.getParam("ki_linear", kil);

    if (nl.hasParam("kd_linear"))
      nl.getParam("kd_linear", kdl);

    pid_.setLinGains(kpl, kil, kdl);

    double kpa = 1.;
    double kia = 0.;
    double kda = 0.;
    if (nl.hasParam("kp_angular"))
      nl.getParam("kp_angular", kpa);

    if (nl.hasParam("ki_angular"))
      nl.getParam("ki_angular", kia);

    if (nl.hasParam("kd_angular"))
      nl.getParam("kd_angular", kda);

    pid_.setAngGains(kpa, kia, kda);

    max_vx_ = 0.3;
    max_wz_ = 0.785;
    if (nl.hasParam("max_vx"))
      nl.getParam("max_vx", max_vx_);
    if (nl.hasParam("max_wz"))
      nl.getParam("max_wz", max_wz_);
    pid_.setMaxVeloes(max_vx_, max_wz_);

    max_ax_ = 0.25;
    max_alphaz_ = 1.57;
    if (nl.hasParam("max_ax"))
      nl.getParam("max_ax", max_ax_);
    if (nl.hasParam("max_alphaz"))
      nl.getParam("max_alphaz", max_alphaz_);

    allow_reverse_ = false;
    if (nl.hasParam("allow_reverse"))
      nl.getParam("allow_reverse", allow_reverse_);
    pid_.setAllowReverse(allow_reverse_);

    th_turn_ = M_PI/4.;
    if (nl.hasParam("th_turn"))
      nl.getParam("th_turn", th_turn_);
    pid_.setTurnThresh(th_turn_);

    return true;
  }


  int PebbleLocalPlanner::decimatePlan(const std::vector<geometry_msgs::PoseStamped> &plan_in, std::vector<geometry_msgs::PoseStamped> &plan_out)
  {
    plan_out.clear();
    plan_out.push_back(plan_in[0]);
    geometry_msgs::PoseStamped last_pose = plan_in[0];

    for (int i = 1; i < plan_in.size()-1; i++)
    {
      double dee = calcPoseDistance(last_pose, plan_in[i]);
      if (dee > d_min_)
      {
        plan_out.push_back(plan_in[i]);
        last_pose = plan_in[i];
      }
    }
    plan_out.push_back(plan_in[plan_in.size()-1]);
    return plan_out.size();
  }


  bool PebbleLocalPlanner::getRobotPose(geometry_msgs::PoseStamped &robot_pose)
  {
    try
    {
      geometry_msgs::TransformStamped transform = tf_buffer_->lookupTransform(map_frame_, robot_frame_, ros::Time(0));
      robot_pose.header = transform.header;
      robot_pose.pose.position.x = transform.transform.translation.x;
      robot_pose.pose.position.y = transform.transform.translation.y;
      robot_pose.pose.position.z = transform.transform.translation.z;
      robot_pose.pose.orientation = transform.transform.rotation;
      return true;
    }
    catch(const std::exception& e)
    {
      ROS_INFO("failed to get robot transform %s", e.what());
      return false;
    }
  }

  int PebbleLocalPlanner::findIdxAlongPlan(geometry_msgs::PoseStamped &robot_pose, 
                                           std::vector<geometry_msgs::PoseStamped> &plan, int start_idx)
  {
    int idx = start_idx;
    for (int i = start_idx; i < plan.size()-2; i++)
    {
      Pt p;
      p.x = robot_pose.pose.position.x;
      p.y = robot_pose.pose.position.y;

      geometry_msgs::PoseStamped plan_i, plan_j;
      plan_i = plan[i];
      plan_i.header.stamp = robot_pose.header.stamp;
      plan_j = plan[i+1];
      plan_j.header.stamp = robot_pose.header.stamp;
      if (robot_pose.header.frame_id != plan[i].header.frame_id)
      {
        try
        {
          plan_i = tf_buffer_->transform(plan_i, robot_pose.header.frame_id);
          plan_j = tf_buffer_->transform(plan_j, robot_pose.header.frame_id);
        }
        catch (const tf2::TransformException &e)
        {
          ROS_INFO("failed to transform plan to robot frame %s", e.what());
          idx = i;
          break;
        }
      }
      Pt l0, l1;
      l0.x = plan_i.pose.position.x;
      l0.y = plan_i.pose.position.y;

      l1.x = plan_j.pose.position.x;
      l1.y = plan_j.pose.position.y;

      double along = calcAlongScore(p, l0, l1);
      if (along < 0.0)
      {
        idx = i;
        break;
      }
      else if (along < 1.)
      {
        idx = i+1;
        break;
      }
    }
    return idx;
  }

  void PebbleLocalPlanner::calcVeloSimple(double xref, double yref, double thref, double dt, double &vx, double &wz)
  {
    // references must already be in robot frame!!!
    double ex = xref;
    double eth = atan2(yref, xref);
    bool reverse = false;
    ROS_INFO("raw errors %5.2f, %5.2f", ex, eth);

    if (ex < 0 && allow_reverse_)
    {
      eth = fmod(eth + M_PI, 2.*M_PI);
      if (eth < -M_PI)
        eth = 2.*M_PI + eth;
      else if (eth > M_PI)
        eth = -2.*M_PI + eth;
      reverse = true;
      ROS_INFO("I should reverse");
    }

    // calculate speed references
    // modulate max vx if at last index
    double max_vx = max_vx_;
    if (idx_plan_ == decimated_global_plan_.size()-1)
    {
      // max_vx = sqrt(2.* max_ax_ * fabs(ex));
      max_vx = max_vx_ * (1. - fabs(ex)/d_min_);
      max_vx = std::max(0., max_vx);
      if (ex < 0.25*xy_tolerance_)
        eth = thref;
      ROS_INFO("is final waypoint, max_vx %5.2f, eth %5.2f", max_vx, eth);
    }
    ROS_INFO("errors %5.2f, %5.2f", ex, eth);

    if (eth > th_turn_)
    {
      wz = max_wz_;
      vx = 0.;
    }
    else if (eth < -th_turn_)
    {
      wz = -max_wz_;
      vx = 0.;
    }
    else
    {
      wz = eth * max_wz_ / th_turn_;
      vx = max_vx * (1. - fabs(eth)/th_turn_);
      if (reverse)
        vx *= -1.;
    }

    ROS_INFO("raw veloes %5.2f, %5.2f", vx, wz);
    
    // enforce acceleration limits
    double dv = vx - prev_vx_;
    if ((dt < 1.e-3 && dv > 0) || dv/dt > max_ax_)
      vx = prev_vx_ + max_ax_*dt;
    else if (dv/dt < -max_ax_)
      vx = prev_vx_ - max_ax_*dt;
    
    double dw = wz - prev_wz_;
    if ((dt < 1.e-3 && dv < 0) || dw/dt > max_alphaz_)
      wz = prev_wz_ + max_alphaz_*dt;
    else if (dv/dt < -max_alphaz_)
      wz = prev_wz_ - max_alphaz_*dt;

    ROS_INFO("dt %5.2f, dvx/dt %5.2f, dwz/dt %5.2f", dt, dv/dt, dw/dt);
    ROS_INFO("final veloes %5.2f, %5.2f", vx, wz);
    ROS_INFO("---");

  }
}