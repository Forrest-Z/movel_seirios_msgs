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

    // adjust for obstacle
    if (!adjustPlanForObstacles())
      return false;

    // update pid
    // transform goal to robot frame
    geometry_msgs::PoseStamped goal_rframe, goal_i;
    try
    {
      goal_i = decimated_global_plan_[idx_plan_];
      goal_i.header.stamp = robot_pose.header.stamp;
      goal_rframe = tf_buffer_->transform(goal_i, robot_frame_);
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
    decimatePlan(plan, decimated_global_plan_, idx_map_);
    ROS_INFO("decimated plan size %lu, idx map size %lu", decimated_global_plan_.size(), idx_map_.size());
    for (int i = 0; i < idx_map_.size(); i++)
    {
      ROS_INFO("%d: %lu", i, idx_map_[i]);
    }

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

    ros::NodeHandle nl("~"+name);
    dyn_config_srv.reset(new dynamic_reconfigure::Server<pebble_local_plannerConfig>(nl));
    dyn_config_cb = boost::bind(&PebbleLocalPlanner::dynConfigCb, this, _1, _2);
    dyn_config_srv->setCallback(dyn_config_cb);

    planner_ptr_.reset(new global_planner::GlobalPlanner());
    planner_ptr_->initialize("inner_local_planner", costmap_ros);

    costmap_ptr_.reset(costmap_ros);
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
    if (nl.hasParam("xy_goal_tolerance"))
      nl.getParam("xy_goal_tolerance", xy_tolerance_);

    th_tolerance_ = 0.785;
    if (nl.hasParam("yaw_goal_tolerance"))
      nl.getParam("yaw_goal_tolerance", th_tolerance_);

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
    if (nl.hasParam("max_vel_x"))
      nl.getParam("max_vel_x", max_vx_);
    if (nl.hasParam("max_vel_theta"))
      nl.getParam("max_vel_theta", max_wz_);
    pid_.setMaxVeloes(max_vx_, max_wz_);

    max_ax_ = 0.25;
    max_alphaz_ = 1.57;
    if (nl.hasParam("acc_lim_x"))
      nl.getParam("acc_lim_x", max_ax_);
    if (nl.hasParam("acc_lim_theta"))
      nl.getParam("acc_lim_theta", max_alphaz_);

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


  int PebbleLocalPlanner::decimatePlan(const std::vector<geometry_msgs::PoseStamped> &plan_in, 
                                       std::vector<geometry_msgs::PoseStamped> &plan_out,
                                       std::vector<size_t> &idx_map)
  {
    plan_out.clear();
    plan_out.push_back(plan_in[0]);
    geometry_msgs::PoseStamped last_pose = plan_in[0];

    idx_map.clear();
    idx_map.push_back(0);

    for (int i = 1; i < plan_in.size()-1; i++)
    {
      double dee = calcPoseDistance(last_pose, plan_in[i]);
      if (dee > d_min_)
      {
        plan_out.push_back(plan_in[i]);
        idx_map.push_back(i);
        last_pose = plan_in[i];
      }
    }
    plan_out.push_back(plan_in[plan_in.size()-1]);
    idx_map.push_back(plan_in.size()-1);
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
      max_vx = max_vx_ * fabs(ex)/d_min_;
      max_vx = std::min(max_vx_, max_vx);
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

  void PebbleLocalPlanner::dynConfigCb(pebble_local_planner::pebble_local_plannerConfig &config, uint32_t level)
  {
    ROS_INFO("[%s] new config!", name_.c_str());
    d_min_ = config.d_min;
    xy_tolerance_ = config.xy_goal_tolerance;
    th_tolerance_ = config.yaw_goal_tolerance;
    max_vx_ = config.max_vel_x;
    max_wz_ = config.max_vel_theta;
    max_ax_ = config.acc_lim_x;
    max_alphaz_ = config.acc_lim_theta;
    allow_reverse_ = config.allow_reverse;
    ROS_INFO("allow reverse is now %d", allow_reverse_);
    th_turn_ = config.th_turn;
  }

  bool PebbleLocalPlanner::adjustPlanForObstacles()
  {
    costmap_2d::Costmap2D* costmap = costmap_ptr_->getCostmap();
    double wx, wy;
    unsigned int mx, my;
    unsigned char cost_idx;
    ROS_INFO("eval obstacle, start at index %d", idx_plan_);
    // ROS_INFO_STREAM("free is" << costmap_2d::FREE_SPACE << ", inflate is " << 
                    // costmap_2d::INSCRIBED_INFLATED_OBSTACLE << ", lethal is " << costmap_2d::LETHAL_OBSTACLE);
    double prev_drobot = 1000.*d_min_;
    size_t jdx = idx_map_[idx_plan_];
    geometry_msgs::PoseStamped robot_pose;
    if (!getRobotPose(robot_pose))
    {
      ROS_INFO("can't get robot pose for obstacle checking");
      return false;
    }

    // march backwards
    while (true)
    {
      ROS_INFO("eval obstacle jdx %lu", jdx);
      wx = global_plan_[jdx].pose.position.x;
      wy = global_plan_[jdx].pose.position.y;
      if (costmap->worldToMap(wx, wy, mx, my))
      {
        cost_idx = costmap->getCost(mx, my);
        if (cost_idx == costmap_2d::FREE_SPACE)
        {
          // decide if we should continue
          // if the distance between global_plan pose to robot increases,
          // then we have passed the closest pose to robot 
          // and the backwards march is complete
          double drobot = calcPoseDistance(robot_pose, global_plan_[jdx]);
          if (drobot > prev_drobot)
          {
            ROS_INFO("backwards march clear");
            return true; 
          }
          prev_drobot = drobot;
          // if we get to zero without finding obstruction, then all's clear
          // jdx is size_t remember, so it's unsigned, 
          // don't be clever and try to do a jdx < 0, you'll get Bad Time
          if (jdx == 0)
          {
            ROS_INFO("backwards march clear by index zero");
            return true;
          }
          --jdx;
        }
        else
        {
          // OK, space between robot and pebble is not free, should be skip the pebble?
          double space_to_pebble = calcPoseDistance(global_plan_[jdx], decimated_global_plan_[idx_plan_]);
          if (space_to_pebble > 0.30) // parametrise to robot radius or footprint
          {
            // there is enough space, replan to same index
            std::vector<geometry_msgs::PoseStamped> interplan, decim_interplan;
            std::vector<size_t> idx_interplan;
            if (planner_ptr_->makePlan(robot_pose, decimated_global_plan_[idx_plan_], interplan))
            {
              // decimate, then insert the obstacle avoiding plan
              int len_decim_interplan = decimatePlan(interplan, decim_interplan, idx_interplan);
              decimated_global_plan_.insert(decimated_global_plan_.begin()+idx_plan_, decim_interplan.begin(), decim_interplan.end());
              return true;
            }
            else
            {
              ROS_INFO("failed to make interplan");
              return false;
            }
          }
          else
          {
            // there isn't enough space, march forward and find the next free index
            jdx = idx_map_[idx_plan_]+1;
            while (true)
            {
              // if we reach the end of global plan without finding a free index, there is none
              if (jdx >= global_plan_.size())
              {
                ROS_INFO("failed to find free index during forwards march");
                return false;
              }
              wx = global_plan_[jdx].pose.position.x;
              wy = global_plan_[jdx].pose.position.y;
              if (costmap->worldToMap(wx, wy, mx, my))
              {
                cost_idx = costmap->getCost(mx, my);
                if (cost_idx == costmap_2d::FREE_SPACE)
                {
                  // found free space
                  // now, find the closest pebble
                  size_t idx_free = idx_plan_;
                  while (jdx > idx_map_[idx_free])
                  {
                    idx_free++;
                    if (idx_free >= decimated_global_plan_.size())
                    {
                      idx_free = decimated_global_plan_.size()-1;
                      break;
                    }
                  }

                  // make plan there
                  std::vector<geometry_msgs::PoseStamped> interplan, decim_interplan;
                  std::vector<size_t> idx_interplan;
                  if (planner_ptr_->makePlan(robot_pose, decimated_global_plan_[idx_free], interplan))
                  {
                    // decimate interplan
                    int len_decim_interplan = decimatePlan(interplan, decim_interplan, idx_interplan);

                    // remove blocked pebbles
                    decimated_global_plan_.erase(decimated_global_plan_.begin()+idx_plan_, decimated_global_plan_.begin()+idx_free);

                    // insert decimated interplan
                    decimated_global_plan_.insert(decimated_global_plan_.begin()+idx_plan_, decim_interplan.begin(), decim_interplan.end());
                    return true;
                  }
                  else
                  {
                    ROS_INFO("failed to make interplan");
                    return false;
                  }
                }
                ++jdx;
              }
              else
              {
                ROS_INFO("forwards march has index outside local costmap?");
                return false;
              }
            }
          }

        }
      }
      else
      {
        ROS_INFO("backwards march has index outside local costmap?");
        return false;
      }
    }
  }

  bool PebbleLocalPlanner::adjustPlanForObstacles2()
  {
    // check if current index is obstructed
    costmap_2d::Costmap2D* costmap = costmap_ptr_->getCostmap();

    double wx, wy;
    unsigned int mx, my;
    unsigned char cost_idx;

    int idx_free = idx_plan_;
    ROS_INFO("eval obstacle, start at index %d", idx_plan_);
    ROS_INFO_STREAM("free is" << costmap_2d::FREE_SPACE << ", inflate is " << 
                    costmap_2d::INSCRIBED_INFLATED_OBSTACLE << ", lethal is " << costmap_2d::LETHAL_OBSTACLE);
    while (true)
    {
      wx = decimated_global_plan_[idx_free].pose.position.x;
      wy = decimated_global_plan_[idx_free].pose.position.y;
      if (costmap->worldToMap(wx, wy, mx, my))
      {
        cost_idx = costmap->getCost(mx, my);
        ROS_INFO_STREAM("idx " << idx_free << " is " << cost_idx);
        if (cost_idx == costmap_2d::FREE_SPACE)
        {
          if (idx_free != idx_plan_)
          {
            ROS_INFO("%d to %d are blocked", idx_plan_, idx_free);
            // make plan there
            int idx_start = std::max(0, idx_plan_-1);
            std::vector<geometry_msgs::PoseStamped> interplan;
            if (planner_ptr_->makePlan(decimated_global_plan_[idx_start], decimated_global_plan_[idx_free], interplan))
            {
              // remove blocked points
              decimated_global_plan_.erase(decimated_global_plan_.begin()+idx_plan_, decimated_global_plan_.begin()+idx_free);

              // insert new plan
              decimated_global_plan_.insert(decimated_global_plan_.begin()+idx_start+1, interplan.begin(), interplan.end());
              ROS_INFO("inserted %lu points to avoid obstacle", interplan.size());
            }
            else
            {
              ROS_INFO("failed to produce plan to avoid obstacle");
              return false;
            }
          }
          return true;
        }
        else
        {
          // find next free index
          idx_free += 1;
          if (idx_free >= decimated_global_plan_.size()-1)
          {
            ROS_INFO("no more free index in the plan");
            return false;
          }
        }
      }
      else
      {
        ROS_INFO("no free index in the local costmap");
        return false;
      }
    }

    
  }
}