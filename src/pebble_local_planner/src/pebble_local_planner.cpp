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


  PebbleLocalPlanner::~PebbleLocalPlanner()
  {
    planner_ptr_.reset();
    costmap_ptr_.reset();
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
    if (local_obsav_) {
      if (N_lookahead_ < 1) {
        if (!adjustPlanForObstacles())
          return false;
      }
      else {
        if (!planAheadForObstacles(N_lookahead_))
          return false;
      }
    }
    // do not proceed if pebble is obstructed since there is no local obstacle avoidance
    else {
      if (N_lookahead_ < 1) {
        if (checkPebbleObstructed(idx_plan_)) {   // just check current pebble
          ROS_INFO("[%s] Current pebble is obstructed!", name_.c_str());  
          return false;
        }
      }
      else {
        for (int n = 0; n < N_lookahead_; n++) {   // check all for all lookahead
          if (idx_plan_ + n >= idx_map_.size())
            break;
          if (checkPebbleObstructed(idx_plan_ + n)) {   // just check current pebble
            ROS_INFO("[%s] pebble idx %d is obstructed! (look ahead = %d)", name_.c_str(), n, N_lookahead_);  
            return false;
          } 
        }
      }
    }

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
    bool linear_check = calcPoseDistance(decimated_global_plan_[decimated_global_plan_.size()-1], robot_pose) < xy_tolerance_;
    linear_check = linear_check || close_enough_;
    if (idx_plan_ == decimated_global_plan_.size()-1 && 
        linear_check && 
        fabs(th_ref) < th_tolerance_)
    {
      goal_reached_ = true;
    }
    else {
      goal_reached_ = false;
      close_enough_ = false;
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
    // reconfigure reload planner

    

    if (reconfg_inner_planner_.size() > 0) {
      ROS_INFO("[%s] New planner reconfigure request: %s", name_.c_str(), reconfg_inner_planner_.c_str());  
      if(loadPlanner(reconfg_inner_planner_, costmap_ptr_.get()))
        ROS_INFO("[%s] New planner reconfigure success: %s", name_.c_str(), reconfg_inner_planner_.c_str());
      else 
        ROS_WARN("[%s] New planner reconfigure failed: %s", name_.c_str(), reconfg_inner_planner_.c_str());
      reconfg_inner_planner_ = std::string("");
    }

    // set plan
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
    using VecPS = std::vector<geometry_msgs::PoseStamped>;
    auto f_path_dist = [&, this](const VecPS& path) -> double {
    double dist = 0.0;
    for (int i = 0; i < path.size()-1; i++)
      dist += calculateDistance(path[i].pose, path[i+1].pose);
    return dist;
    };

    double dist_new_path = f_path_dist(plan);

    if(dist_new_path<2.5)
    {
      geometry_msgs::Pose goal
      getNewGoal(dist_new_path, goal)
      VecPs new_plan_ ;
      int total_number_of_loop = std::hypot(goal.pose.position.x - robot_pose.pose.position.x,goal.pose.position.y - robot_pose.pose.position.y) / d_min_;
      double x_increment = (goal.pose.position.x - robot_pose.pose.position.x) / total_number_of_loop;
      double y_increment = (goal.pose.position.y - robot_pose.pose.position.y) / total_number_of_loop;
      ros::NodeHandle nh;
      for (int i = 0; i < total_number_of_loop; ++i) 
      {
        geometry_msgs::msg::PoseStamped pose;
        pose.pose.position.x = robot_pose.pose.position.x + x_increment * i;
        pose.pose.position.y = robot_pose.pose.position.y + y_increment * i;
        pose.pose.position.z = 0.0;
        pose.pose.orientation.x = 0.0;
        pose.pose.orientation.y = 0.0;
        pose.pose.orientation.z = 0.0;
        pose.pose.orientation.w = 1.0;
        pose.header.stamp = nh->now();
        pose.header.frame_id = "map";
        new_plan_.push_back(pose);
      }
      plan = new_plan_;
    }

    // decimate plan
    decimatePlan(plan, decimated_global_plan_, idx_map_);
    // ROS_INFO("decimated plan size %lu, idx map size %lu", decimated_global_plan_.size(), idx_map_.size());
    // for (int i = 0; i < idx_map_.size(); i++)
    // {
    //   ROS_INFO("%d: %lu", i, idx_map_[i]);
    // }

    // find index closest to robot position in plan
    idx_plan_ = findIdxAlongPlan(robot_pose, decimated_global_plan_, 1);

    // publish path, waypoint
    nav_msgs::Path path;
    path.header = decimated_global_plan_[0].header;
    path.poses = decimated_global_plan_;
    decimated_path_pub_.publish(path);
    waypoint_pub_.publish(decimated_global_plan_[idx_plan_]);
    // pid_.reset();

        // completion check
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

    // completion check
    double r, p, th_ref;    
    quaternionToRPY(goal_rframe.pose.orientation, r, p, th_ref);
    bool linear_check = calcPoseDistance(decimated_global_plan_[decimated_global_plan_.size()-1], robot_pose) < xy_tolerance_;
    linear_check = linear_check || close_enough_;
    if (idx_plan_ == decimated_global_plan_.size()-1 && 
        linear_check && 
        fabs(th_ref) < th_tolerance_){
        goal_reached_ = true;
    }
    else {
      goal_reached_ = false;
      close_enough_ = false;
    }

    return true;
  }

  void PebbleLocalPlanner::odometryCb(nav_msgs::Odometry msg)
  {
    latest_odom_ = msg;
  }

  void PebbleLocalPlanner::getNewGoal(int dist,geometry_msgs::Pose &pose_in_map)
  {
      
      geometry_msgs::TransformStamped transform =
          tf_buffer_.lookupTransform("map", "odom", ros::Time(0));
      geometry_msgs::Pose current_goal = latest_odom_.pose.pose
      current_goal.position.x = current_goal.position.x + dist
      tf2::doTransform(current_goal, pose_in_map, transform);
    }
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

    costmap_ptr_.reset(costmap_ros);

    loadPlanner(inner_planner_, costmap_ptr_.get());

    // planner_ptr_ = bgp_loader_.createInstance(inner_planner_);
    // planner_ptr_->initialize(bgp_loader_.getName(inner_planner_), costmap_ros);
    // ROS_INFO("[%s] Using inner_planner: %s", name_.c_str(), inner_planner_.c_str());

    // costmap_ptr_.reset(costmap_ros);
  }


  bool PebbleLocalPlanner::loadPlanner(const std::string& planner, costmap_2d::Costmap2DROS* costmap_ros)
  {
    boost::shared_ptr<nav_core::BaseGlobalPlanner> new_planner_ptr;
    try {
      new_planner_ptr = bgp_loader_.createInstance(planner);
      new_planner_ptr->initialize(bgp_loader_.getName(planner), costmap_ros);
    } catch (const pluginlib::PluginlibException& ex) {
      ROS_FATAL("[%s] Could not load planner %s", name_.c_str(), planner.c_str());
      return false;
    }
    planner_ptr_ = new_planner_ptr;
    inner_planner_ = planner;
    ROS_INFO("[%s] Using inner_planner: %s", name_.c_str(), inner_planner_.c_str());
    return true;
  }


  bool PebbleLocalPlanner::loadParams()
  {
    ros::NodeHandle nl("~"+name_);
    // std::string resolved = nl.resolveName("d_min");
    // ROS_INFO("resolved dmin %s", resolved.c_str());
    // planner related
    inner_planner_ = "global_planner/GlobalPlanner";
    if (nl.hasParam("inner_planner"))
      nl.getParam("inner_planner", inner_planner_);

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
    // pid_.setTolerances(0.25*d_min_, 0.75*th_tolerance_);

    // pid related
    kpl_ = 1.;
    kil_ = 0.;
    kdl_ = 0.;
    if (nl.hasParam("kp_linear"))
      nl.getParam("kp_linear", kpl_);

    if (nl.hasParam("ki_linear"))
      nl.getParam("ki_linear", kil_);

    if (nl.hasParam("kd_linear"))
      nl.getParam("kd_linear", kdl_);

    // pid_.setLinGains(kpl, kil, kdl);

    kpa_ = 1.;
    kia_ = 0.;
    kda_ = 0.;
    if (nl.hasParam("kp_angular"))
      nl.getParam("kp_angular", kpa_);

    if (nl.hasParam("ki_angular"))
      nl.getParam("ki_angular", kia_);

    if (nl.hasParam("kd_angular"))
      nl.getParam("kd_angular", kda_);

    // pid_.setAngGains(kpa, kia, kda);

    max_vx_ = 0.3;
    max_wz_ = 0.785;
    if (nl.hasParam("max_vel_x"))
      nl.getParam("max_vel_x", max_vx_);
    if (nl.hasParam("max_vel_theta"))
      nl.getParam("max_vel_theta", max_wz_);
    // pid_.setMaxVeloes(max_vx_, max_wz_);

    max_ax_ = 0.25;
    max_alphaz_ = 1.57;
    if (nl.hasParam("acc_lim_x"))
      nl.getParam("acc_lim_x", max_ax_);
    if (nl.hasParam("acc_lim_theta"))
      nl.getParam("acc_lim_theta", max_alphaz_);

    allow_reverse_ = false;
    if (nl.hasParam("allow_reverse"))
      nl.getParam("allow_reverse", allow_reverse_);
    // pid_.setAllowReverse(allow_reverse_);

    th_turn_ = M_PI/4.;
    if (nl.hasParam("th_turn"))
      nl.getParam("th_turn", th_turn_);
    // pid_.setTurnThresh(th_turn_);

    local_obsav_ = true;
    if (nl.hasParam("local_obstacle_avoidance"))
      nl.getParam("local_obstacle_avoidance", local_obsav_);

    th_reverse_ = 0.75 * M_PI;
    if (nl.hasParam("th_reverse"))
      nl.getParam("th_reverse", th_reverse_);

    N_lookahead_ = 2;
    if (nl.hasParam("N_lookahead"))
      nl.getParam("N_lookahead", N_lookahead_);

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
    // ROS_INFO("raw errors %5.2f, %5.2f", ex, eth);

    // if (ex < 0 && allow_reverse_)
    if (fabs(eth) > th_reverse_ && allow_reverse_)
    {
      eth = fmod(eth + M_PI, 2.*M_PI);
      if (eth < -M_PI)
        eth = 2.*M_PI + eth;
      else if (eth > M_PI)
        eth = -2.*M_PI + eth;
      reverse = true;
      // ROS_INFO("I should reverse");
    }

    // calculate speed references
    // modulate max vx if at last index
    double max_vx = max_vx_;
    if (idx_plan_ == decimated_global_plan_.size()-1)
    {
      // max_vx = sqrt(2.* max_ax_ * fabs(ex));
      max_vx = max_vx_ * fabs(ex)/d_min_;
      max_vx = std::min(max_vx_, max_vx);
      if (ex < xy_tolerance_)
      {
        eth = thref;
        close_enough_ = true;
      }
      // ROS_INFO("is final waypoint, max_vx %5.2f, eth %5.2f", max_vx, eth);
    }
    // ROS_INFO("errors %5.2f, %5.2f", ex, eth);

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
      if (close_enough_)
        vx = 0.;
    }

    // ROS_INFO("raw veloes %5.2f, %5.2f", vx, wz);
    
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

    // ROS_INFO("dt %5.2f, dvx/dt %5.2f, dwz/dt %5.2f", dt, dv/dt, dw/dt);
    // ROS_INFO("final veloes %5.2f, %5.2f", vx, wz);
    // ROS_INFO("---");

  }

  void PebbleLocalPlanner::dynConfigCb(pebble_local_planner::pebble_local_plannerConfig &config, uint32_t level)
  {
    ROS_INFO("[%s] new config!", name_.c_str());
    // planner ptr reconfigure done in setPlan()
    if(config.inner_planner != inner_planner_) {
      reconfg_inner_planner_ = config.inner_planner;
    }
    d_min_ = config.d_min;
    xy_tolerance_ = config.xy_goal_tolerance;
    th_tolerance_ = config.yaw_goal_tolerance;
    max_vx_ = config.max_vel_x;
    max_wz_ = config.max_vel_theta;
    max_ax_ = config.acc_lim_x;
    max_alphaz_ = config.acc_lim_theta;
    allow_reverse_ = config.allow_reverse;
    // ROS_INFO("allow reverse is now %d", allow_reverse_);
    th_turn_ = config.th_turn;
    local_obsav_ = config.local_obstacle_avoidance;
    th_reverse_ = config.th_reverse;
    N_lookahead_ = config.N_lookahead;
    kpl_ = config.kp_linear;  
    kil_ = config.ki_linear;
    kdl_ = config.kd_linear;
    kpa_ = config.kp_angular;
    kia_ = config.ki_angular;
    kda_ = config.kd_angular;
  }

  bool PebbleLocalPlanner::checkPebbleObstructed(int idx_pebble)   // copy
  {
    size_t jdx = idx_map_[idx_pebble];
    double wx = global_plan_[jdx].pose.position.x;
    double wy = global_plan_[jdx].pose.position.y;
    unsigned int mx, my;
    costmap_2d::Costmap2D* costmap = costmap_ptr_->getCostmap();
    if (costmap->worldToMap(wx, wy, mx, my)) {
      unsigned char cost_idx = costmap->getCost(mx, my);
      return cost_idx == costmap_2d::LETHAL_OBSTACLE || 
             cost_idx == costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
    }
    else {
      ROS_WARN("[%s] checkPebbleObstructed() failed to check costmap!", name_.c_str());
      return false;
    }
  }

  bool PebbleLocalPlanner::adjustPlanForObstacles()
  {
    costmap_2d::Costmap2D* costmap = costmap_ptr_->getCostmap();
    double wx, wy;
    unsigned int mx, my;
    unsigned char cost_idx;
    // ROS_INFO("eval obstacle, start at index %d", idx_plan_);
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
      wx = global_plan_[jdx].pose.position.x;
      wy = global_plan_[jdx].pose.position.y;
      if (costmap->worldToMap(wx, wy, mx, my))
      {
        cost_idx = costmap->getCost(mx, my);
        // ROS_INFO("eval obstacle jdx %lu, cost %d", jdx, (int) cost_idx);

        if (cost_idx != costmap_2d::LETHAL_OBSTACLE && cost_idx != costmap_2d::INSCRIBED_INFLATED_OBSTACLE)
        {
          // decide if we should continue
          // if the distance between global_plan pose to robot increases,
          // then we have passed the closest pose to robot 
          // and the backwards march is complete
          double drobot = calcPoseDistance(robot_pose, global_plan_[jdx]);
          if (drobot > prev_drobot)
          {
            // ROS_INFO("backwards march clear");
            return true; 
          }
          prev_drobot = drobot;
          // if we get to zero without finding obstruction, then all's clear
          // jdx is size_t remember, so it's unsigned, 
          // don't be clever and try to do a jdx < 0, you'll get Bad Time
          if (jdx == 0)
          {
            // ROS_INFO("backwards march clear by index zero");
            return true;
          }
          --jdx;
        }
        else
        {
          // ROS_INFO("caught obstacle during backwards march %d", (int) cost_idx);
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
              // find jdx for robot pose
              double drobot = calcPoseDistance(robot_pose, global_plan_[jdx]);
              while (drobot > prev_drobot)
              {
                --jdx;
                if (jdx < 0)
                {
                  jdx = 0;
                  break;
                }
                prev_drobot = drobot;
                drobot = calcPoseDistance(robot_pose, global_plan_[jdx]);
              }

              // erase global plan between robot pose to idx_plan_
              global_plan_.erase(global_plan_.begin()+jdx, global_plan_.begin()+idx_map_[idx_plan_]);

              // insert interplan between above
              global_plan_.insert(global_plan_.begin()+jdx, interplan.begin(), interplan.end());
              
              // offset idx_interplan
              for (int i = 0; i < idx_interplan.size(); i++)
              {
                idx_interplan[i] += jdx;
              }

              // offset the rest of the indices
              // this must be done before we manipulate idx_map_
              int idx_offset = jdx + interplan.size() - idx_map_[idx_plan_];
              for (int i = idx_plan_; i < idx_map_.size(); i++)
              {
                idx_map_[i] += idx_offset;
              }

              // insert idx_interplan to idx_map_
              idx_map_.insert(idx_map_.begin()+idx_plan_, idx_interplan.begin(), idx_interplan.end());

              nav_msgs::Path path;
              path.header = decimated_global_plan_[0].header;
              path.poses = decimated_global_plan_;
              decimated_path_pub_.publish(path);
              waypoint_pub_.publish(decimated_global_plan_[idx_plan_]);

              // DEBUG: print new idx_map_
              // ROS_INFO("global plan size %lu, decim plan size %lu, idx map size %lu",
              //          global_plan_.size(), decimated_global_plan_.size(), idx_map_.size());
              // for (int i = 0; i < idx_map_.size(); i++)
              //   ROS_INFO("%d: %lu", i, idx_map_[i]);
              
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
                if (cost_idx != costmap_2d::LETHAL_OBSTACLE && cost_idx != costmap_2d::INSCRIBED_INFLATED_OBSTACLE)
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
                    // ROS_INFO("avoid obstale; idx_plan %d, idx_free %lu, interplan %lu, decim %lu", 
                    //          idx_plan_, idx_free, interplan.size(), decim_interplan.size());

                    // remove blocked pebbles
                    decimated_global_plan_.erase(decimated_global_plan_.begin()+idx_plan_, decimated_global_plan_.begin()+idx_free);

                    // insert decimated interplan
                    decimated_global_plan_.insert(decimated_global_plan_.begin()+idx_plan_, decim_interplan.begin(), decim_interplan.end());

                    jdx = idx_map_[idx_plan_]; // we've advanced this forward, so reset it to reduce traversal
                    // find global plan index closest to robot pose
                    double drobot = calcPoseDistance(robot_pose, global_plan_[jdx]);
                    while (drobot > prev_drobot)
                    {
                      --jdx;
                      if (jdx < 0)
                      {
                        jdx = 0;
                        break;
                      }
                      prev_drobot = drobot;
                      drobot = calcPoseDistance(robot_pose, global_plan_[jdx]);
                    }

                    // erase global plan between robot pose to idx_free
                    global_plan_.erase(global_plan_.begin()+jdx, global_plan_.begin()+idx_map_[idx_free]);

                    // insert interplan between above
                    global_plan_.insert(global_plan_.begin()+jdx, interplan.begin(), interplan.end());
                    
                    // offset idx_interplan
                    for (int i = 0; i < idx_interplan.size(); i++)
                    {
                      idx_interplan[i] += jdx; //idx_map_[idx_plan_];
                    }

                    // offset the rest of the indices
                    // this must be done before we manipulate idx_map_
                    int idx_offset = jdx + interplan.size() - idx_map_[idx_free];
                    for (int i = idx_free; i < idx_map_.size(); i++)
                    {
                      idx_map_[i] += idx_offset;
                    }

                    // remove idx map at blocked pebbles
                    idx_map_.erase(idx_map_.begin()+idx_plan_, idx_map_.begin()+idx_free);

                    // insert idx_interplan to idx_map_
                    idx_map_.insert(idx_map_.begin()+idx_plan_, idx_interplan.begin(), idx_interplan.end());

                    nav_msgs::Path path;
                    path.header = decimated_global_plan_[0].header;
                    path.poses = decimated_global_plan_;
                    decimated_path_pub_.publish(path);
                    waypoint_pub_.publish(decimated_global_plan_[idx_plan_]);

                    // DEBUG: print new idx_map_
                    // ROS_INFO("global plan size %lu, decim plan size %lu, idx map size %lu",
                    //         global_plan_.size(), decimated_global_plan_.size(), idx_map_.size());
                    // for (int i = 0; i < idx_map_.size(); i++)
                    //   ROS_INFO("%d: %lu", i, idx_map_[i]);
                    
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
                // ROS_INFO("forwards march has index outside local costmap?");
                return false;
              }
            }
          }
        }
      }
      else
      {
        // ROS_INFO("backwards march has index outside local costmap?");
        return false;
      }
    }
  }

  bool PebbleLocalPlanner::planAheadForObstacles(int N)
  {
    int idx_test = std::min(idx_plan_+N, (int) decimated_global_plan_.size()-1);
    int pebble_fwd = idx_test;
    int pebble_bck = idx_test;
    int free_fwd = idx_test;
    int free_bck = -1;
    int jdx_f = idx_map_[idx_test];
    int jdx_b = idx_map_[idx_test];
    int robot_idx_global = 0; // index of global plan closest to robot pose
    unsigned int search_state = 0;

    costmap_2d::Costmap2D* costmap = costmap_ptr_->getCostmap();
    double wx, wy;
    unsigned int mx, my;
    unsigned char cost_jdx_f, cost_jdx_b;

    double prev_drobot = 1000.* N * d_min_;
    size_t jdx = idx_map_[idx_plan_];
    geometry_msgs::PoseStamped robot_pose;
    if (!getRobotPose(robot_pose))
    {
      ROS_INFO("can't get robot pose for obstacle checking");
      return false;
    }

    // search for free pebbles in horizon
    while (true)
    {
      // ROS_INFO("search state %u, free_fwd %d/%lu, free_bck %d/%lu",
      //          search_state, free_fwd, decimated_global_plan_.size(), free_bck, decimated_global_plan_.size());
      if (search_state == 0)
      {
        // no obstruction found yet, march both forward and backward indices back towards the robot
        wx = global_plan_[jdx_f].pose.position.x;
        wy = global_plan_[jdx_f].pose.position.y;
        if (costmap->worldToMap(wx, wy, mx, my))
        {
          cost_jdx_f = costmap->getCost(mx, my);
        }
        else
        {
          // ROS_INFO("search state 0; plan out of map bounds?");
          cost_jdx_f = costmap_2d::FREE_SPACE;
          // return false;
        }

        if (cost_jdx_f != costmap_2d::LETHAL_OBSTACLE &&
            cost_jdx_f != costmap_2d::INSCRIBED_INFLATED_OBSTACLE)
        {
          jdx_f--;
          jdx_b--;

          // check if we can bring free_fwd closer to robot
          if (pebble_fwd > 0 && jdx_f == idx_map_[pebble_fwd-1])
          {
            pebble_fwd--;
            pebble_bck = pebble_fwd;
            free_fwd = pebble_fwd;
            free_bck = free_fwd;
          }

          // check if we've run out of indices
          if (jdx_f < 0)
          {
            // ROS_INFO("state 0 backwards march reached head of plan");
            return true;
          }

          // check if we're at robot pose
          double drobot = calcPoseDistance(robot_pose, global_plan_[jdx_f]);
          if (drobot > prev_drobot)
          {
            // ROS_INFO("state 0 backwards march reached robot pose");
            return true;
          }
          
          prev_drobot = drobot;
        }
        else
        {
          // obstructed during backwards march, 
          // if this is the first check do forward-backward search (state 1)
          // if we already have a valid free_fwd just continue the backward search (state 2)
          if (jdx_f == idx_map_[idx_test])
          {
            if (pebble_fwd < decimated_global_plan_.size()-1)
            {
              pebble_fwd++;
              jdx_f = idx_map_[pebble_fwd];
            }
            else
            {
              ROS_INFO("final pebble is blocked");
              return false;
            }
            jdx_b--;
            if (jdx_b < 0)
            {
              free_bck = -1;
              break;
            }
            search_state = 1;
            // ROS_INFO("move to search state 1");
            continue;
          }
          else
          {
            jdx_b--;
            if (jdx_b < 0)
            {
              free_bck = -1;
              break;
            }
            search_state = 2;
            // ROS_INFO("move to search state 2");
            continue;
          }
        }
      }
      else if (search_state == 1)
      {
        // search forward and backward
        // it is not possible to enter this state with pebble_bck equals 0
        wx = global_plan_[jdx_b].pose.position.x;
        wy = global_plan_[jdx_b].pose.position.y;
        if (costmap->worldToMap(wx, wy, mx, my))
        {
          cost_jdx_b = costmap->getCost(mx, my);
        }
        else
        {
          // ROS_INFO("search state 1; backwards; plan out of map bounds?");
          cost_jdx_b = costmap_2d::FREE_SPACE;
          // return false;
        }

        // forward march may get us outside the local costmap,
        // in that case, declare the cell free
        // if it is occupied after all, we'll find out when we approach
        wx = global_plan_[jdx_f].pose.position.x;
        wy = global_plan_[jdx_f].pose.position.y;
        if (costmap->worldToMap(wx, wy, mx, my))
        {
          cost_jdx_f = costmap->getCost(mx, my);
        }
        else
        {
          // ROS_INFO("search state 1; forwards; plan out of map bounds?");
          cost_jdx_f = costmap_2d::FREE_SPACE;
          // return false;
        }

        // evaluate forward index
        if (cost_jdx_f != costmap_2d::LETHAL_OBSTACLE &&
            cost_jdx_f != costmap_2d::INSCRIBED_INFLATED_OBSTACLE)
        {
          free_fwd = pebble_fwd;
          search_state = 2;
          // ROS_INFO("move to search state 2");
        }
        else
        {
          pebble_fwd++;
          if (pebble_fwd >= decimated_global_plan_.size())
          {
            ROS_INFO("search state 1 all forward pebbles are blocked %d, %lu", pebble_fwd, decimated_global_plan_.size());
            return false;
          }
          jdx_f = idx_map_[pebble_fwd];
        }

        // evaluate bacward index
        if (cost_jdx_b != costmap_2d::LETHAL_OBSTACLE &&
            cost_jdx_b != costmap_2d::INSCRIBED_INFLATED_OBSTACLE)
        {
          // ROS_INFO("search state 1, free cell pebble %d, free %d", pebble_bck, free_bck);
          // check if we have to move free_bck closer to robot
          if (pebble_bck > 0 && free_bck < 0)
          {
            // pebble_bck--;
            free_bck = pebble_bck-1;
          }
        }
        else
        {
          // reset free_bck, because the path isn't clear between the robot and the previously found free_bck
          free_bck = -1;
        }

        // update index and pebble
        jdx_b--;
        // ROS_INFO("pebble progress check jdx %d, pebble %d, map %d, prev map %d",
        //          jdx_b, pebble_bck, idx_map_[pebble_bck-1], idx_map_[pebble_bck]);
        if (pebble_bck > 0 && jdx_b == (int)idx_map_[pebble_bck-1])
        {
          pebble_bck--;
          // ROS_INFO("search state 1 new pebble bck %d, jdx %d", pebble_bck, jdx_b);
        }

        // check if we've run out of global plan
        if (jdx_b < 0)
        {
          free_bck = -1;
          robot_idx_global = 0;
          if (free_fwd < 0)
          {
            pebble_fwd++;
            // ROS_INFO("move to search state 3");
            search_state = 3;
          }
          else
          {
            break;
          }
        }

        // check if we've reached robot pose
        double drobot = calcPoseDistance(robot_pose, global_plan_[jdx_b]);
        if (drobot > prev_drobot)
        {
          robot_idx_global = jdx_b;
          if (free_fwd < 0) // do we still need to find free_fwd
          {
            pebble_fwd++;
            // ROS_INFO("move to search state 3");
            search_state = 3;
          }
          else // or can we proceed to planning?
          {
            break;
          }
        }
        prev_drobot = drobot;
      }
      else if (search_state == 2)
      {
        // stop forward, search backward
        wx = global_plan_[jdx_b].pose.position.x;
        wy = global_plan_[jdx_b].pose.position.y;
        if (costmap->worldToMap(wx, wy, mx, my))
        {
          cost_jdx_b = costmap->getCost(mx, my);
        }
        else
        {
          // ROS_INFO("search state 2; plan out of map bounds?");
          cost_jdx_b = costmap_2d::FREE_SPACE;
          // return false;
        }
        // ROS_INFO("search state 2, jdx_b %d, cost %d, pebble %d", jdx_b, (int)cost_jdx_b, pebble_bck);

        if (cost_jdx_b != costmap_2d::LETHAL_OBSTACLE &&
            cost_jdx_b != costmap_2d::INSCRIBED_INFLATED_OBSTACLE)
        {
          // check if we have to move free_bck closer to the robot
          // ROS_INFO("search state 2, free cell pebble %d, free %d", pebble_bck, free_bck);
          if (pebble_bck > 0 && free_bck < 0)
          {
            // pebble_bck--;
            free_bck = pebble_bck;
          }
        }
        else
        {
          // reset free_bck
          free_bck = -1;
        }

        // update idx
        jdx_b--;
        // ROS_INFO("pebble progress check jdx %d, pebble %d, map %d, prev map %d",
        //          jdx_b, pebble_bck, idx_map_[pebble_bck-1], idx_map_[pebble_bck]);
        if (pebble_bck > 0 && jdx_b == (int)idx_map_[pebble_bck-1])
        {
          pebble_bck--;
          // ROS_INFO("search state 2 new pebble bck %d, jdx %d", pebble_bck, jdx_b);
        }

        // check if we've run out of global plan
        if (jdx_b < 0)
        {
          free_bck = -1;
          robot_idx_global = 0;
          break;
        }

        // check if we've reached robot pose
        double drobot = calcPoseDistance(robot_pose, global_plan_[jdx_b]);
        if (drobot > prev_drobot)
        {
          // ROS_INFO("search state 2; reached robot pose");
          if (free_bck < 0)
          {
            robot_idx_global = jdx_b;
          }
          break; // we would have found free_bck or it would be -1, either is usable for planning
        }
        prev_drobot = drobot;

      }
      else if (search_state == 3)
      {
        // stop backward, search forward
        jdx_f = idx_map_[pebble_fwd];
        wx = global_plan_[jdx_f].pose.position.x;
        wy = global_plan_[jdx_f].pose.position.y;
        if (costmap->worldToMap(wx, wy, mx, my))
        {
          cost_jdx_f = costmap->getCost(mx, my);
        }
        else
        {
          // ROS_INFO("search state 3; plan out of map bounds?");
          cost_jdx_f = costmap_2d::FREE_SPACE;
          // return false;
        }

        // evaluate forward index
        if (cost_jdx_f != costmap_2d::LETHAL_OBSTACLE &&
            cost_jdx_f != costmap_2d::INSCRIBED_INFLATED_OBSTACLE)
        {
          // found a free pebble, move on to planning
          free_fwd = pebble_fwd;
          break;
        }
        else
        {
          pebble_fwd++;
          if (pebble_fwd >= decimated_global_plan_.size())
          {
            ROS_INFO("search state 3, no free forward pebble");
            return false;
          }
        }
      }
      else // invalid search state
      {
        return false;
      }
    }

    // ROS_INFO("planning obsav test %d bck %d/%d, fwd %d/%d", idx_test, free_bck, pebble_bck, free_fwd, pebble_fwd);
    
    // plan between free_fwd and free_bck
    // if the horizon is clear, you won't reach here
    free_bck -= 3;
    geometry_msgs::PoseStamped pose_start;
    if (free_bck < 0)
    {
      pose_start = robot_pose;
    }
    else
    {
      pose_start = decimated_global_plan_[std::max(free_bck, pebble_bck)];
    }
  
    free_fwd = std::min(free_fwd+3, (int)decimated_global_plan_.size()-1);
    std::vector<geometry_msgs::PoseStamped> interplan;
    if (planner_ptr_->makePlan(pose_start, decimated_global_plan_[free_fwd], interplan))
    {
      std::vector<size_t> decim_inter_idx;
      std::vector<geometry_msgs::PoseStamped> decim_interplan;
      int len_decim_interplan = decimatePlan(interplan, decim_interplan, decim_inter_idx);

      // ROS_INFO("got interplan, size %lu, decimated %lu", interplan.size(), decim_interplan.size());

      // insert interplan into global plan
      int i, j;
      i = robot_idx_global;
      if (free_bck >= 0)
      {
        i = idx_map_[free_bck];
      }
      j = idx_map_[free_fwd];
      global_plan_.erase(global_plan_.begin()+i, global_plan_.begin()+j);
      // ROS_INFO("global erase OK %d, %d", i, j);
      global_plan_.insert(global_plan_.begin()+i, interplan.begin(), interplan.end());
      // ROS_INFO("global insert OK");

      // insert decim interplan to decim global plan
      decimated_global_plan_.erase(decimated_global_plan_.begin()+std::max(pebble_bck, free_bck), 
                                   decimated_global_plan_.begin()+free_fwd);
      // ROS_INFO("decim erase OK");

      decimated_global_plan_.insert(decimated_global_plan_.begin()+std::max(pebble_bck, free_bck),
                                    decim_interplan.begin(), decim_interplan.end());
      // ROS_INFO("decim insert OK");
      if (idx_plan_ < free_bck)
        idx_plan_ = free_bck;
      else if (free_bck < 0)
        idx_plan_ = pebble_bck;
      
      nav_msgs::Path new_path;
      new_path.header = decimated_global_plan_[0].header;
      new_path.poses = decimated_global_plan_;
      decimated_path_pub_.publish(new_path);
      waypoint_pub_.publish(decimated_global_plan_[idx_plan_]);
      // ROS_INFO("publish OK");

      // update idx_map_
      int head_idx = std::max((int)idx_map_[std::max(pebble_bck, free_bck)], robot_idx_global);
      int tail_idx = idx_map_[free_fwd]; // must get this before idx_map_ is manipulated
      for (int ii = 0; ii < decim_inter_idx.size(); ii++)
      {
        decim_inter_idx[ii] += head_idx;
      }
      // ROS_INFO("inter offset OK");

      idx_map_.erase(idx_map_.begin()+std::max(pebble_bck, free_bck), idx_map_.begin()+free_fwd);
      // ROS_INFO("idx map erase OK");
      idx_map_.insert(idx_map_.begin()+std::max(pebble_bck, free_bck), 
                      decim_inter_idx.begin(), decim_inter_idx.end());
      // ROS_INFO("idx map insert OK");

      int offset = head_idx + interplan.size() - tail_idx;
      for (int ii = std::max(pebble_bck, free_bck) + decim_inter_idx.size();
           ii < idx_map_.size(); ii++)
      {
        idx_map_[ii] += offset;
      }
      // ROS_INFO("rest offset OK");

      // ROS_INFO("post-offset idx map, must be monotonic increase");
      // for (int i = 0; i < idx_map_.size(); i++)
      // {
      //   ROS_INFO("%d: %lu", i, idx_map_[i]);
      // }
    }
    else
    {
      ROS_INFO("failed to make obsav plan between free pebbles");
      return false;
    }

    return true;
  }
}
