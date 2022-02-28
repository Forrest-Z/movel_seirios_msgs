#include <planner_adjuster/planner_adjuster.hpp>
#include <movel_hasp_vendor/license.h>

double quaternionToYaw(geometry_msgs::Quaternion q)
{
  double aa = 2.0 * (q.w * q.z + q.y * q.x);
  double bb = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  double theta = atan2(aa, bb);
  return theta;
}

PlannerAdjuster::PlannerAdjuster()
  : nh_("~"), tf_ear_(tf_buffer_), has_goal_(false), controller_stage_(0), dist_feasible(true)
{
  if (!getParams())
  {
    ROS_ERROR("Parameter error. Try again");
  }
  setupTopics();
  ROS_INFO("wait for valid time");
  ros::Time::waitForValid();
  t_prev_ = ros::Time::now();

  ROS_INFO("go");
}

bool PlannerAdjuster::getParams()
{
  ros::param::param<bool>("~dock_backwards", dock_backwards_, false);
  std::vector<double> angle_gains_init, angle_gains_final, dist_gains;
  if (nh_.hasParam("angle_gains_init"))
    nh_.getParam("angle_gains_init", angle_gains_init);
  else
    return false;

  if (nh_.hasParam("angle_gains_final"))
    nh_.getParam("angle_gains_final", angle_gains_final);
  else
    return false;

  if (nh_.hasParam("dist_gains"))
    nh_.getParam("dist_gains", dist_gains);
  else
    return false;

  if (nh_.hasParam("dist_feasible"))
    nh_.getParam("dist_feasible", dist_feasible_);
  else
    return false;

  angle_PID_init.setGains(angle_gains_init[0], angle_gains_init[1], angle_gains_init[2]);
  angle_PID_final.setGains(angle_gains_final[0], angle_gains_final[1], angle_gains_final[2]);
  dist_PID.setGains(dist_gains[0], dist_gains[1], dist_gains[2]);

  // ROS_INFO("angle_gains %5.2f, %5.2f, %5.2f", angle_gains[0], angle_gains[1], angle_gains[2]);
  // ROS_INFO("dist_gains %5.2f, %5.2f, %5.2f", dist_gains[0], dist_gains[1], dist_gains[2]);

  if (nh_.hasParam("angle_tolerance"))
  {
    nh_.getParam("angle_tolerance", angle_tolerance_);
  }
  else
  {
    return false;
  }
  // ROS_INFO("angle_tolerance %5.2f", angle_tolerance_);

  if (nh_.hasParam("dist_tolerance"))
  {
    nh_.getParam("dist_tolerance", dist_tolerance_);
  }
  else
  {
    return false;
  }
  // ROS_INFO("dist_tolerance %5.2f", dist_tolerance_);

  if (nh_.hasParam("max_linear_speed"))
    nh_.getParam("max_linear_speed", max_linear_speed_);
  else
    return false;

  if (nh_.hasParam("max_angular_speed"))
    nh_.getParam("max_angular_speed", max_angular_speed_);
  else
    return false;

  if (nh_.hasParam("min_angular_speed"))
    nh_.getParam("min_angular_speed", min_angular_speed_);
  else
    return false;

  if (nh_.hasParam("odom_topic"))
    nh_.getParam("odom_topic", odom_topic_);
  else
    return false;

  return true;
}

bool PlannerAdjuster::setupTopics()
{
  cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
  reached_pub_ = nh_.advertise<std_msgs::Bool>("/goal/status", 1);
  goal_sub_ = nh_.subscribe("/pid_goal", 1, &PlannerAdjuster::goalCb, this);
  odom_sub_ = nh_.subscribe("/" + odom_topic_, 1, &PlannerAdjuster::odometryCb, this);
  stop_now_sub_ = nh_.subscribe("/stop_now", 1, &PlannerAdjuster::stopNowCb, this);

  return true;
}

void PlannerAdjuster::odometryCb(nav_msgs::Odometry msg)
{
  latest_odom_ = msg;
  if (has_goal_)
  {
    geometry_msgs::TransformStamped transform =
        tf_buffer_.lookupTransform(current_goal_.header.frame_id, msg.header.frame_id, ros::Time(0));
    //std::cout << "goal frame " << current_goal_.header.frame_id << "odom frame " << msg.header.frame_id << "transform" << transform << std::endl;
    geometry_msgs::Pose pose_in_map;
    tf2::doTransform(msg.pose.pose, pose_in_map, transform);
    latest_pose_ = pose_in_map;
    // ROS_INFO("w in odom %5.2f, w in map %5.2f", msg.pose.pose.orientation.w, latest_pose_.orientation.w);
    if (calcDist(latest_pose_, current_goal_.pose) > dist_feasible_)
    {
      has_goal_ = false;
      controller_stage_ = 0;
      geometry_msgs::Twist cmd_vel;
      cmd_vel_pub_.publish(cmd_vel);
      ROS_INFO("Not Feasible");
    }
    else if (!stop_check)
    {
      doControl(pose_in_map);
    }
  }
}

void PlannerAdjuster::goalCb(const geometry_msgs::PoseStamped msg)
{
  ROS_INFO("[planner_adjuster] new goal! %5.2f, %5.2f", msg.pose.position.x, msg.pose.position.y);
  if (controller_stage_ >= 1){
    ROS_INFO("[planner_adjuster] not changing goal at stage %d. Stop me first.", controller_stage_);
    return;}
  current_goal_ = msg;
  has_goal_ = true;
  stop_check = false;
  t_prev_ = ros::Time::now();

  geometry_msgs::TransformStamped transform =
      tf_buffer_.lookupTransform(current_goal_.header.frame_id, latest_odom_.header.frame_id, ros::Time(0));
  tf2::doTransform(latest_odom_.pose.pose, latest_pose_, transform);
  if (calcDist(latest_pose_, current_goal_.pose) > dist_feasible_)
    dist_feasible = false;
  //controller_stage_ = 0;
  // double theta = 2.0 * acos(latest_pose_.orientation.w);
  double theta = quaternionToYaw(latest_pose_.orientation);
  double dx_aux, dy_aux, dth_aux;
  
  dy_aux = current_goal_.pose.position.y - latest_pose_.position.y;
  dx_aux = current_goal_.pose.position.x - latest_pose_.position.x;
  dth_aux = atan2(dy_aux, dx_aux);
  
  if (dock_backwards_){dth_aux += M_PI;}
  if (dth_aux > M_PI){dth_aux -= 2*M_PI;}
  double theta_aux = dth_aux;

  // ROS_INFO("th %5.2f, dth %5.2f, th_aux %5.2f, dy %5.2f, dx %5.2f",
  //    theta, dth_aux, theta_aux, dy_aux, dx_aux);

  angle_PID_init.reset();
  angle_PID_init.setRef(theta_aux);
  angle_PID_final.reset();
  angle_PID_final.setRef(theta_aux);
  return;
}

double PlannerAdjuster::calcDist(geometry_msgs::Pose a, geometry_msgs::Pose b)
{
  double dx, dy;
  dx = a.position.x - b.position.x;
  dy = a.position.y - b.position.y;
  return sqrt(dx * dx + dy * dy);
}

void PlannerAdjuster::doControl(geometry_msgs::Pose current_pose)
{
  double dx = calcDist(current_pose, current_goal_.pose);
  double dt = (ros::Time::now() - t_prev_).toSec();
  t_prev_ = ros::Time::now();

  double u_th = 0.0;
  double u_x = 0.0;

  double theta = quaternionToYaw(latest_pose_.orientation);

  // check stage progression
  if (controller_stage_ == 0)
  {
    double dtheta = fmod((angle_PID_init.getRef() - theta + 2 * M_PI), 2 * M_PI);
    if (dtheta > M_PI)
    {
      dtheta = 2 * M_PI - dtheta;
    }
    // ROS_INFO("stage %d, error %5.2f, tolerance %5.2f", controller_stage_, dtheta, angle_tolerance_);
    if (fabs(dtheta) < angle_tolerance_)
    {
      ROS_INFO("[planner_adjuster] stage 0 to 1");
      controller_stage_ = 1;
      dist_PID.reset();
      dist_PID.setRef(0.0);
      // angle_PID.reset();
      // angle_PID.setRef(0.0);
    }
  }
  else if (controller_stage_ == 1)
  {
    // ROS_INFO("stage %d, error %5.2f, tolerance %5.2f", controller_stage_, dx, dist_tolerance_);
    double dx_aux, dy_aux, dth_aux;
    dy_aux = current_goal_.pose.position.y - latest_pose_.position.y;
    dx_aux = current_goal_.pose.position.x - latest_pose_.position.x;
    dth_aux = atan2(dy_aux, dx_aux);
    if (dock_backwards_){
      dth_aux += M_PI; 
        if (dth_aux > M_PI){ 
          dth_aux -= 2*M_PI;}
    }
    double dtheta = fmod((angle_PID_init.getRef() - dth_aux + 2 * M_PI), 2 * M_PI);
    if (dtheta > M_PI)
    {
      dtheta = 2 * M_PI - dtheta;
    }
    // ROS_INFO("dth_aux %f,pid_ref %f , theta %f", fabs(dtheta), angle_PID_init.getRef(), dth_aux);
    if (fabs(dtheta) > 6 * angle_tolerance_)
    {
      ROS_INFO("[planner_adjuster] stage 1 to 0, dtheta %5.2f, tolerance %5.2f", dtheta, angle_tolerance_);
      controller_stage_ = 0;
      angle_PID_init.reset();
      angle_PID_init.setRef(dth_aux);
    }

    if (dx < dist_tolerance_)
    {
      ROS_INFO("[planner_adjuster] stage 1 to 2");
      controller_stage_ = 2;
      angle_PID_final.reset();
      // double theta_goal = 2.0 * acos(current_goal_.pose.orientation.w);
      double theta_goal = quaternionToYaw(current_goal_.pose.orientation);
      angle_PID_final.setRef(theta_goal);
    }
  }
  else if (controller_stage_ == 2)
  {
    // double theta_goal = 2.0 * acos(current_goal_.pose.orientation.w);
    double theta_goal = quaternionToYaw(current_goal_.pose.orientation);
    double dtheta = fmod((theta_goal - theta + 2 * M_PI), 2 * M_PI);
    if (dtheta > M_PI)
      dtheta = 2 * M_PI - dtheta;
    // ROS_INFO("stage %d, error %5.2f, tolerance %5.2f", controller_stage_, dtheta, angle_tolerance_);
    if (dx > 6 * dist_tolerance_)
    {
      ROS_INFO("[planner_adjuster] stage 2 to 0");
      double dx_aux, dy_aux, dth_aux;
      dy_aux = current_goal_.pose.position.y - latest_pose_.position.y;
      dx_aux = current_goal_.pose.position.x - latest_pose_.position.x;
      dth_aux = atan2(dy_aux, dx_aux);
      if (dock_backwards_) {
        dth_aux += M_PI;
          if (dth_aux > M_PI) 
            {dth_aux -= 2*M_PI;}
      }
      angle_PID_init.reset();
      angle_PID_init.setRef(dth_aux);
      controller_stage_ = 0;
      dist_PID.reset();
      dist_PID.setRef(0.0);
    }

    if (fabs(dtheta) < angle_tolerance_)
    {
      has_goal_ = false;
      controller_stage_ = 0;
      geometry_msgs::Twist cmd_vel;
      cmd_vel_pub_.publish(cmd_vel);
      std_msgs::Bool reached;
      reached.data = true;
      reached_pub_.publish(reached);
      ROS_INFO("[planner_adjuster] REACH REACH REACH");
      return;
    }
  }

  // calculate control input based on stage
  if (controller_stage_ == 0)
  {
    if (fabs(angle_PID_init.getRef() - theta) > M_PI)
    {
      if (theta > 0)
        theta -= 2 * M_PI;
      else
        theta += 2 * M_PI;
    }
    u_th = angle_PID_init.update(theta, dt);
    //ROS_INFO("stage %d, goal %5.2f, state %5.2f", controller_stage_, angle_PID_init.getRef(), theta);
  }
  else if (controller_stage_ == 1)
  {
    u_x = dist_PID.update(-dx, dt);
    if (fabs(angle_PID_init.getRef() - theta) > M_PI)
    {
      if (theta > 0)
        theta -= 2 * M_PI;
      else
        theta += 2 * M_PI;
    }
    // double dx_aux, dy_aux;
    // dx_aux = current_goal_.pose.position.x - latest_pose_.position.x;
    // dy_aux = current_goal_.pose.position.y - latest_pose_.position.y;
    // double dtheta = atan2(dy_aux, dx_aux);
    // u_th = angle_PID.update(-dtheta, dt);
    u_th = angle_PID_init.update(theta, dt);
    //ROS_INFO("stage %d, dist goal %5.2f, dx %5.2f, angle goal %5.2f,theta %5.2f", controller_stage_, dist_PID.getRef(), dx, angle_PID_init.getRef(), theta);
  }
  else if (controller_stage_ == 2)
  {
    if (fabs(angle_PID_final.getRef() - theta) > M_PI)
    {
      if (theta > 0)
        theta -= 2 * M_PI;
      else
        theta += 2 * M_PI;
    }
    u_th = angle_PID_final.update(theta, dt);
    //ROS_INFO("stage %d, goal %5.2f, state %5.2f", controller_stage_, angle_PID_final.getRef(), theta);
  }

  if(isnan(u_x) || isnan(u_th))
  {
    has_goal_ = false;
    controller_stage_ = 0;
    geometry_msgs::Twist cmd_vel;
    cmd_vel_pub_.publish(cmd_vel);
    std_msgs::Bool reached;
    reached.data = false;
    reached_pub_.publish(reached);
    ROS_INFO("[planner_adjuster] NaN value in velocity command. Stopping.");
    return;
  }

  // Apply max linear speed limit
  if(u_x < 0)
    u_x = std::max(u_x, -max_linear_speed_);
  else
    u_x = std::min(u_x, max_linear_speed_);

  // Apply max & min angular speed limits
  if(u_th < 0)
  {
    u_th = std::max(u_th, -max_angular_speed_);
    u_th = std::min(u_th, -min_angular_speed_);
  }
  else
  {
    u_th = std::min(u_th, max_angular_speed_);
    u_th = std::max(u_th, min_angular_speed_);
  }

  // Publish velocity command
  geometry_msgs::Twist cmd_vel;
  cmd_vel.angular.z = u_th;
  if (!dock_backwards_)
    cmd_vel.linear.x = u_x;
  else if (dock_backwards_)
      cmd_vel.linear.x = -u_x;
  cmd_vel_pub_.publish(cmd_vel);
}

void PlannerAdjuster::stopNowCb(const std_msgs::Bool msg)
{
  if (msg.data)
  {
    stop_check = true;
    geometry_msgs::Twist cmd_vel;
    cmd_vel_pub_.publish(cmd_vel);
    has_goal_ = false;
    controller_stage_ = 0;
    ROS_INFO("Stop Now");
  }
}

int main(int argc, char** argv)
{
#ifdef MOVEL_LICENSE
  MovelLicense ml;
  if (!ml.login())
    return 1;
#endif

  ros::init(argc, argv, "planner_adjuster");

  PlannerAdjuster planner_adjuster;
  ros::spin();

#ifdef MOVEL_LICENSE
  ml.logout();
#endif

  return 0;
}
