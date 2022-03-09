#include <pallet_detection/pallet_docking.h>
#include <movel_hasp_vendor/license.h>

PalletDocking::PalletDocking()
  : nh_private_("~"), run_(false), init_(true), odom_received_(false), status_received_(false)
{
  initialize();
}

void PalletDocking::initialize()
{
  if (!loadParams())
  {
    ROS_FATAL("[pallet_docking] Error during parameter loading. Shutting down.");
    return;
  }
  ROS_INFO("[pallet_docking] All parameters loaded. Launching.");

  setupTopics();

  ros::spin();
}

// Load ROS params
bool PalletDocking::loadParams()
{
  ros_utils::ParamLoader loader(nh_private_);
  loader.get_required("loop_rate", loop_rate_);
  loader.get_required("docking_speed", docking_speed_);
  loader.get_required("docking_distance", docking_distance_);
  loader.get_required("xy_tolerance", xy_tolerance_);
  loader.get_required("yaw_tolerance", yaw_tolerance_);
  loader.get_required("backward", backward_);
  loader.get_required("undock", undock_);
  loader.get_required("use_move_base", use_move_base_);
  loader.get_required("retry", retry_);
  loader.get_required("retry_undocking_distance", retry_undocking_distance_);
  return loader.params_valid();
}

// Set up callbacks
void PalletDocking::setupTopics()
{
  vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel_mux/autonomous", 1);
  success_pub_ = nh_private_.advertise<std_msgs::Bool>("success", 1);
  odom_sub_ = nh_.subscribe("/odom", 19, &PalletDocking::odomCb, this);
  goal_sub_ = nh_.subscribe("/pid_goal", 10, &PalletDocking::goalCb, this);
  start_sub_ = nh_.subscribe("/goal/status", 10, &PalletDocking::startCb, this);
  current_goal_sub_ = nh_.subscribe("/pallet_detection/current_goal", 10, &PalletDocking::currentGoalCb, this);
  timer_ = nh_.createTimer(ros::Duration(1.0 / loop_rate_), &PalletDocking::timerCallback, this);
  mb_sub_ = nh_.subscribe("/move_base/result", 1, &PalletDocking::mbCallback, this);

  if(use_move_base_)
    status_pub_ = nh_.advertise<std_msgs::Bool>("/goal/status", 1);
}

// Get odom data
void PalletDocking::odomCb(nav_msgs::Odometry odom)
{
  current_odom_ = odom;
  odom_received_ = true;
}

// Goal from initial docking
void PalletDocking::goalCb(geometry_msgs::PoseStamped goal)
{
  current_goal_ = goal;
}

// Real-time goal update during docking
void PalletDocking::currentGoalCb(geometry_msgs::PoseStamped goal)
{
  current_goal_ = goal;
}

// Trigger final stage docking after initial stage is complete (with planner_adjuster)
void PalletDocking::startCb(std_msgs::Bool start)
{
  if(!use_move_base_)
  {
    if(status_received_)
    {
      run_ = true;
      return;
    }
    status_received_ = true;
  }
  //next_stage_ = true;
}

// Trigger final stage docking after initial stage is complete (with move_base)
void PalletDocking::mbCallback(const move_base_msgs::MoveBaseActionResult::ConstPtr &msg)
{
  std_msgs::Bool status;

  if(use_move_base_ && msg->status.status == 3)
  {
    status.data = true;
    if(status_received_ && ros::Time::now().toSec() - start_time_.toSec() > 0.5)
    {
      run_ = true;
      status_pub_.publish(status);
      return;
    }
    status_received_ = true;
    start_time_ = ros::Time::now();
    status_pub_.publish(status);
  }
  else if(use_move_base_ && (msg->status.status != 0 && msg->status.status != 1 && msg->status.status != 2))
  {
    status.data = false;
    status_pub_.publish(status);
  }
}

// Main loop
void PalletDocking::timerCallback(const ros::TimerEvent& e)
{
  // Dock
  if(run_ && odom_received_)
  {
    // Record odom and docking goal before starting docking
    if(init_)
    {
      final_goal_ = current_goal_;
      //ros::Duration(3.0).sleep();
      ROS_INFO("[pallet_docking] Start docking");
      initial_odom_ = current_odom_;
      init_ = false;
    }
    
    // Stop when goal deviation exceeds threshold
    if(calcDistance(final_goal_.pose, current_goal_.pose) > xy_tolerance_ || calcAngleDiff(final_goal_.pose, current_goal_.pose) > yaw_tolerance_)
    {
      geometry_msgs::Twist stop;
      vel_pub_.publish(stop);
      run_ = false;
      init_ = true;
      std_msgs::Bool fail;
      fail.data = false;
      success_pub_.publish(fail);
      //retry_ = true;
      //distance_ = calcDistance(initial_odom_.pose.pose, current_odom_.pose.pose) + 0.5;
      return;
    }
    
    // Publish velocity command for docking
    if(calcDistance(initial_odom_.pose.pose, current_odom_.pose.pose) < docking_distance_)
    {
      geometry_msgs::Twist vel;
      if(!backward_)
        vel.linear.x = docking_speed_;
      else
        vel.linear.x = -docking_speed_;
      vel_pub_.publish(vel);      
    }

    // Stop when docking completes
    else
    {
      geometry_msgs::Twist stop;
      vel_pub_.publish(stop);
      run_ = false;
      init_ = true;
      std_msgs::Bool succeed;
      succeed.data = true;
      success_pub_.publish(succeed);
    }
  }
  /*else if(retry_ && odom_received_)
  {
    if(init_)
    {
      initial_odom_ = current_odom_;
      init_ = false;
    }

    if(calcDistance(initial_odom_.pose.pose, current_odom_.pose.pose) < distance_)
    {
      geometry_msgs::Twist vel;
      if(!backward_)
        vel.linear.x = -docking_speed_;
      else
        vel.linear.x = docking_speed_;
      vel_pub_.publish(vel);      
    }
    else
    {
      geometry_msgs::Twist stop;
      vel_pub_.publish(stop);
      retry_ = false;
      init_ = true;
      std_msgs::Bool fail;
      fail.data = false;
      success_pub_.publish(fail);
    }*/

    /*if(calcDistance(initial_odom_.pose.pose, current_odom_.pose.pose) > 0.05)
    {
      geometry_msgs::Twist vel;
      if(!backward_)
        vel.linear.x = -docking_speed_;
      else
        vel.linear.x = docking_speed_;
      vel_pub_.publish(vel);      
    }
    else
    {
      geometry_msgs::Twist stop;
      vel_pub_.publish(stop);
      retry_ = false;
      init_ = true;
      std_msgs::Bool fail;
      fail.data = false;
      success_pub_.publish(fail);
    }*/
  //}

  // Undock
  else if(undock_ && odom_received_)
  {
    // Record odom before undocking
    if(init_)
    {
      ROS_INFO("[pallet_docking] Start undocking");
      initial_odom_ = current_odom_;
      init_ = false;
    }

    // Select undocking distance (retry has longer undocking distance)
    double distance;
    if(!retry_)
      distance = docking_distance_;
    else
      distance = retry_undocking_distance_;

    // Publish velocity command for undocking
    if(calcDistance(initial_odom_.pose.pose, current_odom_.pose.pose) < distance)
    {
      geometry_msgs::Twist vel;
      if(!backward_)
        vel.linear.x = -docking_speed_;
      else
        vel.linear.x = docking_speed_;
      vel_pub_.publish(vel);      
    }

    // Stop when undocking completes
    else
    {
      geometry_msgs::Twist stop;
      vel_pub_.publish(stop);
      undock_ = false;
      init_ = true;
      std_msgs::Bool succeed;
      succeed.data = true;
      success_pub_.publish(succeed);
    }
  }
}

double PalletDocking::calcDistance(geometry_msgs::Pose pose1, geometry_msgs::Pose pose2)
{
  double dx = pose1.position.x - pose2.position.x;
  double dy = pose1.position.y - pose2.position.y;
  return sqrt(dx * dx + dy * dy);
}

double PalletDocking::calcAngleDiff(geometry_msgs::Pose init_pose, geometry_msgs::Pose target_pose)
{
  tf::Quaternion q1(init_pose.orientation.x, init_pose.orientation.y,
                    init_pose.orientation.z, init_pose.orientation.w),
                 q2(target_pose.orientation.x, target_pose.orientation.y,
                    target_pose.orientation.z, target_pose.orientation.w);
  tf::Matrix3x3 m1(q1), m2(q2);
  double r1, p1, y1, r2, p2, y2;
  m1.getRPY(r1, p1, y1);
  m2.getRPY(r2, p2, y2);
  
  double dtheta = fabs(y1 - y2);
  dtheta = std::min(dtheta, 2.0*M_PI - dtheta);
  //dtheta = dtheta / M_PI * 180;
  return dtheta;
}

int main(int argc, char** argv)
{
#ifdef MOVEL_LICENSE
  MovelLicense ml;
  if (!ml.login())
    return 1;
#endif

  ros::init(argc, argv, "pallet_docking");
  PalletDocking dock;

#ifdef MOVEL_LICENSE
  ml.logout();
#endif
}