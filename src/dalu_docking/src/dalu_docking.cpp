#include <dalu_docking/dalu_docking.h>
#include <movel_hasp_vendor/license.h>

DaluDocking::DaluDocking() : nh_private_("~"), tfListener_(tfBuffer_), history_index_(0)
{
  initialize();
}

void DaluDocking::initialize()
{
  goal_received_ = false;
  run_ = false;
  x_loop_ = false;
  y_loop_ = false;
  yaw_loop_ = false;

  if(!loadParams())
  {
    ROS_FATAL("[dalu_docking] Failed to load params. Shutting down.");
    return;
  }
  setupTopics();

  ros::spin();
}

bool DaluDocking::loadParams()
{
  if (nh_private_.hasParam("xy_tolerance"))
    nh_private_.getParam("xy_tolerance", p_xy_tolerance_);
  else
    return false;

  if (nh_private_.hasParam("yaw_tolerance"))
    nh_private_.getParam("yaw_tolerance", p_yaw_tolerance_);
  else
    return false;

  if (nh_private_.hasParam("frames_tracked"))
    nh_private_.getParam("frames_tracked", p_frames_tracked_);
  else
    return false;

  if (nh_private_.hasParam("max_linear_vel"))
    nh_private_.getParam("max_linear_vel", p_max_linear_vel_);
  else
    return false;

  if (nh_private_.hasParam("min_linear_vel"))
    nh_private_.getParam("min_linear_vel", p_min_linear_vel_);
  else
    return false;

  if (nh_private_.hasParam("max_turn_vel"))
    nh_private_.getParam("max_turn_vel", p_max_turn_vel_);
  else
    return false;

  if (nh_private_.hasParam("min_turn_vel"))
    nh_private_.getParam("min_turn_vel", p_min_turn_vel_);
  else
    return false;

  return true;
}

void DaluDocking::setupTopics()
{
  success_pub_ = nh_private_.advertise<std_msgs::Bool>("success", 1);
  vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel_mux/autonomous", 1);
  pose_sub_ = nh_private_.subscribe("goal", 1, &DaluDocking::goalCallback, this);

  run_service_ = nh_private_.advertiseService("run", &DaluDocking::runService, this);
  run_timer_ = nh_.createTimer(ros::Duration(0.1), boost::bind(&DaluDocking::runDocking, this, _1));
}

void DaluDocking::runDocking(const ros::TimerEvent& event)
{
  if(run_)
  {
    geometry_msgs::Twist stop;
    geometry_msgs::TransformStamped dock_goal;
    geometry_msgs::PoseStamped goal_pose;
    geometry_msgs::Twist correctional_vel;  
    double current_vel;
    double yaw;

    // If dock with apriltag
    if(!goal_received_)
    {
      try
      {
        dock_goal = tfBuffer_.lookupTransform("base_link", "dock_position", ros::Time(0), ros::Duration(0.5));
      }
      catch (tf2::TransformException &ex)
      {
        ROS_WARN("%s", ex.what());
        cleanupTask(false);
        return;
      }
      tfToPose(dock_goal, goal_pose);
      yaw = getYaw(goal_pose);
    }
    
    // If dock with goal input
    else
    {
      geometry_msgs::TransformStamped transform;
      try
      {
        transform = tfBuffer_.lookupTransform("base_link", goal_pose_.header.frame_id, ros::Time(0), ros::Duration(0.5));
      }
      catch (tf2::TransformException &ex)
      {
        ROS_WARN("%s", ex.what());
        cleanupTask(false);
        return;
      }
      tf2::doTransform(goal_pose_, goal_pose, transform);
      yaw = getYaw(goal_pose);
    }

    // Correct yaw w.r.t. docking goal
    if(fabs(yaw) > p_yaw_tolerance_ && !y_loop_ && !x_loop_)
    {
      if(fabs(yaw) > 0.2)
        current_vel = p_max_turn_vel_;
      else
        current_vel = std::max(fabs(yaw) / 0.2 * p_max_turn_vel_, p_min_turn_vel_);
      yaw_loop_ = true;
      if(yaw > 0)
      {
        correctional_vel.angular.z = current_vel;
        vel_pub_.publish(correctional_vel);
      }
      else
      {
        correctional_vel.angular.z = -current_vel;
        vel_pub_.publish(correctional_vel);
      }
    }
    else if(fabs(yaw) < p_yaw_tolerance_ && yaw_loop_)
    {
      vel_pub_.publish(stop);
      ros::Duration(1.0).sleep();
      yaw_loop_ = false;
    }

    if(!historyAveraging(dock_goal))
    {
      cleanupTask(false);
      return;
    }

    // Correct y position w.r.t. docking goal
    if(fabs(dock_goal.transform.translation.y) > p_xy_tolerance_ && !yaw_loop_)
    {
      if(x_loop_)
      {
        ros::Duration(1.0).sleep();
        x_loop_ = false;
      }

      if(fabs(dock_goal.transform.translation.y) > 0.2)
        current_vel = p_max_linear_vel_;
      else
        current_vel = std::max(fabs(dock_goal.transform.translation.y) / 0.2 * p_max_linear_vel_, p_min_linear_vel_);
      y_loop_ = true;
      if(dock_goal.transform.translation.y > 0)
      {
        correctional_vel.linear.y = current_vel;
        vel_pub_.publish(correctional_vel);
      }
      else
      {
        correctional_vel.linear.y = -current_vel;
        vel_pub_.publish(correctional_vel);
      }
    }
    else if(fabs(dock_goal.transform.translation.y) < p_xy_tolerance_ && y_loop_)
    {
      vel_pub_.publish(stop);
      ros::Duration(1.0).sleep();
      y_loop_ = false;
    }

    // Correct x position w.r.t. docking goal
    if(fabs(dock_goal.transform.translation.x) > p_xy_tolerance_ && !yaw_loop_ && !y_loop_)
    {
      if(fabs(dock_goal.transform.translation.x) > 0.2)
        current_vel = p_max_linear_vel_;
      else
        current_vel = std::max(fabs(dock_goal.transform.translation.x) / 0.2 * p_max_linear_vel_, p_min_linear_vel_);
      x_loop_ = true;
      if(dock_goal.transform.translation.x > 0)
      {
        correctional_vel.linear.x = current_vel;
        vel_pub_.publish(correctional_vel);
      }
      else
      {
        correctional_vel.linear.x = -current_vel;
        vel_pub_.publish(correctional_vel);
      }
    }
    else if(fabs(dock_goal.transform.translation.x) < p_xy_tolerance_ && x_loop_)
    {
      vel_pub_.publish(stop);
      ros::Duration(1.0).sleep();
      x_loop_ = false;
    }

    // Docking complete
    if(fabs(dock_goal.transform.translation.x) < p_xy_tolerance_ && fabs(dock_goal.transform.translation.y) < p_xy_tolerance_)
    {
      cleanupTask(true);
      ROS_INFO("[dalu_docking] Docked");
    }
  }
}

bool DaluDocking::historyAveraging(geometry_msgs::TransformStamped& goal)
{
  geometry_msgs::TransformStamped dock_goal;
  geometry_msgs::PoseStamped goal_pose;
  
  // Record a few transforms of docking goal to get the average transform
  while(x_history_.size() < p_frames_tracked_)
  {
    // If dock with apriltag
    if(!goal_received_)
    {
      try
      {
        dock_goal = tfBuffer_.lookupTransform("base_link", "dock_position", ros::Time(0), ros::Duration(0.5));
      }
      catch (tf2::TransformException &ex)
      {
        ROS_WARN("%s", ex.what());
        return false;
      }
      x_history_.push_back(dock_goal.transform.translation.x);
      y_history_.push_back(dock_goal.transform.translation.y);
    }
    
    // If dock with goal input
    else
    {
      geometry_msgs::TransformStamped transform;
      try
      {
        transform = tfBuffer_.lookupTransform("base_link", goal_pose_.header.frame_id, ros::Time(0), ros::Duration(0.5));
      }
      catch (tf2::TransformException &ex)
      {
        ROS_WARN("%s", ex.what());
        return false;
      }
      tf2::doTransform(goal_pose_, goal_pose, transform);
      x_history_.push_back(goal_pose.pose.position.x);
      y_history_.push_back(goal_pose.pose.position.y);
    }
    ros::Duration(0.5).sleep();
  }
  
  double sum_x = 0;
  double sum_y = 0;

  // If dock with apriltag
  if(!goal_received_)
  {
    try
    {
      dock_goal = tfBuffer_.lookupTransform("base_link", "dock_position", ros::Time(0), ros::Duration(0.5));
    }
    catch (tf2::TransformException &ex)
    {
      ROS_WARN("%s", ex.what());
      return false;
    }
    //! Overwrite old xy transform of goal with latest transform
    x_history_[history_index_] = dock_goal.transform.translation.x;
    y_history_[history_index_] = dock_goal.transform.translation.y;
  }
  
  // If dock with goal input
  else
  {
    geometry_msgs::TransformStamped transform;
    try
    {
      transform = tfBuffer_.lookupTransform("base_link", goal_pose_.header.frame_id, ros::Time(0), ros::Duration(0.5));
    }
    catch (tf2::TransformException &ex)
    {
      ROS_WARN("%s", ex.what());
      return false;
    }
    tf2::doTransform(goal_pose_, goal_pose, transform);
    //! Overwrite old xy transform of goal with latest transform
    x_history_[history_index_] = goal_pose.pose.position.x;
    y_history_[history_index_] = goal_pose.pose.position.y;
  }

  history_index_ = (history_index_ + 1) % p_frames_tracked_;

  // Find average xy transform of goal
  for(size_t i = 0; i < p_frames_tracked_; i++)
  {
    sum_x += x_history_[i];
    sum_y += y_history_[i];
  }

  goal.transform.translation.x = sum_x / p_frames_tracked_;
  goal.transform.translation.y = sum_y / p_frames_tracked_;
  return true;
}

bool DaluDocking::runService(std_srvs::SetBool::Request& request, std_srvs::SetBool::Response& response)
{
  // Start docking
  if(request.data)
  {
    ROS_INFO("[dalu_docking] Start docking with apriltag");
    goal_received_ = false;
    run_ = true;
  }
  
  // Stop docking
  else
  {
    goal_received_ = false;
    run_ = false;
    geometry_msgs::Twist stop;
    vel_pub_.publish(stop);
    x_history_.clear();
    y_history_.clear();
    history_index_ = 0;
    x_loop_ = false;
    y_loop_ = false;
    yaw_loop_ = false;
    ROS_INFO("[dalu_docking] Docking cancelled");
  }
  response.success = true;
  return true;
}

void DaluDocking::cleanupTask(bool success)
{
  goal_received_ = false;
  run_ = false;
  geometry_msgs::Twist stop;
  vel_pub_.publish(stop);
  x_history_.clear();
  y_history_.clear();
  history_index_ = 0;
  x_loop_ = false;
  y_loop_ = false;
  yaw_loop_ = false;
  std_msgs::Bool success_pub;
  success_pub.data = success;
  success_pub_.publish(success_pub);
}

void DaluDocking::goalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  geometry_msgs::PoseStamped pose;
  if(!run_)
  {
    goal_pose_ = *msg;
    goal_received_ = true;
    run_ = true;
    ROS_INFO("[dalu_docking] Start docking with goal input");
  }
  else
    ROS_WARN("[dalu_docking] Received goal input while docking. Please cancel current docking before giving new goal");
}

double DaluDocking::getYaw(geometry_msgs::PoseStamped pose)
{
  tf::Quaternion q(pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w);
  tf::Matrix3x3 m(q);
  double r, p, y;
  m.getRPY(r, p, y);
  return y;
}

void DaluDocking::tfToPose(geometry_msgs::TransformStamped tf, geometry_msgs::PoseStamped& pose)
{
  double norm = sqrt(pow(tf.transform.rotation.z, 2) + pow(tf.transform.rotation.w, 2));
  pose.header.frame_id = "base_link";
  pose.pose.position.x = tf.transform.translation.x;
  pose.pose.position.y = tf.transform.translation.y;
  pose.pose.position.z = 0;
  pose.pose.orientation.x = 0;
  pose.pose.orientation.y = 0;
  pose.pose.orientation.z = tf.transform.rotation.z / norm;
  pose.pose.orientation.w = tf.transform.rotation.w / norm;

  tf2::Quaternion q_orig, q_rot, q_new;
  tf2::convert(pose.pose.orientation, q_orig);
  double r = 0, p = 0, y = M_PI/2;
  q_rot.setRPY(r, p, y);
  q_new = q_rot * q_orig;
  q_new.normalize();
  tf2::convert(q_new, pose.pose.orientation);
}

int main(int argc, char** argv)
{
  #ifdef MOVEL_LICENSE                                                                                                    
  MovelLicense ml(37);                                                                                                   
  if (!ml.login())                                                                                                      
    return 1;                                                                                                           
  #endif

  ros::init(argc, argv, "dalu_docking");
  DaluDocking dd;
  
  #ifdef MOVEL_LICENSE                                                                                                    
  ml.logout();          
  #endif
}
