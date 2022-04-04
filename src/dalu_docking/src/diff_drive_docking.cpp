#include <dalu_docking/diff_drive_docking.h>
#include <movel_hasp_vendor/license.h>

DiffDriveDocking::DiffDriveDocking() : nh_private_("~"), tfListener_(tfBuffer_), history_index_(0), action_state_("IDLE"),
                                       odom_received_(false), start_final_approach_(false), parallel_approach_(false)
{
  initialize();
}

void DiffDriveDocking::initialize()
{
  //goal_received_ = false;
  run_ = true; //false;
  start_ = true; //false;
  turn_loop_ = false;
  approach_loop_ = false;

  if(!loadParams())
  {
    ROS_FATAL("[diff_drive_docking] Failed to load params. Shutting down.");
    return;
  }
  setupTopics();

  ros::spin();
}

bool DiffDriveDocking::loadParams()
{
  if (nh_private_.hasParam("init_xy_tolerance"))
    nh_private_.getParam("init_xy_tolerance", p_init_xy_tolerance_);
  else
    return false;

  if (nh_private_.hasParam("final_xy_tolerance"))
    nh_private_.getParam("final_xy_tolerance", p_final_xy_tolerance_);
  else
    return false;

  if (nh_private_.hasParam("yaw_tolerance"))
    nh_private_.getParam("yaw_tolerance", p_yaw_tolerance_);
  else
    return false;

  if (nh_private_.hasParam("final_yaw_tolerance"))
    nh_private_.getParam("final_yaw_tolerance", p_final_yaw_tolerance_);
  else
    return false;

  if (nh_private_.hasParam("reverse"))
    nh_private_.getParam("reverse", p_reverse_);
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

  if (nh_private_.hasParam("linear_acc"))
    nh_private_.getParam("linear_acc", p_linear_acc_);
  else
    return false;

  if (nh_private_.hasParam("final_approach_dist"))
    nh_private_.getParam("final_approach_dist", p_final_approach_dist_);
  else
    return false;

  if (nh_private_.hasParam("max_yaw_diff"))
    nh_private_.getParam("max_yaw_diff", p_max_yaw_diff_);
  else
    return false;

  if (nh_private_.hasParam("y_bounds"))
    nh_private_.getParam("y_bounds", p_y_bounds_);
  else
    return false;

  if (nh_private_.hasParam("reference_frame"))
    nh_private_.getParam("reference_frame", p_reference_frame_);
  else
    return false;

  if (nh_private_.hasParam("xy_update_tolerance"))
    nh_private_.getParam("xy_update_tolerance", p_xy_update_tolerance_);
  else
    return false;

  if (nh_private_.hasParam("yaw_update_tolerance"))
    nh_private_.getParam("yaw_update_tolerance", p_yaw_update_tolerance_);
  else
    return false;

  if(p_reverse_)
    ROS_INFO("[diff_drive_docking] Dock in reverse");
  else
    ROS_INFO("[diff_drive_docking] Dock forwards");

  return true;
}

void DiffDriveDocking::setupTopics()
{
  success_pub_ = nh_private_.advertise<std_msgs::Bool>("success", 1);
  vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel_mux/autonomous", 1);
  //pose_sub_ = nh_private_.subscribe("goal", 1, &DiffDriveDocking::goalCallback, this);
  odom_sub_ = nh_.subscribe("odom", 1, &DiffDriveDocking::odomCb, this);

  run_service_ = nh_private_.advertiseService("run", &DiffDriveDocking::runService, this);
  run_timer_ = nh_.createTimer(ros::Duration(1.0/15.0), boost::bind(&DiffDriveDocking::runDocking, this, _1));
  pose_pub_ = nh_private_.advertise<geometry_msgs::PoseStamped>("current_goal", 1);
}

// Get odom data
void DiffDriveDocking::odomCb(nav_msgs::Odometry odom)
{
  current_odom_ = odom;
  odom_received_ = true;
}

void DiffDriveDocking::approachDock(double dtheta, double distance)
{
  geometry_msgs::Twist stop;

  // Correct heading of robot to face docking position
  if(fabs(dtheta) >= p_yaw_tolerance_)
  {
    if(approach_loop_)
    {
      linear_vel_ = 0;
      vel_pub_.publish(stop);
      ros::Duration(1.0).sleep();
      approach_loop_ = false;
      ROS_INFO("[diff_drive_docking] Turn to face goal");
    }
    if(action_state_ != "TURNING1")
    {
      action_state_ = "TURNING1";
    }
    turn_loop_ = true;
  }

  // Approach docking position
  else
  {
    if(turn_loop_)
    {
      linear_vel_ = 0;
      vel_pub_.publish(stop);
      ros::Duration(1.0).sleep();
      turn_loop_ = false;
      ROS_INFO("[diff_drive_docking] Move towards goal");
    }
    if (action_state_ != "LINEAR")
    {
      action_state_ == "LINEAR";
    }
    linearMotion(distance, p_reverse_);
    approach_loop_ = true;
  }
}

void DiffDriveDocking::backAway(double dtheta)
{
  geometry_msgs::Twist stop;

  // Match current yaw with final yaw at docking position
  if(fabs(dtheta) >= p_yaw_tolerance_)
  {
    if(approach_loop_)
    {
      linear_vel_ = 0;
      vel_pub_.publish(stop);
      ros::Duration(1.0).sleep();
      approach_loop_ = false;
      ROS_INFO("[diff_drive_docking] Reorient before backing away");
    }
    if(action_state_ != "TURNING2")
    {
      action_state_ = "TURNING2";
    }
    turn_loop_ = true;
  }

  // Move away from docking position
  else
  {
    if(turn_loop_)
    {
      linear_vel_ = 0;
      vel_pub_.publish(stop);
      ros::Duration(1.0).sleep();
      turn_loop_ = false;
      ROS_INFO("[diff_drive_docking] Back away");
    }
    if (action_state_ != "LINEAR")
    {
      action_state_ == "LINEAR";
    }
    linearMotion(0.25, !p_reverse_);
    approach_loop_ = true;
  }
}

void DiffDriveDocking::parallelApproach(double dtheta, double distance)
{
  geometry_msgs::Twist stop;

  // Match current yaw with final yaw at docking position
  if(fabs(dtheta) >= p_yaw_tolerance_)
  {
    if(approach_loop_)
    {
      linear_vel_ = 0;
      vel_pub_.publish(stop);
      ros::Duration(1.0).sleep();
      approach_loop_ = false;
      ROS_INFO("[diff_drive_docking] Reorient before approach");
    }
    if(action_state_ != "TURNING2")
    {
      action_state_ = "TURNING2";
    }
    turn_loop_ = true;
  }

  // Move towards docking position
  else
  {
    if(turn_loop_)
    {
      linear_vel_ = 0;
      vel_pub_.publish(stop);
      ros::Duration(1.0).sleep();
      turn_loop_ = false;
      ROS_INFO("[diff_drive_docking] Parallel approach");
    }
    if (action_state_ != "LINEAR")
    {
      action_state_ == "LINEAR";
    }
    linearMotion(distance, p_reverse_);
    approach_loop_ = true;
  }
}

void DiffDriveDocking::linearMotion(double distance, bool backwards)
{
  geometry_msgs::Twist correctional_vel;
  double current_vel;

  // Set max speed based on remaining travel distance
  if(distance > 0.5)
    current_vel = p_max_linear_vel_;
  else
    current_vel = std::max(distance / 0.5 * p_max_linear_vel_, p_min_linear_vel_);

  // Gradual speed increment (accleration)
  current_vel = std::min(linear_vel_ + p_linear_acc_ / 15, current_vel);
  linear_vel_ = current_vel;

  // Set direction
  if(!backwards)
    correctional_vel.linear.x = current_vel;
  else
    correctional_vel.linear.x = -current_vel;
  vel_pub_.publish(correctional_vel);
}

void DiffDriveDocking::correctYaw(double dtheta)
{
  geometry_msgs::Twist correctional_vel;
  double current_vel;

  // Set max speed based on remaining angle rotation
  if(fabs(dtheta) > 0.35)
    current_vel = p_max_turn_vel_;
  else
    current_vel = std::max(fabs(dtheta) / 0.35 * p_max_turn_vel_, p_min_turn_vel_);
  
  // Set direction
  if(dtheta < 0)
    correctional_vel.angular.z = -current_vel;
  else
    correctional_vel.angular.z = current_vel;
  vel_pub_.publish(correctional_vel);
}

void DiffDriveDocking::runDocking(const ros::TimerEvent& event)
{
  if(run_)
  {
    double p_xy_tolerance = (start_) ? p_init_xy_tolerance_ : p_final_xy_tolerance_;

    // Final linear motion
    if(start_final_approach_)
    {
      double distance = p_final_approach_dist_ - sqrt(pow(current_odom_.pose.pose.position.x - init_odom_.pose.pose.position.x, 2) + 
                                                      pow(current_odom_.pose.pose.position.y - init_odom_.pose.pose.position.y, 2));
      // Go towards docking position
      if(distance > p_xy_tolerance)
      {
        linearMotion(distance, p_reverse_);
        return;
      }

      // Docked
      else
      {
        cleanupTask(true);
        ROS_INFO("[diff_drive_docking] Docked");
        return;
      }      
    }

    geometry_msgs::TransformStamped dock_goal, robot_pose;

    // Get average docking pose
    historyAveraging(dock_goal);

    // Get current robot pose
    try
    {
      robot_pose = tfBuffer_.lookupTransform(p_reference_frame_, "base_link", ros::Time(0), ros::Duration(0.5));
    }
    catch (tf2::TransformException &ex)
    {
      ROS_WARN("%s", ex.what());
      return;
    }
    //geometry_msgs::PoseStamped pose;
    //tfToPose(dock_goal, pose);
    //pose_pub_.publish(pose);

    // Turn to face docking position
    if(action_state_ == "TURNING1")
    {
      double current_yaw = tf::getYaw(robot_pose.transform.rotation);
      double goal_heading = atan2(dock_goal.transform.translation.y - robot_pose.transform.translation.y, dock_goal.transform.translation.x - robot_pose.transform.translation.x);
      if(p_reverse_)
      {
        goal_heading += M_PI;
        if(goal_heading > M_PI)
          goal_heading -= 2*M_PI;
      }
      double dtheta = goal_heading - current_yaw;
      if(dtheta>M_PI){dtheta = dtheta-(2*M_PI);}
      if(dtheta<-M_PI){dtheta = dtheta+(2*M_PI);}

      if(fabs(dtheta) > p_final_yaw_tolerance_)
      {
        correctYaw(dtheta);
        return;
      }
      else
      {
        geometry_msgs::Twist stop;
        vel_pub_.publish(stop);
        action_state_ = "IDLE";
      }
    }
    // Turn to match current yaw with final yaw
    else if (action_state_ == "TURNING2")
    {
      double target_yaw = tf::getYaw(dock_goal.transform.rotation);
      double current_yaw = tf::getYaw(robot_pose.transform.rotation);

      double dtheta = target_yaw - current_yaw;
      if(dtheta>M_PI){dtheta = dtheta-(2*M_PI);}
      if(dtheta<-M_PI){dtheta = dtheta+(2*M_PI);}

      if(fabs(dtheta) > p_final_yaw_tolerance_)
      {
        correctYaw(dtheta);
        return;
      }
      else
      {
        geometry_msgs::Twist stop;
        vel_pub_.publish(stop);
        action_state_ = "IDLE";
      }
    }

    // Not reached goal
    if (sqrt(pow(dock_goal.transform.translation.x - robot_pose.transform.translation.x, 2) + pow(dock_goal.transform.translation.y - robot_pose.transform.translation.y, 2)) > p_xy_tolerance)
    {
      double current_yaw = tf::getYaw(robot_pose.transform.rotation);
      double goal_heading = atan2(dock_goal.transform.translation.y - robot_pose.transform.translation.y, dock_goal.transform.translation.x - robot_pose.transform.translation.x);
      if(p_reverse_)
      {
        goal_heading += M_PI;
        if(goal_heading > M_PI)
          goal_heading -= 2*M_PI;
      }
      double target_yaw = tf::getYaw(dock_goal.transform.rotation);
      double theta = target_yaw - goal_heading;
      if(theta>M_PI){theta = theta-(2*M_PI);}
      if(theta<-M_PI){theta = theta+(2*M_PI);}

      // Exceed yaw difference tolerance between final yaw and yaw to go towards goal (only applies to 2nd phase)
      if(!start_ && fabs(theta) > p_max_yaw_diff_ || parallel_approach_)
      {
        dock_goal.header.stamp = ros::Time::now();
        dock_goal.header.frame_id = p_reference_frame_;
        dock_goal.child_frame_id = "current_goal";
        br_.sendTransform(dock_goal);

        geometry_msgs::TransformStamped base_link_to_goal;
        try
        {
          base_link_to_goal = tfBuffer_.lookupTransform("current_goal", "base_link", ros::Time(0), ros::Duration(0.5));
        }
        catch (tf2::TransformException &ex)
        {
          ROS_WARN("%s", ex.what());
          return;
        }

        double dtheta = target_yaw - current_yaw;
        if(dtheta>M_PI){dtheta = dtheta-(2*M_PI);}
        if(dtheta<-M_PI){dtheta = dtheta+(2*M_PI);}

        // Robot outside y bounds relative to docking position
        if(fabs(base_link_to_goal.transform.translation.y) > p_y_bounds_ || fabs(theta) > M_PI/2)
        {
          backAway(dtheta);
          parallel_approach_ = false;
        }
        // Robot within y bounds relative to docking position
        else
        {
          double distance = sqrt(pow(dock_goal.transform.translation.x - robot_pose.transform.translation.x, 2) +
                               pow(dock_goal.transform.translation.y - robot_pose.transform.translation.y, 2));
          parallelApproach(dtheta, distance);
          parallel_approach_ = true;
        }
      }

      // Within yaw difference tolereance between final yaw and yaw to go towards goal
      else
      {
        double dtheta = goal_heading - current_yaw;
        if(dtheta>M_PI){dtheta = dtheta-(2*M_PI);}
        if(dtheta<-M_PI){dtheta = dtheta+(2*M_PI);}

        double distance = sqrt(pow(dock_goal.transform.translation.x - robot_pose.transform.translation.x, 2) +
                               pow(dock_goal.transform.translation.y - robot_pose.transform.translation.y, 2));
        approachDock(dtheta, distance);
      }
      log_printed_ = false;
    }

    // Reached goal
    else
    {
      double target_yaw = tf::getYaw(dock_goal.transform.rotation);
      double current_yaw = tf::getYaw(robot_pose.transform.rotation);      
      double dtheta = target_yaw - current_yaw;
      if(dtheta>M_PI){dtheta = dtheta-(2*M_PI);}
      if(dtheta<-M_PI){dtheta = dtheta+(2*M_PI);}

      // Correct yaw after reaching goal
      if(fabs(dtheta) > p_final_yaw_tolerance_)
      {
        if(action_state_ != "TURNING2")
        {
          action_state_ = "TURNING2";
        }
        if(!log_printed_)
        {
          ROS_INFO("[diff_drive_docking] Final yaw correction");
          log_printed_ = true;
        }
      }

      // Go to next stage in docking motion
      else
      {
        // Change goal from initial position to dock position
        if(start_)
        {
          start_ = false;
          linear_vel_ = 0;
          geometry_msgs::Twist stop;
          vel_pub_.publish(stop);
          x_history_.clear();
          y_history_.clear();
          yaw_history_.clear();
          history_index_ = 0;
          turn_loop_ = false;
          approach_loop_ = false;
          action_state_ = "IDLE";
          ROS_INFO("[diff_drive_docking] Proceed to phase 2");
        }

        // Start final linear motion
        else
        {
          init_odom_ = current_odom_;
          if(action_state_ != "LINEAR")
            action_state_ = "LINEAR";
          ROS_INFO("[diff_drive_docking] Final approach");
          start_final_approach_ = true;
        }
      }
    }
  }
}

bool DiffDriveDocking::historyAveraging(geometry_msgs::TransformStamped& goal)
{
  geometry_msgs::TransformStamped dock_goal;
  geometry_msgs::PoseStamped goal_pose;
  double sum_x = 0;
  double sum_y = 0;
  double sum_yaw = 0;

  if(x_history_.size() == 0)
    ros::Duration(3.0).sleep();

  if(action_state_ != "TURNING1" && action_state_ != "TURNING2")
  {
    // Record a few transforms of docking goal to get the average transform
    while(x_history_.size() < p_frames_tracked_)
    {
      // If dock with apriltag
      //if(!goal_received_)
      {
        try
        {
          if(start_)
            dock_goal = tfBuffer_.lookupTransform(p_reference_frame_, "offset_tf2", ros::Time(0), ros::Duration(0.5));
          else
            dock_goal = tfBuffer_.lookupTransform(p_reference_frame_, "offset_tf", ros::Time(0), ros::Duration(0.5));
        }
        catch (tf2::TransformException &ex)
        {
          ROS_WARN("%s", ex.what());
          return false;
        }
        x_history_.push_back(dock_goal.transform.translation.x);
        y_history_.push_back(dock_goal.transform.translation.y);
        yaw_history_.push_back(tf::getYaw(dock_goal.transform.rotation));
      }
    
      // If dock with goal input
      /*else
      {
        geometry_msgs::TransformStamped transform;
        try
        {
          if(start_)
            dock_goal = tfBuffer_.lookupTransform("map", "offset_tf2", ros::Time(0), ros::Duration(0.5));
          else
            dock_goal = tfBuffer_.lookupTransform("map", "offset_tf", ros::Time(0), ros::Duration(0.5));
        }
        catch (tf2::TransformException &ex)
        {
          ROS_WARN("%s", ex.what());
          return false;
        }
        tf2::doTransform(goal_pose_, goal_pose, transform);
        x_history_.push_back(goal_pose.pose.position.x);
        y_history_.push_back(goal_pose.pose.position.y);
        yaw_history_.push_back(tf::getYaw(goal_pose.pose.orientation));
      }*/
      ros::Duration(0.1).sleep();
    }

    // If dock with apriltag
    //if(!goal_received_)
    {
      try
      {
        if(start_)
          dock_goal = tfBuffer_.lookupTransform(p_reference_frame_, "offset_tf2", ros::Time(0), ros::Duration(0.5));
        else
          dock_goal = tfBuffer_.lookupTransform(p_reference_frame_, "offset_tf", ros::Time(0), ros::Duration(0.5));
      }
      catch (tf2::TransformException &ex)
      {
        ROS_WARN("%s", ex.what());
        goal = current_goal_;
        return false;
      }
      //! Overwrite old xy transform of goal with latest transform
      x_history_[history_index_] = dock_goal.transform.translation.x;
      y_history_[history_index_] = dock_goal.transform.translation.y;
      yaw_history_[history_index_] = tf::getYaw(dock_goal.transform.rotation);
    }
  
    // If dock with goal input
    /*else
    {
      geometry_msgs::TransformStamped transform;
      try
      {
        transform = tfBuffer_.lookupTransform("map", goal_pose_.header.frame_id, ros::Time(0), ros::Duration(0.5));
      }
      catch (tf2::TransformException &ex)
      {
        ROS_WARN("%s", ex.what());
        goal = current_goal_;
        return false;
      }
      tf2::doTransform(goal_pose_, goal_pose, transform);
      //! Overwrite old xy transform of goal with latest transform
      x_history_[history_index_] = goal_pose.pose.position.x;
      y_history_[history_index_] = goal_pose.pose.position.y;
      yaw_history_[history_index_] = tf::getYaw(goal_pose.pose.orientation);
    }*/

    history_index_ = (history_index_ + 1) % p_frames_tracked_;
  }
  // Find average xy transform of goal
  for(size_t i = 0; i < p_frames_tracked_; i++)
  {
    sum_x += x_history_[i];
    sum_y += y_history_[i];
    sum_yaw += yaw_history_[i];
  }

  geometry_msgs::TransformStamped temp_goal;
  temp_goal.transform.translation.x = sum_x / p_frames_tracked_;
  temp_goal.transform.translation.y = sum_y / p_frames_tracked_;

  tf::Quaternion q;
  geometry_msgs::Quaternion quat;
  q.setRPY(0, 0, sum_yaw / p_frames_tracked_);
  tf::quaternionTFToMsg(q, quat);
  temp_goal.transform.rotation = quat;

  geometry_msgs::PoseStamped pose;
  if(!goal_published_)
  {
    goal = temp_goal;
    current_goal_ = goal;
    tfToPose(goal, pose);
    pose_pub_.publish(pose);
    goal_published_ = true;
  }
  else
  {
    double distance = sqrt(pow(current_goal_.transform.translation.x - temp_goal.transform.translation.x, 2) + 
                           pow(current_goal_.transform.translation.y - temp_goal.transform.translation.y, 2));
    double current_yaw = tf::getYaw(current_goal_.transform.rotation);
    double temp_yaw = tf::getYaw(temp_goal.transform.rotation);
    double dtheta = temp_yaw - current_yaw;
    if(dtheta>M_PI){dtheta = dtheta-(2*M_PI);}
    if(dtheta<-M_PI){dtheta = dtheta+(2*M_PI);}

    if(distance > p_xy_update_tolerance_ || dtheta > p_yaw_update_tolerance_)
    {
      goal = temp_goal;
      current_goal_ = goal;
      tfToPose(goal, pose);
      pose_pub_.publish(pose);
    }
    else
      goal = current_goal_;
  }
  return true;
}

bool DiffDriveDocking::runService(std_srvs::SetBool::Request& request, std_srvs::SetBool::Response& response)
{
  // Start docking
  if(request.data)
  {
    ROS_INFO("[diff_drive_docking] Start docking with apriltag");
    //goal_received_ = false;
    run_ = true;
    start_ = true;
  }
  
  // Stop docking
  else
  {
    cleanupTask(true);
    ROS_INFO("[diff_drive_docking] Docking cancelled");
  }
  response.success = true;
  return true;
}

void DiffDriveDocking::cleanupTask(bool success)
{
  geometry_msgs::Twist stop;
  vel_pub_.publish(stop);
  //goal_received_ = false;
  run_ = false;
  start_ = false;
  linear_vel_ = 0;
  start_final_approach_ = false;
  parallel_approach_ = false;
  x_history_.clear();
  y_history_.clear();
  yaw_history_.clear();
  history_index_ = 0;
  turn_loop_ = false;
  approach_loop_ = false;
  action_state_ = "IDLE";
  std_msgs::Bool success_pub;
  success_pub.data = success;
  success_pub_.publish(success_pub);
}

/*void DiffDriveDocking::goalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  geometry_msgs::PoseStamped pose;
  if(!run_)
  {
    goal_pose_ = *msg;
    goal_received_ = true;
    run_ = true;
    start_ = true;
    ROS_INFO("[diff_drive_docking] Start docking with goal input");
  }
  else
    ROS_WARN("[diff_drive_docking] Received goal input while docking. Please cancel current docking before giving new goal");
}*/

void DiffDriveDocking::tfToPose(geometry_msgs::TransformStamped tf, geometry_msgs::PoseStamped& pose)
{
  double norm = sqrt(pow(tf.transform.rotation.z, 2) + pow(tf.transform.rotation.w, 2));
  pose.header.frame_id = p_reference_frame_;
  pose.pose.position.x = tf.transform.translation.x;
  pose.pose.position.y = tf.transform.translation.y;
  pose.pose.position.z = 0;
  pose.pose.orientation.x = 0;
  pose.pose.orientation.y = 0;
  pose.pose.orientation.z = tf.transform.rotation.z / norm;
  pose.pose.orientation.w = tf.transform.rotation.w / norm;
}

int main(int argc, char** argv)
{
  #ifdef MOVEL_LICENSE                                                                                                    
  MovelLicense ml;                                                                                                   
  if (!ml.login())                                                                                                      
    return 1;                                                                                                           
  #endif

  ros::init(argc, argv, "diff_drive_docking");
  DiffDriveDocking dd;
  
  #ifdef MOVEL_LICENSE                                                                                                    
  ml.logout();          
  #endif
}
