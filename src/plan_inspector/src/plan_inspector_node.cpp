#include <plan_inspector/plan_inspector.hpp>
#include <movel_hasp_vendor/license.h>

PlanInspector::PlanInspector()
: enable_(true), have_plan_(false), have_costmap_(false)
, have_action_status_(false), timer_active_(false), path_obstructed_(false), reconfigure_(false)
, tf_ear_(tf_buffer_), stop_(false), use_teb_(false), override_velo_(false), task_pause_status_(false)
, internal_pause_trigger_(false), terminal_state_(false)
{
  if (!setupParams())
  {
    ROS_INFO("bad parameters");
    return;
  }

  if (!setupTopics())
  {
    ROS_INFO("failed to setup topics");
    return;
  }

  abort_timer_ = nh_.createTimer(ros::Duration(clearing_timeout_),
                                 &PlanInspector::abortTimerCb, this,
                                 true, false);

  control_timer_ = nh_.createTimer(ros::Duration(1.0/control_frequency_),
                                   &PlanInspector::controlTimerCb, this,
                                   false, false);

  set_common_params_.waitForExistence();
  if (use_teb_)
    set_teb_params_.waitForExistence();

  if (enable_){
    bool success = reconfigureParams("reconfigure");
    reconfigure_ = true;
  }

}

bool PlanInspector::setupParams()
{
  ros::NodeHandle nl("~");

  if (nl.hasParam("enable"))
    nl.getParam("enable", enable_);

  action_server_name_ = "/move_base";
  if (nl.hasParam("action_server_name"))
    nl.getParam("action_server_name", action_server_name_);

  plan_topic_ = "/move_base/DWAPlannerROS/global_plan";
  if (nl.hasParam("plan_topic"))
    nl.getParam("plan_topic", plan_topic_);

  costmap_topic_ = "/move_base/local_costmap/costmap";
  if (nl.hasParam("costmap_topic"))
    nl.getParam("costmap_topic", costmap_topic_);

  obstruction_threshold_ = 75;
  if (nl.hasParam("obstruction_threshold"))
    nl.getParam("obstruction_threshold", obstruction_threshold_);

  clearing_timeout_ = 10.0;
  if (nl.hasParam("clearing_timeout"))
    nl.getParam("clearing_timeout", clearing_timeout_);

  cmd_vel_topic_ = "/cmd_vel_mux/teleop/keyboard";
  if (nl.hasParam("cmd_vel_topic"))
    nl.getParam("cmd_vel_topic", cmd_vel_topic_);

  control_frequency_ = 20.0;
  if (nl.hasParam("control_frequency"))
    nl.getParam("control_frequency", control_frequency_);

  config_topic_ = "/move_base/set_parameters";
  if (nl.hasParam("move_base_config"))
    nl.getParam("move_base_config", config_topic_);
  
  stop_distance_ = -1;
  if(nl.hasParam("stop_distance"))
    nl.getParam("stop_distance", stop_distance_);

  rotate_fov_ = true;
  if(nl.hasParam("rotate_fov"))
    nl.getParam("rotate_fov", rotate_fov_);

  rotation_speed_ = 0.7;
  if(nl.hasParam("rotation_speed"))
    nl.getParam("rotation_speed", rotation_speed_);
  saveParams();

  return true;
}

bool PlanInspector::setupTopics()
{
  // Subscribed topic
  plan_sub_ = nh_.subscribe(plan_topic_, 1, &PlanInspector::pathCb, this);
  costmap_sub_ = nh_.subscribe(costmap_topic_, 1, &PlanInspector::costmapCb, this); 

  // Goal topic
  string action_status_topic = action_server_name_ + "/status";
  action_status_sub_ = nh_.subscribe(action_status_topic, 1, &PlanInspector::actionStatusCb, this);
  pause_status_sub_ = nh_.subscribe("/task_supervisor/pause_status", 1, &PlanInspector::pauseStatusCb, this);
  action_pause_pub_ = nh_.advertise<std_msgs::Bool>("/task_supervisor/pause", 1); // only task_supervisor has pause, so no messing around with action_server_name_ here
  
  string action_cancel_topic = action_server_name_ + "/cancel";
  action_cancel_pub_ = nh_.advertise<actionlib_msgs::GoalID>(action_cancel_topic, 1);

  // Velocity topic
  zerovel_pub_ = nh_.advertise<geometry_msgs::Twist>(cmd_vel_topic_, 1);

  // Enabler
  enable_sub_ = nh_.advertiseService("enable_plan_inspector", &PlanInspector::enableCb, this);

  // Checker
  stop_obstacle_checker = nh_.advertiseService("/stop_obstacle_check", &PlanInspector::onStopObstacleCheck, this);

  // Dynamic Reconffgure
  set_common_params_ = nh_.serviceClient<dynamic_reconfigure::Reconfigure>(config_topic_);
  set_teb_params_ = nh_.serviceClient<dynamic_reconfigure::Reconfigure>("/move_base/TebLocalPlannerROS/set_parameters");
  task_supervisor_type = nh_.serviceClient<movel_seirios_msgs::GetTaskType>("/task_supervisor/get_task_type");
  // Reporting Topics
  obstruction_status_pub_ = nh_.advertise<movel_seirios_msgs::ObstructionStatus>("/obstruction_status",1);
  logger_sub_ = nh_.subscribe("/rosout", 1, &PlanInspector::loggerCb, this);
  planner_report_pub_ = nh_.advertise<std_msgs::String>("/planner_report",1);

  // move_base action client
  nav_ac_ptr_ = std::make_shared<actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> >("move_base", true);
  ROS_INFO("[plan_inspector] wait for move_base action server");
  if(!nav_ac_ptr_->waitForServer(ros::Duration(60.0)))
  {
    ROS_INFO("[plan_inspector] failed to connect to move_base action server");
    return false;
  }

  return true;
}

void PlanInspector::pathCb(nav_msgs::Path msg)
{
  // ROS_INFO("new path");

  // don't overwrite plan if it's not from the active task
  if (task_pause_status_)
    return;

  latest_plan_ = msg;
  have_plan_ = true;
  terminal_state_ = false;

  processNewInfo();
}

void PlanInspector::costmapCb(nav_msgs::OccupancyGrid msg)
{
  // ROS_INFO("new costmap");
  latest_costmap_ = msg;
  have_costmap_ = true;

  processNewInfo();
}

void PlanInspector::processNewInfo()
{
  if (task_pause_status_ && !internal_pause_trigger_)
    return;

  if (have_plan_ && have_costmap_)
  {
    bool obstructed = checkObstruction();

    // report regardless of enable
    if (obstructed)
    {
      std_msgs::String report;
      report.data = "global_plan";
      planner_report_pub_.publish(report);
    }

    if (enable_)
    {
      // rising edge
      if (obstructed && !path_obstructed_)
      {
        ROS_INFO("[plan_inspector] obstacle on path");
        path_obstructed_ = true;

        // report obstruction
        movel_seirios_msgs::ObstructionStatus report_obs;
        report_obs.reporter = "plan_inspector";
        report_obs.status = "true";
        report_obs.location = first_path_map_.pose;
        obstruction_status_pub_.publish(report_obs);

        geometry_msgs::PoseStamped robot_pose;
        if (!getRobotPose(robot_pose))
          return;

        // stop immediately
        if (stop_distance_ < 0.)
        {
          pauseTask();

          // orient robot
          if (rotate_fov_)
          {
            ROS_INFO("[plan_inspector] face to obstacle");
            double target_yaw = calcYaw(robot_pose.pose, first_path_map_.pose);
            robot_pose.pose.orientation.x = 0.0;
            robot_pose.pose.orientation.y = 0.0;
            robot_pose.pose.orientation.z = sin(0.5*target_yaw);
            robot_pose.pose.orientation.w = cos(0.5*target_yaw);
            move_base_msgs::MoveBaseGoal goal;
            goal.target_pose = robot_pose;
            nav_ac_ptr_->sendGoal(goal);
          }
          else
          {
            ROS_INFO("immediate stop");
            double timeout = clearing_timeout_;
            if (clearing_timeout_ < 0.)
              timeout = 24 * 3600;

            abort_timer_.setPeriod(ros::Duration(timeout));
            abort_timer_.start();

            stop_ = true;
          }
        }
        // or approach the obstacle
        else
        {
          // pause current task
          pauseTask();

          // get new goal
          double dee = calculateDistance();
          if (dee > stop_distance_)
          {
            ROS_INFO("[plan_inspector] approach obstacle");
            // rollback path
            int last_idx = 0;
            for (int i = 0; i < latest_plan_.poses.size(); i++)
            {
              double small_dee = calculateDistance(latest_plan_.poses[i].pose, first_path_map_.pose);
              if (small_dee <= stop_distance_)
              {
                last_idx = std::max(i, 0);
                break;
              }
            }
            geometry_msgs::PoseStamped intermediate_goal = latest_plan_.poses[last_idx];

            // override orientation
            if (rotate_fov_)
            {
              double target_yaw = calcYaw(intermediate_goal.pose, first_path_map_.pose);
              intermediate_goal.pose.orientation.x = 0.0;
              intermediate_goal.pose.orientation.y = 0.0;
              intermediate_goal.pose.orientation.z = sin(0.5*target_yaw);
              intermediate_goal.pose.orientation.w = cos(0.5*target_yaw);
            }

            // dispatch to move_base
            move_base_msgs::MoveBaseGoal goal;
            goal.target_pose = intermediate_goal;
            nav_ac_ptr_->sendGoal(goal);
          }
          else if (rotate_fov_)
          {
            ROS_INFO("[plan_inspector] want to approach, but already too close; face to obstacle");
            double target_yaw = calcYaw(robot_pose.pose, first_path_map_.pose);
            robot_pose.pose.orientation.x = 0.0;
            robot_pose.pose.orientation.y = 0.0;
            robot_pose.pose.orientation.z = sin(0.5*target_yaw);
            robot_pose.pose.orientation.w = cos(0.5*target_yaw);
            move_base_msgs::MoveBaseGoal goal;
            goal.target_pose = robot_pose;
            nav_ac_ptr_->sendGoal(goal);
          }

          // TODO: fill in above, clear below
          // double timeout = clearing_timeout_;
          // if (clearing_timeout_ < 0.)
          //   timeout = 24 * 3600;

          // abort_timer_.setPeriod(ros::Duration(timeout));
          // abort_timer_.start();
        }
      }
      // falling edge
      else if (!obstructed && path_obstructed_)
      {
        ROS_INFO("[plan_inspector] obstacle cleared");
        path_obstructed_ = false;

        // report clearance
        movel_seirios_msgs::ObstructionStatus report_obs;
        report_obs.reporter = "plan_inspector";
        report_obs.status = "false";
        report_obs.location = first_path_map_.pose;
        obstruction_status_pub_.publish(report_obs);

        // clear timers
        abort_timer_.stop();
        control_timer_.stop();
        stop_ = false;

        // resume task
        resumeTask();
      }
      // general obstructed 
      else if (obstructed)
      {
        actionlib::SimpleClientGoalState state = nav_ac_ptr_->getState();
        if (state.isDone())
        {
          if (!stop_)
          {
            double timeout = clearing_timeout_;
            if (clearing_timeout_ < 0.)
              timeout = 24 * 3600;

            abort_timer_.setPeriod(ros::Duration(timeout));
            abort_timer_.start();

            stop_ = true;
          }
        }
      }
    }
  }
}

void PlanInspector::processNewInfo2()
{
  if (have_plan_ && have_costmap_ && enable_)
  {
    // Checked the path obstruction on costmap
    bool obstructed = checkObstruction();

    // Retrieve a robot pose (x, y, theta)
    geometry_msgs::TransformStamped transform2 = 
    tf_buffer_.lookupTransform("map", "base_link", ros::Time(0));
    tf2::Transform position;
    tf2::fromMsg(transform2.transform, position);
    base_link_map_.pose.position.x = position.getOrigin()[0];
    base_link_map_.pose.position.y = position.getOrigin()[1];
    base_link_map_.pose.orientation.x = position.getRotation()[0];
    base_link_map_.pose.orientation.y = position.getRotation()[1];
    base_link_map_.pose.orientation.z = position.getRotation()[2];
    base_link_map_.pose.orientation.w = position.getRotation()[3];

    
    // The robot stop immediately after getting know the path has obstructed
    if(stop_distance_ == -1)
    {
      // Rising Edge
      if (!path_obstructed_ && obstructed)    //if not obstructed yet, and recent path is obstructed
      {
        // The robot doesn't rotate. Just stop it 
        // Or if the robot rotate and it's aligned now
        if (!rotate_fov_ || (rotate_fov_ && (align_ = checkPose()) ) )      
        {
          if (clearing_timeout_ == -1)
          {
            ROS_INFO("Newly obstructed. Stop robot. Waiting forever");
            // Set abort timer and stop the robot
            abort_timer_.setPeriod(ros::Duration(36000));   //wait forever (10 hours)
          }
          else
          {
            ROS_INFO("Newly obstructed. Stop robot. Countdown to abort %5.1f [s]", clearing_timeout_);
            // Set abort timer and stop the robot
            abort_timer_.setPeriod(ros::Duration(clearing_timeout_));
          }
          abort_timer_.start();
          control_timer_.start();

          // Obstruction Status reporting
          movel_seirios_msgs::ObstructionStatus report_obs;
          report_obs.reporter = "plan_inspector";
          report_obs.status = "true";
          report_obs.location = first_path_map_.pose;
          obstruction_status_pub_.publish(report_obs);

          // Plan reporting
          std_msgs::String report;
          report.data = "global_plan";
          planner_report_pub_.publish(report);

          stop_ = true;
        }
        // The robot should rotate on its place to the fov to the obstacle
        else if ( rotate_fov_ && !(align_ = checkPose()) )           
        { 
          // Make linear velocity zero while still maintain a angular velocity.
          std::cout<<"Not align with obstruction, Rotate"<<std::endl;
          // reconfigureOscillation("reconfigure");
          control_timer_.start();
        }
      }
      // check for falling edge, stop timers
      else if (path_obstructed_ && !obstructed && stop_)
      {
        ROS_INFO("Obstruction cleared before timeout. Resuming movement");
        abort_timer_.stop();
        control_timer_.stop();
        stop_ = false;
	      override_velo_ = false;

        // Obstruction Status reporting
        movel_seirios_msgs::ObstructionStatus report_obs;
        report_obs.reporter = "plan_inspector";
        report_obs.status = "false";
        report_obs.location = first_path_map_.pose;
        obstruction_status_pub_.publish(report_obs);
      }
      else if (!obstructed && !stop_ && override_velo_)
      {
        ROS_INFO("Stopping velocity take over");
        abort_timer_.stop();
        control_timer_.stop();
        override_velo_ = false;
      }
      path_obstructed_ = obstructed && stop_;
    }
    else       // stopping at distance
    {
      // Rising Edge
      if (!path_obstructed_ && obstructed)    //if not obstructed yet, and recent path is obstructed
      {
        double distance = calculateDistance();    // calculate distance
        std::cout<<"Obstacle is : "<<distance<< " meters away"<<std::endl;

        if(distance <= stop_distance_)
        {
          // The robot doesn't rotate. Just stop it 
          // Or if the robot rotate and it's aligned now
          if (!rotate_fov_ || (rotate_fov_ && (align_ = checkPose()) ) )      
          {
            if (clearing_timeout_ == -1)
            {
            ROS_INFO("Newly obstructed. Stop robot. Waiting forever");
              // Set abort timer and stop the robot
              abort_timer_.setPeriod(ros::Duration(36000));   //wait forever (10 hours)
            }
            else
            {
              ROS_INFO("Newly obstructed. Stop robot. Countdown to abort %5.1f [s]", clearing_timeout_);
              // Set abort timer and stop the robot
              abort_timer_.setPeriod(ros::Duration(clearing_timeout_));
            }
            abort_timer_.start();
            control_timer_.start();

            // Obstruction Status reporting
            movel_seirios_msgs::ObstructionStatus report_obs;
            report_obs.reporter = "plan_inspector";
            report_obs.status = "true";
            report_obs.location = first_path_map_.pose;
            obstruction_status_pub_.publish(report_obs);

            // Plan reporting
            std_msgs::String report;
            report.data = "global_plan";
            planner_report_pub_.publish(report);

            stop_ = true;
          }
          // The robot should rotate on its place to the fov to the obstacle
          else if ( rotate_fov_ && !(align_ = checkPose()) )           
          { 
            // Make linear velocity zero while still maintain a angular velocity.
            std::cout<<"Not align with obstruction, Rotate"<<std::endl;
            // reconfigureOscillation("reconfigure");
            control_timer_.start();
          }
        }
      }
      // check for falling edge, stop timers
      else if (path_obstructed_ && !obstructed && stop_)
      {
        ROS_INFO("Obstruction cleared before timeout. Resuming movement");
        abort_timer_.stop();
        control_timer_.stop();
        stop_ = false;
	      override_velo_ = false;

        // Obstruction Status reporting
        movel_seirios_msgs::ObstructionStatus report_obs;
        report_obs.reporter = "plan_inspector";
        report_obs.status = "false";
        report_obs.location = first_path_map_.pose;
        obstruction_status_pub_.publish(report_obs);

      }
      else if (!obstructed && !stop_ && override_velo_)
      {
        ROS_INFO("Stopping velocity take over");
        abort_timer_.stop();
        control_timer_.stop();
	      override_velo_ = false;
      }

      path_obstructed_ = obstructed && stop_;
    }

  }
  else if (have_plan_ && have_costmap_)
  {
    bool obstructed = checkObstruction();

    if(obstructed)
    {
        std_msgs::String report;
        report.data = "global_plan";
        planner_report_pub_.publish(report);
    }
  }
}

bool PlanInspector::checkPose()
{
  // Calculate desired pose
  double theta = angleFromVector();

  // Calculate robot pose
  double siny_cosp = 2 * (base_link_map_.pose.orientation.w * base_link_map_.pose.orientation.z + base_link_map_.pose.orientation.x * base_link_map_.pose.orientation.y);
  double cosy_cosp = 1 - 2 * (base_link_map_.pose.orientation.y * base_link_map_.pose.orientation.y + base_link_map_.pose.orientation.z * base_link_map_.pose.orientation.z);
  double yaw = std::atan2(siny_cosp, cosy_cosp);

  // Compare robot heading with the desired angle
  double tolerance = (5.0/360.0) * 2 * M_PI;

  // Angle value handler
  if (theta < -(M_PI/2) && yaw > M_PI/2)
  {
    error_ = -1 *(M_PI - (abs(theta)) + (M_PI - yaw));
  }
  else if (theta > M_PI/2 && yaw < -(M_PI/2))
  {
    error_ = (M_PI - (abs(yaw)) + (M_PI - theta));
  }
  else if (theta > yaw)
    error_ = -1 * (theta - yaw);
  else
    error_ = (yaw - theta);
  
  std::cout<<"Heading Error: "<<error_<<std::endl;

  if (fabs(error_) < tolerance)
    return true;
  else
    return false;
}

double PlanInspector::angleFromVector()
{
  double vect_path[] = { (first_path_map_.pose.position.x - base_link_map_.pose.position.x),(first_path_map_.pose.position.y - base_link_map_.pose.position.y), 0};
  double unit_vect[] = {1, 0, 0};
  
  double dot = (vect_path[0] * unit_vect[0] + vect_path[1] * unit_vect[1] + vect_path[2] * unit_vect[2]);
  double angle_cos = dot/((std::sqrt(vect_path[0] * vect_path[0] + vect_path[1] * vect_path[1] + vect_path[2] * vect_path[2]))*(std::sqrt(unit_vect[0] * unit_vect[0] + unit_vect[1] * unit_vect[1] + unit_vect[2] * unit_vect[2])));
  double cross[] = {vect_path[1] * unit_vect[2] - vect_path[2] * unit_vect[1],
                    vect_path[2] * unit_vect[0] - vect_path[0] * unit_vect[2],
                    vect_path[0] * unit_vect[1] - vect_path[1] * unit_vect[0]};
  double dot_test = (0 * cross[0] + 0 * cross[1] + 1 * cross[2]);
  
  if (dot_test > 0)
  {
    double angle = -1 * std::acos(angle_cos);
    return angle;
  }
  else
  {
    double angle = std::acos(angle_cos);
    return angle;
  }
}

double PlanInspector::calculateDistance()
{
  double distance = sqrt((base_link_map_.pose.position.x - first_path_map_.pose.position.x) * (base_link_map_.pose.position.x - first_path_map_.pose.position.x) +
                         (base_link_map_.pose.position.y - first_path_map_.pose.position.y) * (base_link_map_.pose.position.y - first_path_map_.pose.position.y) );
  return distance;
}

void PlanInspector::actionStatusCb(actionlib_msgs::GoalStatusArray msg)
{
  if (msg.status_list.size() > 0)
  {
    latest_goal_status_ = msg.status_list[0];
    have_action_status_ = true;
    if (msg.status_list[0].status != 0 && msg.status_list[0].status != 1)
    {
      abort_timer_.stop();
      control_timer_.stop();
      if (!terminal_state_)
      {
        have_plan_ = false;
        latest_plan_.poses.clear();
        terminal_state_ = true;
      }
    }
  }
}

void PlanInspector::abortTimerCb(const ros::TimerEvent& msg)
{
  ROS_INFO("Obstructed long enough. Abort action");
  if (path_obstructed_ && have_action_status_)
  {
    actionlib_msgs::GoalID action_id = latest_goal_status_.goal_id;
    action_id.stamp = ros::Time::now();
    action_cancel_pub_.publish(action_id);

    abort_timer_.stop();
    control_timer_.stop();

    have_action_status_ = false;
    have_costmap_ = false;
    have_plan_ = false;
    stop_ = false;
    path_obstructed_ = false;
  }
}

void PlanInspector::controlTimerCb(const ros::TimerEvent& msg)
{
  if ((!rotate_fov_ && path_obstructed_) || (rotate_fov_ && align_) )
  {
    // std::cout<<"Zeroing"<<std::endl;
    zero_vel_.linear.x = 0.0;
    zero_vel_.linear.y = 0.0;
    zero_vel_.linear.z = 0.0;
    zero_vel_.angular.x = 0.0;
    zero_vel_.angular.y = 0.0;
    zero_vel_.angular.z = 0.0;
    override_velo_ = true;
    zerovel_pub_.publish(zero_vel_);
  }
  else if (rotate_fov_ && !align_)
  {
    // std::cout<<"Rotate"<<std::endl;
    zero_vel_.linear.x = 0.0;
    zero_vel_.linear.y = 0.0;
    zero_vel_.linear.z = 0.0;
    zero_vel_.angular.x = 0.0;
    zero_vel_.angular.y = 0.0;
    override_velo_ = true;
    if (error_ < 0)
      zero_vel_.angular.z = rotation_speed_;
    else
      zero_vel_.angular.z = -rotation_speed_;
    zerovel_pub_.publish(zero_vel_);
    processNewInfo();
  }
}

bool PlanInspector::enableCb(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res)
{
  enable_ = req.data;
  if (enable_)
  {
    if (!reconfigure_){
      // Reconfigure the move_base params
      ROS_INFO("Plan inspector is now ON");
      saveParams();
      bool success = reconfigureParams("reconfigure");
      reconfigure_ = true;
      if (success) {
        ROS_INFO("Parameters has been reconfigured");
      }
      else {
        ROS_INFO("Failed to reconfigure parameters");
      }
    }
    else{
      ROS_INFO("Parameter has already been reconfigured.");
    }
    task_pause_status_ = false; // allow clearing pause status by toggling plan_inspector
  }
  else
  {
    ROS_INFO("Plan inspector is now OFF");
    bool success = reconfigureParams("revert");
    reconfigure_ = false;
    if (success) {
      ROS_INFO("Parameters has been reverted");
    }
    else {
      ROS_INFO("Failed to revert parameters");
    }
  }
  res.success = true;
  return true;
}

bool PlanInspector::onStopObstacleCheck(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res)
{
  if (enable_ == true) {
    res.success = true;
    res.message = "Stop obstacle enabled";
  }
  else {
    res.success = false;
    res.message = "Stop obstacle not enabled";
  }
  return true;
}

bool PlanInspector::checkObstruction()
{
  // ROS_INFO("checking for obstruction");
  if (!have_costmap_ || ! have_plan_)
    return false;

  int max_occupancy = 0;
  bool first = true;
  // march through the plan
  // ROS_INFO("march through plan, there are %lu poses", latest_plan_.poses.size());
  for (int i = 0; i < latest_plan_.poses.size(); i++)
  {
    geometry_msgs::PoseStamped pose_i = latest_plan_.poses[i];
    geometry_msgs::PoseStamped pose_costmap, pose_costmap_local;
    // ROS_INFO_STREAM(i << " original plan pose " << pose_i.pose.position);

    // transform plan pose to costmap frame
    geometry_msgs::TransformStamped transform = 
      tf_buffer_.lookupTransform(latest_costmap_.header.frame_id, 
                                 pose_i.header.frame_id, ros::Time(0));

    tf2::doTransform(pose_i.pose, pose_costmap.pose, transform);

    // calculate pose index in costmap
    geometry_msgs::TransformStamped transform_cm;
    transform_cm.child_frame_id = transform.header.frame_id;
    transform_cm.header = transform.header;
    transform_cm.header.frame_id = "local_costmap";
    
    transform_cm.transform.translation.x = -1*latest_costmap_.info.origin.position.x;
    transform_cm.transform.translation.y = -1*latest_costmap_.info.origin.position.y;
    transform_cm.transform.translation.z = -1*latest_costmap_.info.origin.position.z;

    tf2::Quaternion rot_odom_to_costmap, rot_costmap_to_odom;
    tf2::fromMsg(latest_costmap_.info.origin.orientation, rot_costmap_to_odom);
    rot_odom_to_costmap = tf2::inverse(rot_costmap_to_odom);
    transform_cm.transform.rotation = tf2::toMsg(rot_odom_to_costmap);
    
    tf2::doTransform(pose_costmap.pose, pose_costmap_local.pose, transform_cm);

    double res = latest_costmap_.info.resolution;
    int row, col;
    col = (int) (pose_costmap_local.pose.position.x / res);
    row = (int) (pose_costmap_local.pose.position.y / res);

    // ROS_INFO_STREAM("dimensions " << latest_costmap_.info.height << "/" << latest_costmap_.info.width);

    // update maximum occupancy
    if (row < latest_costmap_.info.height && col < latest_costmap_.info.width
        && row >= 0 && col >= 0)
    {
      int idx = row * latest_costmap_.info.width + col;
      int occupancy = latest_costmap_.data[idx];
      // if (occupancy > obstruction_threshold_)
      // {
      //   ROS_INFO_STREAM("pose in same frame as local costmap \n" << pose_costmap.pose.position);
      //   ROS_INFO_STREAM("pose in local costmap frame \n" << pose_costmap_local.pose.position);
      //   ROS_INFO_STREAM("row: " << row << " col: " << col << " occ: " << occupancy);
      // }
      if ((occupancy >= obstruction_threshold_) && first)
      {
          first = false;
          first_path_obs_ =  pose_i;
          geometry_msgs::TransformStamped transform1 = tf_buffer_.lookupTransform("map", first_path_obs_.header.frame_id, ros::Time(0));
          tf2::doTransform(first_path_obs_.pose, first_path_map_.pose, transform1);
      }
      if (occupancy > max_occupancy)
      {
        max_occupancy = occupancy;
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

bool PlanInspector::reconfigureParams(std::string op)
{
  dynamic_reconfigure::Reconfigure reconfigure_common, reconfigure_teb;
  dynamic_reconfigure::DoubleParameter set_planning_frequency, set_oscillation_timeout, set_weight_obstacle;
  dynamic_reconfigure::IntParameter set_max_planning_retries;
  dynamic_reconfigure::BoolParameter set_recovery_behavior_enabled, set_clearing_rotation_allowed;
  double freq, local_value, osc_time, weight_obstacle;
  int retries;
  bool rotate_behavior, clearing_rotation;
  if(op == "reconfigure"){
    ROS_INFO("Reconfiguring params...");
    freq = -1.0;
    retries = 0;
    rotate_behavior = false;
    clearing_rotation = false;
    osc_time = 0.0;
    if (use_teb_)
      weight_obstacle = 0.0;
  }
  else
  {
    ROS_INFO("Reverting the params...");
    freq = frequency_temp_;
    retries = retries_temp_;
    rotate_behavior = rotate_behavior_temp_;
    clearing_rotation = clearing_rotation_temp_;
    osc_time = osc_timeout_;
    if (use_teb_)
      weight_obstacle = weight_obstacle_temp_;
  }

  set_planning_frequency.name = "planner_frequency";
  set_planning_frequency.value = freq;
  set_max_planning_retries.name = "max_planning_retries";
  set_max_planning_retries.value = retries;
  set_recovery_behavior_enabled.name = "recovery_behavior_enabled";
  set_recovery_behavior_enabled.value = rotate_behavior;
  set_clearing_rotation_allowed.name = "clearing_rotation_allowed";
  set_clearing_rotation_allowed.value = clearing_rotation;
  set_oscillation_timeout.name = "oscillation_timeout";
  set_oscillation_timeout.value = osc_time;
  if (use_teb_)
  {
    set_weight_obstacle.name = "weight_obstacle";
    set_weight_obstacle.value = weight_obstacle;
  }

  reconfigure_common.request.config.doubles.push_back(set_planning_frequency);
  reconfigure_common.request.config.ints.push_back(set_max_planning_retries);
  reconfigure_common.request.config.bools.push_back(set_recovery_behavior_enabled);
  reconfigure_common.request.config.bools.push_back(set_clearing_rotation_allowed);
  reconfigure_common.request.config.doubles.push_back(set_oscillation_timeout);

  if(set_common_params_.call(reconfigure_common))
    {}
  else
    return false;

  if (use_teb_)
  {
    reconfigure_teb.request.config.doubles.push_back(set_weight_obstacle);
    if(set_teb_params_.call(reconfigure_teb))
      {}
    else
      return false;
  }
  return true;
}

void PlanInspector::saveParams()
{
    std::string local_planner;
    ros::NodeHandle nl("~");

    nl.getParam("/move_base/planner_frequency", frequency_temp_);
    nl.getParam("/move_base/max_planning_retries", retries_temp_);
    nl.getParam("/move_base/recovery_behavior_enabled", rotate_behavior_temp_);
    nl.getParam("/move_base/clearing_rotation_allowed", clearing_rotation_temp_);
    nl.getParam("/move_base/oscillation_timeout", osc_timeout_);
    nl.getParam("/move_base/base_local_planner", local_planner);
    if (local_planner == "teb_local_planner/TebLocalPlannerROS")
    {
      use_teb_ = true;
      nl.getParam("/move_base/TebLocalPlannerROS/weight_obstacle", weight_obstacle_temp_);
    }
}

void PlanInspector::loggerCb(rosgraph_msgs::Log msg)
{
  if (msg.msg == "DWA planner failed to produce path.")
  {
    std_msgs::String report;
    report.data = "dwa_plan";
    planner_report_pub_.publish(report);
  }
  else if (msg.msg == "TebLocalPlannerROS: trajectory is not feasible. Resetting planner...")
  {
      std_msgs::String report;
      report.data = "teb_plan";
      planner_report_pub_.publish(report);
  }
  else if (msg.msg == "Failed to get a plan.")
  {
      std_msgs::String report;
      report.data = "global_planner";
      planner_report_pub_.publish(report);
  }
}

void PlanInspector::pauseTask()
{
  if (task_pause_status_)
  {
    ROS_INFO("[plan_inspector] want to pause, but robot is already paused");
    return;
  }

  movel_seirios_msgs::GetTaskType srv;
  if (task_supervisor_type.call(srv))
  {
    ROS_INFO("[plan_inspector] type number %d", srv.response.task_type);
      
    if(srv.response.task_type != 0){
      std_msgs::Bool pause;
      pause.data = true;
      // action_pause_pub_.publish(pause);
      ros::Rate r(10.0);

      while (!task_pause_status_)
      {
        ros::spinOnce();
        ROS_INFO("[plan_inspector] pause called by path_inspector");
        action_pause_pub_.publish(pause);
        internal_pause_trigger_ = true;
        r.sleep();
      }
      task_pause_status_ = true; // preëmptive setting, otherwise teb feasibility check will override the pause
      if (nav_ac_ptr_->getState() != actionlib::SimpleClientGoalState::LOST)
        nav_ac_ptr_->cancelGoal();
    }
  }
  else
  {
      ROS_ERROR("[plan_inspector] Failed to call service get_task_type");
      // return 1;
  }
}

void PlanInspector::resumeTask()
{
  if(internal_pause_trigger_){
    std_msgs::Bool pause;
    pause.data = false;
    action_pause_pub_.publish(pause);
    task_pause_status_ = false;
    internal_pause_trigger_ = false;
  }
}

double PlanInspector::calcYaw(geometry_msgs::Pose a, geometry_msgs::Pose b)
{
  double dx, dy;
  dx = b.position.x - a.position.x;
  dy = b.position.y - a.position.y;

  return atan2(dy, dx);
}

bool PlanInspector::getRobotPose(geometry_msgs::PoseStamped& pose)
{
  try
  {
    geometry_msgs::TransformStamped transform = tf_buffer_.lookupTransform("map", "base_link", ros::Time(0));
    tf2::Transform position;
    tf2::fromMsg(transform.transform, position);
    pose.pose.position.x = position.getOrigin()[0];
    pose.pose.position.y = position.getOrigin()[1];
    pose.pose.orientation.x = position.getRotation()[0];
    pose.pose.orientation.y = position.getRotation()[1];
    pose.pose.orientation.z = position.getRotation()[2];
    pose.pose.orientation.w = position.getRotation()[3];

    pose.header.stamp = transform.header.stamp;
    pose.header.frame_id = "map";

    return true;
  }
  catch (tf2::TransformException &e)
  {
    return false;
  }
}

double PlanInspector::calculateDistance(geometry_msgs::Pose a, geometry_msgs::Pose b)
{
  double dx = b.position.x - a.position.x;
  double dy = b.position.y - a.position.y;

  return sqrt(dx*dx + dy*dy);
}

void PlanInspector::pauseStatusCb(std_msgs::Bool msg)
{
  task_pause_status_ = msg.data;
}

int main(int argc, char** argv)
{
  #ifdef MOVEL_LICENSE
    MovelLicense ml(20);
    if (!ml.login())
      return 1;
  #endif

  ros::init(argc, argv, "plan_inspector");

  PlanInspector pinspector;
  ros::spin();

  #ifdef MOVEL_LICENSE
    ml.logout();
  #endif

  return 0;
}

