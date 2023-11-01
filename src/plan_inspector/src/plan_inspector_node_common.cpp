#include <plan_inspector/plan_inspector.hpp>
#include <movel_hasp_vendor/license.h>

PlanInspector::PlanInspector()
: enable_(true), have_plan_(false), have_costmap_(false)
, is_stop_at_obstacle_enabled(true)
, have_action_status_(false), timer_active_(false), path_obstructed_(false), reconfigure_(false)
, tf_ear_(tf_buffer_), stop_(false), use_teb_(false), override_velo_(false), task_pause_status_(false)
, internal_pause_trigger_(false), have_result_(false), yaw_calculated_(false)
, use_pebble_(false)
{
  if (!setupParams())
  {
    ROS_INFO("[plan_inspector] bad parameters");
    return;
  }

  if (!setupTopics())
  {
    ROS_INFO("[plan_inspector] failed to setup topics");
    return;
  }

  abort_timer_ = nh_.createTimer(ros::Duration(clearing_timeout_),
                                 &PlanInspector::abortTimerCb, this,
                                 true, false);

  control_timer_ = nh_.createTimer(ros::Duration(1.0/control_frequency_),
                                   &PlanInspector::controlTimerCb, this,
                                   false, false);

  // partial_blockage_timer_ = nh_.createTimer(ros::Duration(1.0),
  //                                           &PlanInspector::partialBlockageTimerCb, this,
  //                                           false, false);

  set_common_params_.waitForExistence();
  if (use_teb_)
    set_teb_params_.waitForExistence();
  if (use_pebble_)
    set_pebble_params_.waitForExistence();

  if (enable_){
    bool success = reconfigureParams("reconfigure");
    reconfigure_ = true;
  }

  // redis
  opts_.host = redis_host_;
  opts_.port = stoi(redis_port_);
  opts_.socket_timeout = std::chrono::milliseconds(socket_timeout_);

  redis_stop_at_obstacle_lim_key_ = "stop_at_obstacle_enabled";

  auto redis = sw::redis::Redis(opts_);
  auto sub = redis.subscriber();

  auto val_stop_at_obs = redis.get(redis_stop_at_obstacle_lim_key_);
  if(*val_stop_at_obs=="true")
    is_stop_at_obstacle_enabled = true;
  else if(*val_stop_at_obs=="false")
    is_stop_at_obstacle_enabled = false;
  
  enable_ = is_stop_at_obstacle_enabled;
}

bool PlanInspector::setupParams()
{
  ros::NodeHandle nl("~");

  if (nl.hasParam("enable"))
    nl.getParam("enable", enable_);

  action_server_name_ = "/move_base";
  if (nl.hasParam("action_server_name"))
    nl.getParam("action_server_name", action_server_name_);

  plan_topic_ = "/move_base/GlobalPlanner/plan";
  if (nl.hasParam("move_base_params/base_global_planner")) {
    std::string planner;    
    nl.getParam("move_base_params/base_global_planner", planner);
    pluginlib::ClassLoader<nav_core::BaseGlobalPlanner> bgp_loader_{"nav_core", "nav_core::BaseGlobalPlanner"};
    plan_topic_ = "/move_base/" + bgp_loader_.getName(planner) + "/plan";
  }

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

  angular_tolerance_ = 0.1754;
  if(nl.hasParam("angular_tolerance"))
    nl.getParam("angular_tolerance", angular_tolerance_);

  enable_replan_ = false;
  if(nl.hasParam("enable_replan"))
    nl.getParam("enable_replan", enable_replan_);

  enable_partial_blockage_replan_ = false;
  if(nl.hasParam("enable_partial_blockage_replan"))
    nl.getParam("enable_partial_blockage_replan", enable_partial_blockage_replan_);

  partial_blockage_path_length_threshold_ = 30.0;
  if(nl.hasParam("partial_blockage_path_length_threshold"))
    nl.getParam("partial_blockage_path_length_threshold", partial_blockage_path_length_threshold_);

  //redis
  if (nl.hasParam("redis_host"))
    nl.getParam("redis_host", redis_host_);
  if (nl.hasParam("redis_port"))
    nl.getParam("redis_port", redis_port_);

  if (nl.hasParam("socket_timeout"))
    nl.getParam("socket_timeout", socket_timeout_);
  else
    socket_timeout_= 1 ;
    
  saveParams();
  return true;
}

bool PlanInspector::setupTopics()
{
  //TODO: use local nodehandle?
  
  // Subscribed topic
  plan_sub_ = nh_.subscribe(plan_topic_, 1, &PlanInspector::pathCb, this);
  costmap_sub_ = nh_.subscribe(costmap_topic_, 1, &PlanInspector::costmapCb, this); 

  // Goal topic
  action_goal_sub_ = nh_.subscribe("/task_supervisor/goal", 1, &PlanInspector::actionGoalCb, this);
  action_result_sub_ = nh_.subscribe("/task_supervisor/result", 1, &PlanInspector::actionResultCb, this);
  pause_status_sub_ = nh_.subscribe("/task_supervisor/pause_status", 1, &PlanInspector::pauseStatusCb, this);
  action_pause_pub_ = nh_.advertise<std_msgs::Bool>("/task_supervisor/pause", 1); // only task_supervisor has pause, so no messing around with action_server_name_ here
  action_abort_pub_ = nh_.advertise<std_msgs::String>("/task_supervisor/abort", 1);

  // Velocity topic
  zerovel_pub_ = nh_.advertise<geometry_msgs::Twist>(cmd_vel_topic_, 1);

  // Enabler
  enable_sub_ = nh_.advertiseService("enable_plan_inspector", &PlanInspector::enableCb, this);
  reconfig_srv_ = nh_.advertiseService("change_reconfig", &PlanInspector::reconfig_cb, this);

  // Checker
  stop_obstacle_checker_ = nh_.advertiseService("/stop_obstacle_check", &PlanInspector::onStopObstacleCheck, this);

  // Dynamic Reconfigure
  set_pebble_params_ = nh_.serviceClient<dynamic_reconfigure::Reconfigure>("/move_base/PebbleLocalPlanner/set_parameters");
  set_common_params_ = nh_.serviceClient<dynamic_reconfigure::Reconfigure>(config_topic_);
  set_teb_params_ = nh_.serviceClient<dynamic_reconfigure::Reconfigure>("/move_base/TebLocalPlannerROS/set_parameters");
  task_supervisor_type_ = nh_.serviceClient<movel_seirios_msgs::GetTaskType>("/task_supervisor/get_task_type");
  // set_stop_obs_mb_ = nh_.serviceClient<std_srvs::SetBool>("/move_base/stop_at_obstacle");
  
  // Reporting Topics
  obstruction_status_pub_ = nh_.advertise<movel_seirios_msgs::ObstructionStatus>("/obstruction_status",1);
  logger_sub_ = nh_.subscribe("/rosout", 1, &PlanInspector::loggerCb, this);
  planner_report_pub_ = nh_.advertise<std_msgs::String>("/planner_report",1);

  // dynamic reconfigure for internal params
  dyn_config_cb_ = boost::bind(&PlanInspector::dynamicReconfigureCb, this, _1, _2);
  dyn_config_srv_.setCallback(dyn_config_cb_);

  // move_base action client
  nav_ac_ptr_ = std::make_shared<actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> >("move_base", true);
  ROS_INFO("[plan_inspector] wait for move_base action server");
  if(!nav_ac_ptr_->waitForServer(ros::Duration(60.0)))
  {
    ROS_INFO("[plan_inspector] failed to connect to move_base action server");
    return false;
  }

  // partial/full blockage check
  make_sync_plan_client_ = nh_.serviceClient<nav_msgs::GetPlan>("/planner_utils/make_sync_plan");
  partial_blockage_check_pub_ = nh_.advertise<nav_msgs::Path>("partial_blockage_check", 1);

  return true;
}

void PlanInspector::dynamicReconfigureCb(plan_inspector::PlanInspectorConfig &config, uint32_t level)
{
  ROS_INFO("[plan_inspector] Dynamic reconfigure");
  clearing_timeout_ = config.clearing_timeout;
  enable_replan_ = config.enable_replan;
  stop_distance_ = config.stop_distance;
}

bool PlanInspector::reconfig_cb(movel_seirios_msgs::StopReconfig::Request &req, movel_seirios_msgs::StopReconfig::Response &res)
{
  if(req.planner_frequency > 0.0)
  {
    reconfigureParams("revert");
  }
  else
  {
    reconfigureParams("reconfigure");
  }
  ROS_INFO("[plan_inspector] oscillation %lf  frequency %lf", req.oscillation_timeout,req.planner_frequency);
  return true;
}

void PlanInspector::pathCb(nav_msgs::Path msg)
{
  // ROS_INFO("new path");

  // don't overwrite plan if it's not from the active task
  if (task_pause_status_)
    return;

  latest_plan_ = msg;
  if(latest_plan_.poses.size()>0)  
  {  
    have_plan_ = true;
    processNewInfo();
  }
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

    // std::cout << "BASE_LINK_MAP" << base_link_map_ << std::endl;

    // report regardless of enable
    if (obstructed)
    {
      std_msgs::String report;
      report.data = "global_plan";
      planner_report_pub_.publish(report);
    }

    if (enable_)
    {
      geometry_msgs::PoseStamped robot_pose;
      if (!getRobotPose(robot_pose))
        return;
      if (stop_distance_ > 0 && obstructed && calculateDistance(robot_pose.pose, first_path_map_.pose) > stop_distance_)
        obstructed = false;
      // rising edge
      if (obstructed && !path_obstructed_)
      {
        ROS_INFO("[plan_inspector] obstacle on path");
        // path_obstructed_ = true;

        // report obstruction
        movel_seirios_msgs::ObstructionStatus report_obs;
        report_obs.reporter = "plan_inspector";
        report_obs.status = "true";
        report_obs.location = first_path_map_.pose;
        obstruction_status_pub_.publish(report_obs);

        // stop at obstacle
        BlockageType blockage_check = BlockageType::FULL;   // default to stop at obstacle
        // partial/full blockage
        if (enable_partial_blockage_replan_) {
          ROS_INFO("[plan_inspector] Checking for partial/full blockage");
          blockage_check = checkPartialBlockage();
        }
        if (blockage_check == BlockageType::PARTIAL) {
          // do nothing, let move base replan if it wants to
          ROS_INFO("[plan_inspector] Partial blockage detected, allow replanning");
        }
        else if (blockage_check == BlockageType::FULL) {   // default
          ROS_INFO("[plan_inspector] Full blockage detected, waiting for clearing timeout");
          path_obstructed_ = true;
          
          // stop immediately
          pauseTask();

          // orient robot
          if (rotate_fov_ && !yaw_calculated_)
          {
            yaw_calculated_ = true;
            ROS_INFO("[plan_inspector] face to obstacle");
            target_yaw_ = calcYaw(robot_pose.pose, first_path_map_.pose);
            control_timer_.start();
          }
          else
          {
            ROS_INFO("[plan_inspector] immediate stop");
            double timeout = clearing_timeout_;
            if (clearing_timeout_ < 0.)
              timeout = 24 * 3600;

            abort_timer_.setPeriod(ros::Duration(timeout));
            abort_timer_.start();

            stop_ = true;
          }
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
        // partial_blockage_timer_.stop();
        yaw_calculated_ = false;
        stop_ = false;

        // Wait for global costmap to clear before resume task
        ros::Duration(1.0).sleep();
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

void PlanInspector::actionGoalCb(movel_seirios_msgs::RunTaskListActionGoal msg)
{
  if (msg.goal.task_list.tasks[0].type == 3)
  {
    have_result_ = false;
  }
}

void PlanInspector::actionResultCb(movel_seirios_msgs::RunTaskListActionResult msg)
{
  have_result_ = true;
  abort_timer_.stop();
  control_timer_.stop();
  // partial_blockage_timer_.stop();
  latest_plan_.poses.clear();
  yaw_calculated_ = false;
  have_plan_ = false;
  have_costmap_ = false;
  stop_ = false;
  path_obstructed_ = false;
}

void PlanInspector::abortTimerCb(const ros::TimerEvent& msg)
{
  latest_plan_.poses.clear();
  abort_timer_.stop();
  control_timer_.stop();
  // partial_blockage_timer_.stop();
  yaw_calculated_ = false;
  have_costmap_ = false;
  have_plan_ = false;
  stop_ = false;

  if(!enable_replan_)
  {
    ROS_INFO("[plan_inspector] Obstructed long enough. Abort action");
    if (path_obstructed_ && !have_result_)
    {
      // Cancel task
      std_msgs::String action_abort_msg;
      action_abort_msg.data = "[plan_inspector] Obstructed long enough. Abort action";
      action_abort_pub_.publish(action_abort_msg);
      path_obstructed_ = false;
    }
  }
  else
  {
    ROS_INFO("[plan_inspector] Obstructed long enough. Replan path");
    if (path_obstructed_ && !have_result_)
    {
      path_obstructed_ = false;

      // Get new plan
      resumeTask();
    }
  }
}

void PlanInspector::controlTimerCb(const ros::TimerEvent& msg)
{
  geometry_msgs::PoseStamped current_pose;
  getRobotPose(current_pose);
  double current_yaw = tf::getYaw(current_pose.pose.orientation);
  if (fabs(target_yaw_ - current_yaw) > angular_tolerance_)
  {
    if((target_yaw_ > current_yaw && fabs(target_yaw_ - current_yaw) < M_PI) || (target_yaw_ < current_yaw && fabs(target_yaw_ - current_yaw) > M_PI))
      zero_vel_.angular.z = rotation_speed_;
    else
      zero_vel_.angular.z = -rotation_speed_;
    zerovel_pub_.publish(zero_vel_);
  }
  else
  {
    ROS_INFO("[plan_inspector] Stop rotation");
    zero_vel_.angular.z = 0.0;
    zerovel_pub_.publish(zero_vel_);

    double timeout = clearing_timeout_;
    if (clearing_timeout_ < 0.)
      timeout = 24 * 3600;

    abort_timer_.setPeriod(ros::Duration(timeout));
    abort_timer_.start();
    stop_ = true;
    yaw_calculated_ = false;
    control_timer_.stop();
  }
}

void PlanInspector::partialBlockageTimerCb(const ros::TimerEvent& msg)
{
  // ROS_INFO("[plan_inspector] Checking for partial/full blockage");
  // BlockageType blockage_check = checkPartialBlockage();
  // if (blockage_check == BlockageType::PARTIAL) {
  //   ROS_INFO("[plan_inspector] Partial blockage detected, replanning");
  //   resumeTask();
  // }
  // else if (blockage_check == BlockageType::FULL) {
  //   ROS_INFO("[plan_inspector] Full blockage detected, waiting for clearing timeout");
  // }
}

PlanInspector::BlockageType PlanInspector::checkPartialBlockage()
{
  using VecPS = std::vector<geometry_msgs::PoseStamped>;
  // find remainder of current path
  VecPS latest_path = latest_plan_.poses;   // make a copy
  // get nearest point in path
  geometry_msgs::PoseStamped robot_pose;
  getRobotPose(robot_pose);
  int nearest_idx = 0;
  double nearest_dist = std::numeric_limits<double>::infinity();
  for (int i = 0; i < latest_path.size(); i++) {
    double dist = calculateDistance(robot_pose.pose, latest_path[i].pose);
    if (dist < nearest_dist) {
      nearest_dist = dist;
      nearest_idx = i;
    }
  }
  // remainder of current path
  VecPS latest_path_remainder(latest_path.begin() + nearest_idx, latest_path.end());
  // get new path from current robot pose
  nav_msgs::GetPlan srv{};
  srv.request.start = robot_pose;   // current pose
  srv.request.goal = latest_path.back();   // goal
  try{
  	if (!make_sync_plan_client_.call(srv)) {
    		ROS_WARN("[plan_inspector] checkPartialBlockage service call to make_sync_plan_client failed");
    	return BlockageType::FAILED;
  	}
     }catch(...)
  {
     ROS_WARN("[plan_inspector] checkPartialBlockage service call to make_sync_plan_client failed");
     return BlockageType::FAILED;     
  }
  const VecPS& new_path = srv.response.plan.poses;
  partial_blockage_check_pub_.publish(srv.response.plan);
  // partial/full blockage detection
  // sanity check
  if (latest_path_remainder.size() < 2 ||
      new_path.size() < 2)
  {
    ROS_WARN("[plan_inspector] checkPartialBlockage no path found");
    return BlockageType::FAILED;
  }
  // compare path lengths
  auto f_path_dist = [&, this](const VecPS& path) -> double {
    double dist = 0.0;
    for (int i = 0; i < path.size()-1; i++)
      dist += calculateDistance(path[i].pose, path[i+1].pose);
    return dist;
  };
  double dist_latest_path_remainder = f_path_dist(latest_path_remainder);
  double dist_new_path = f_path_dist(new_path);
  // double path_diff = std::abs(dist_latest_path_remainder - dist_new_path);
  double path_diff = dist_new_path - dist_latest_path_remainder;
  ROS_INFO("[plan_inspector] checkPartialBlockage path diff: %f", path_diff);
  bool is_partial_blockage = path_diff < partial_blockage_path_length_threshold_;   // partial if new path is shorter
  return is_partial_blockage ? BlockageType::PARTIAL : BlockageType::FULL;
}


bool PlanInspector::enableCb(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res)
{
  if(req.data && enable_)
    res.message = "plan_inspector already enabled";
  else if(req.data && !enable_)
    res.message = "plan_inspector enabled";
  else if(!req.data && !enable_)
    res.message = "plan_inspector already disabled";
  else if(!req.data && enable_)
    res.message = "plan_inspector disabled";

  enable_ = req.data;
  // std_srvs::SetBool set_req;
  // set_req.request.data = enable_;
  // ros::service::waitForService("/move_base/stop_at_obstacle",ros::Duration(2.0));
  //   set_stop_obs_mb_.call(set_req);
  if (enable_)
  {
    if (!reconfigure_){
      // Reconfigure the move_base params
      ROS_INFO("Plan inspector is now ON");
      saveParams();
      bool success = reconfigureParams("reconfigure");
      reconfigure_ = true;
      if (success) {
        ROS_INFO("[plan_inspector] Parameters has been reconfigured");
      }
      else {
        ROS_INFO("[plan_inspector] Failed to reconfigure parameters");
      }
    }
    else{
      ROS_INFO("[plan_inspector] Parameter has already been reconfigured.");
    }
    task_pause_status_ = false; // allow clearing pause status by toggling plan_inspector
  }
  else
  {
    ROS_INFO("Plan inspector is now OFF");
    bool success = reconfigureParams("revert");
    reconfigure_ = false;
    if (success) {
      ROS_INFO("[plan_inspector] Parameters has been reverted");
    }
    else {
      ROS_INFO("[plan_inspector] Failed to revert parameters");
    }
  }

  res.success = true;
  return true;
}

bool PlanInspector::onStopObstacleCheck(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
  if (enable_) {
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

  // Find index of nearest waypoint in path plan
  geometry_msgs::PoseStamped robot_pose;
  getRobotPose(robot_pose);
  int current_index = 0;
  double nearest_distance = sqrt(pow((robot_pose.pose.position.x - latest_plan_.poses[0].pose.position.x), 2) +
                                 pow((robot_pose.pose.position.y - latest_plan_.poses[0].pose.position.y), 2));

  for (int i = 1; i < latest_plan_.poses.size(); i++)
  {
    double distance = sqrt(pow((robot_pose.pose.position.x - latest_plan_.poses[i].pose.position.x), 2) +
                           pow((robot_pose.pose.position.y - latest_plan_.poses[i].pose.position.y), 2));
    if(distance < nearest_distance)
    {
      nearest_distance = distance;
      current_index = i;
    }
  }

  // march through the plan
  // ROS_INFO("march through plan, there are %lu poses", latest_plan_.poses.size());
  for (int i = current_index; i < latest_plan_.poses.size(); i++)
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
        max_occupancy = occupancy;   // TODO: can have early return if obstruction detected?
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
  bool rotate_behavior, clearing_rotation, obs_check, obs_avoid;
  if(op == "reconfigure"){
    ROS_INFO("[plan_inspector] Reconfiguring params...");
    freq = -1.0;
    retries = 0;
    rotate_behavior = false;
    clearing_rotation = false;
    if(use_pebble_)
    {
      obs_check = true;
      obs_avoid = false;
    }
    osc_time = 0.0;
    if (use_teb_)
      weight_obstacle = 0.0;
  }
  else
  {
    ROS_INFO("[plan_inspector] Reverting the params...");
    freq = frequency_temp_;
    retries = retries_temp_;
    rotate_behavior = rotate_behavior_temp_;
    clearing_rotation = clearing_rotation_temp_;
    osc_time = osc_timeout_;
    if (use_teb_)
      weight_obstacle = weight_obstacle_temp_;
    if (use_pebble_)
    {
      obs_check = false;
      obs_avoid = true;
    }
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

  /*if (use_teb_)
  {
    reconfigure_teb.request.config.doubles.push_back(set_weight_obstacle);
    if(set_teb_params_.call(reconfigure_teb))
      {}
    else
      return false;
  }*/
  if (use_pebble_)
  {
    dynamic_reconfigure::Reconfigure pebble_reconfigure;
    dynamic_reconfigure::BoolParameter set_obs_avoid;
    set_obs_avoid.name = "local_obstacle_avoidance";
    set_obs_avoid.value = obs_avoid;
    pebble_reconfigure.request.config.bools.push_back(set_obs_avoid);

    if(set_pebble_params_.call(pebble_reconfigure))
      { ROS_INFO("[plan_inspector] Pebble plan set params..."); }
    else{
      ROS_ERROR("[plan_inspector] Failed Pebble plan set params...");
      return false;
    }
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
    else if(local_planner == "pebble_local_planner::PebbleLocalPlanner")
    {
      use_pebble_ = true;
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
  if (task_supervisor_type_.call(srv))
  {
    ROS_INFO("[plan_inspector] type number %d", srv.response.task_type);
      
    if(srv.response.task_type != 0){
      std_msgs::Bool pause;
      pause.data = true;
      // action_pause_pub_.publish(pause);
      ros::Rate r(10.0);

      while (!task_pause_status_ && !have_result_)
      {
        ros::spinOnce();
        ROS_INFO("[plan_inspector] pause called by path_inspector");
        action_pause_pub_.publish(pause);
        internal_pause_trigger_ = true;
        r.sleep();
      }
      task_pause_status_ = true; // preëmptive setting, otherwise teb feasibility check will override the pause
    }
    // // partial blockage
    // if (enable_partial_blockage_replan_)
    //   partial_blockage_timer_.start();
  }
  else
  {
      ROS_ERROR("[plan_inspector] Failed to call service get_task_type");
      // return 1;
  }
}

void PlanInspector::resumeTask()
{
  // if(internal_pause_trigger_){
  if(task_pause_status_) {
    std_msgs::Bool pause;
    pause.data = false;
    action_pause_pub_.publish(pause);
    task_pause_status_ = false;   // TODO: remove? (might cause data race on repeated pasue/resume)
    internal_pause_trigger_ = false;
    // reset timers
    abort_timer_.stop();
    control_timer_.stop();
    // partial_blockage_timer_.stop();
  }
  else {
    ROS_WARN("[plan_inspector] Resume task called when task is not paused");
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
    MovelLicense ml;
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