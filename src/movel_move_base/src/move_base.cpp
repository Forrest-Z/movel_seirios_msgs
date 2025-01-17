#include <movel_move_base/move_base.h>
#include <move_base_msgs/RecoveryStatus.h>
#include <geometry_msgs/Twist.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <boost/algorithm/string.hpp>
#include <boost/thread.hpp>
#include <cmath>

using BlockageType = PlanInspector::BlockageType;

namespace move_base
{
MoveBase::MoveBase(tf2_ros::Buffer& tf)
  : tf_(tf)
  , as_(NULL)
  , planner_costmap_ros_(NULL)
  , controller_costmap_ros_(NULL)
  , bgp_loader_("nav_core", "nav_core::BaseGlobalPlanner")
  , blp_loader_("nav_core", "nav_core::BaseLocalPlanner")
  , recovery_loader_("nav_core", "nav_core::RecoveryBehavior")
  , planner_plan_(NULL)
  , latest_plan_(NULL)
  , controller_plan_(NULL)
  , runPlanner_(false)
  , setup_(false)
  , p_freq_change_(false)
  , c_freq_change_(false)
  , new_global_plan_(false)
  , plan_obstructed_(false)
  , has_valid_control_(false)
  , stop_caused_by_obstacle_(false)
  , use_pebble_(false)
  , use_teb_(false)
{
  as_ = new MoveBaseActionServer(ros::NodeHandle(), "move_base", boost::bind(&MoveBase::executeCb, this, _1), false);

  ros::NodeHandle private_nh("~");
  ros::NodeHandle nh;

  recovery_trigger_ = PLANNING_R;

  // get some parameters that will be global to the move base node
  std::string global_planner, local_planner;
  private_nh.param("base_global_planner", global_planner, std::string("navfn/NavfnROS"));
  private_nh.param("base_local_planner", local_planner, std::string("base_local_planner/TrajectoryPlannerROS"));
  private_nh.param("global_costmap/robot_base_frame", robot_base_frame_, std::string("base_link"));
  private_nh.param("global_costmap/global_frame", global_frame_, std::string("map"));
  private_nh.param("planner_frequency", planner_frequency_, 0.0);
  private_nh.param("controller_frequency", controller_frequency_, 20.0);
  private_nh.param("planner_patience", planner_patience_, 5.0);
  private_nh.param("controller_patience", controller_patience_, 15.0);
  private_nh.param("max_planning_retries", max_planning_retries_, -1);  // disabled by default

  private_nh.param("oscillation_timeout", oscillation_timeout_, 0.0);
  private_nh.param("oscillation_distance", oscillation_distance_, 0.5);

  // parameters of make_plan service
  private_nh.param("make_plan_clear_costmap", make_plan_clear_costmap_, true);
  private_nh.param("make_plan_add_unreachable_goal", make_plan_add_unreachable_goal_, true);

  // movel_move_base parameters
  private_nh.param("stop_at_obstacle", stop_at_obstacle_, false);
  private_nh.param("stop_at_obstacle_distance", stop_at_obstacle_distance_, 1.0);
  private_nh.param("obstacle_clearing_timeout", clearing_timeout_, 30.0);
  private_nh.param("allow_replan_after_timeout", allow_replan_after_timeout_, false);
  private_nh.param("allow_partial_blockage_replan", allow_partial_blockage_replan_, false);
  private_nh.param("allow_recovery_during_timeout", allow_recovery_during_timeout_, false);
  private_nh.param("use_circumscribed_cost_as_obstruction_threshold", use_circumscribed_cost_as_obstruction_threshold_, true);

  // plan inspector parameters
  private_nh.param("plan_inspector/obstruction_threshold", obstruction_threshold_, 99);
  private_nh.param("plan_inspector/partial_blockage_path_length_threshold", partial_blockage_path_length_threshold_,
                   30.0);

  // set up plan triple buffer
  planner_plan_ = new std::vector<geometry_msgs::PoseStamped>();
  latest_plan_ = new std::vector<geometry_msgs::PoseStamped>();
  controller_plan_ = new std::vector<geometry_msgs::PoseStamped>();

  // set up the planner's thread
  planner_thread_ = new boost::thread(boost::bind(&MoveBase::planThread, this));

  // for commanding the base
  vel_pub_ = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
  current_goal_pub_ = private_nh.advertise<geometry_msgs::PoseStamped>("current_goal", 0);

  ros::NodeHandle action_nh("move_base");
  action_goal_pub_ = action_nh.advertise<move_base_msgs::MoveBaseActionGoal>("goal", 1);
  recovery_status_pub_ = action_nh.advertise<move_base_msgs::RecoveryStatus>("recovery_status", 1);

  // we'll provide a mechanism for some people to send goals as PoseStamped messages over a topic
  // they won't get any useful information back about its status, but this is useful for tools
  // like nav_view and rviz
  ros::NodeHandle simple_nh("move_base_simple");
  goal_sub_ = simple_nh.subscribe<geometry_msgs::PoseStamped>("goal", 1, boost::bind(&MoveBase::goalCB, this, _1));

  // we'll assume the radius of the robot to be consistent with what's specified for the costmaps
  private_nh.param("local_costmap/inscribed_radius", inscribed_radius_, 0.325);
  private_nh.param("local_costmap/circumscribed_radius", circumscribed_radius_, 0.46);
  private_nh.param("clearing_radius", clearing_radius_, circumscribed_radius_);
  private_nh.param("conservative_reset_dist", conservative_reset_dist_, 3.0);

  private_nh.param("shutdown_costmaps", shutdown_costmaps_, false);
  private_nh.param("clearing_rotation_allowed", clearing_rotation_allowed_, true);
  private_nh.param("recovery_behavior_enabled", recovery_behavior_enabled_, true);

  // create the ros wrapper for the planner's costmap... and initializer a pointer we'll use with the underlying map
  planner_costmap_ros_ = new costmap_2d::Costmap2DROS("global_costmap", tf_);
  planner_costmap_ros_->pause();

  // initialize the global planner
  try
  {
    planner_ = bgp_loader_.createInstance(global_planner);
    planner_->initialize(bgp_loader_.getName(global_planner), planner_costmap_ros_);
  }
  catch (const pluginlib::PluginlibException& ex)
  {
    ROS_FATAL(
        "Failed to create the %s planner, are you sure it is properly registered and that the containing library is "
        "built? Exception: %s",
        global_planner.c_str(), ex.what());
    exit(1);
  }

  // create the ros wrapper for the controller's costmap... and initializer a pointer we'll use with the underlying map
  controller_costmap_ros_ = new costmap_2d::Costmap2DROS("local_costmap", tf_);
  controller_costmap_ros_->pause();

  // create a local planner
  try
  {
    tc_ = blp_loader_.createInstance(local_planner);
    ROS_INFO("Created local_planner %s", local_planner.c_str());
    tc_->initialize(blp_loader_.getName(local_planner), &tf_, controller_costmap_ros_);
  }
  catch (const pluginlib::PluginlibException& ex)
  {
    ROS_FATAL(
        "Failed to create the %s planner, are you sure it is properly registered and that the containing library is "
        "built? Exception: %s",
        local_planner.c_str(), ex.what());
    exit(1);
  }

  // Start actively updating costmaps based on sensor data
  planner_costmap_ros_->start();
  controller_costmap_ros_->start();

  // advertise a service for getting a plan
  make_plan_srv_ = private_nh.advertiseService("make_plan", &MoveBase::planService, this);

  // advertise a service for clearing the costmaps
  clear_costmaps_srv_ = private_nh.advertiseService("clear_costmaps", &MoveBase::clearCostmapsService, this);

  stop_obstacle_srv_ = private_nh.advertiseService("stop_at_obstacle", &MoveBase::stopObstacleService, this);
  plan_inspector_srv_ = private_nh.advertiseService("/enable_plan_inspector", &MoveBase::enablePlanInspector, this);
  
  // Checker
  stop_obstacle_checker_ = private_nh.advertiseService("/stop_obstacle_check", &MoveBase::onStopObstacleCheck, this);

  // if we shutdown our costmaps when we're deactivated... we'll do that now
  if (shutdown_costmaps_)
  {
    ROS_DEBUG_NAMED("move_base", "Stopping costmaps initially");
    planner_costmap_ros_->stop();
    controller_costmap_ros_->stop();
  }

  // load any user specified recovery behaviors, and if that fails load the defaults
  if (!loadRecoveryBehaviors(private_nh))
  {
    loadDefaultRecoveryBehaviors();
  }

  // initially, we'll need to make a plan
  state_ = PLANNING;

  // we'll start executing recovery behaviors at the beginning of our list
  recovery_index_ = 0;

  // we're all set up now so we can start the action server
  as_->start();

  redisInit(); // set before dynamic reconfigure, so we can update the stop at obstacle on dynamic reconfigure based on redis var

  dsrv_ = new dynamic_reconfigure::Server<movel_move_base::MoveBaseConfig>(ros::NodeHandle("~"));
  dynamic_reconfigure::Server<movel_move_base::MoveBaseConfig>::CallbackType cb =
      boost::bind(&MoveBase::reconfigureCB, this, _1, _2);
  dsrv_->setCallback(cb);

  // movel_move_base specifics

  obstruction_status_pub_ = nh.advertise<movel_seirios_msgs::ObstructionStatus>("/obstruction_status", 1);
  
  if (use_circumscribed_cost_as_obstruction_threshold_)
  {
    updateCircumscribedCostThreshold();
    ROS_INFO("[movel_move_base] obstruction_threshold overriden by the calculated circumscribed cost");
  }

  plan_inspector_ = use_circumscribed_cost_as_obstruction_threshold_ ? 
    std::make_unique<PlanInspector>(&tf_, circumscribed_cost_threshold_, partial_blockage_path_length_threshold_) :
    std::make_unique<PlanInspector>(&tf_, obstruction_threshold_, partial_blockage_path_length_threshold_);
  
  // dynamic reconfigure - change everytime stop at obstacle updated
  set_pebble_params_ = private_nh.serviceClient<dynamic_reconfigure::Reconfigure>("/move_base/PebbleLocalPlanner/set_parameters");
  set_teb_params_ = private_nh.serviceClient<dynamic_reconfigure::Reconfigure>("/move_base/TebLocalPlannerROS/set_parameters");
  set_move_base_param_ = private_nh.serviceClient<dynamic_reconfigure::Reconfigure>("/move_base/set_parameters");
  
  //save params
  saveParams();

  if (use_pebble_)
    set_pebble_params_.waitForExistence();

  set_move_base_param_.waitForExistence();

  // reconfigureParams(stop_at_obstacle_); //blocking

  if (stop_at_obstacle_) // if true, set the local planner to not do replanning
  {
    planner_frequency_= -1.0;
    max_planning_retries_= 0;
    recovery_behavior_enabled_= false;
    clearing_rotation_allowed_= false;
    oscillation_timeout_= 0.0;
  }

  ROS_INFO("Move Base Ready");
}

void MoveBase::saveParams()
{
  std::string local_planner_name;
  ros::NodeHandle nl("~");

  nl.getParam("/move_base/base_local_planner", local_planner_name);

  if (local_planner_name == "teb_local_planner/TebLocalPlannerROS")
  {
    use_teb_ = true;
    nl.getParam("/move_base/TebLocalPlannerROS/weight_obstacle", weight_obstacle_temp_);
  }
  else if(local_planner_name == "pebble_local_planner::PebbleLocalPlanner")
  {
    use_pebble_ = true;
  }

  planner_frequency_temp_ = planner_frequency_;
  max_planning_retries_temp_ = max_planning_retries_;
  recovery_behavior_enabled_temp_ = recovery_behavior_enabled_;
  clearing_rotation_allowed_temp_ = clearing_rotation_allowed_;
  oscillation_timeout_temp_ = oscillation_timeout_;
}

void MoveBase::reconfigureParams(bool stop_at_obstacle_state)
{
  dynamic_reconfigure::DoubleParameter set_weight_obstacle;
  double weight_obstacle;
  bool obs_check, obs_avoid, stop_obstacle_param;

  if (stop_at_obstacle_state) // if true, set the local planner to not do replanning
  {
    ROS_INFO("Reconfigure Params - Stop at obstacle");

    if(use_pebble_)
    {
      obs_check = true;
      obs_avoid = false;
    }
    
    planner_frequency_= -1.0;
    max_planning_retries_= 0;
    recovery_behavior_enabled_= false;
    clearing_rotation_allowed_= false;
    oscillation_timeout_= 0.0;
  }
  else // if do obstacle avoidance
  {
    if (use_pebble_)
    {
      obs_check = false;
      obs_avoid = true;
    }

    planner_frequency_= planner_frequency_temp_;
    max_planning_retries_ = max_planning_retries_temp_;
    recovery_behavior_enabled_= recovery_behavior_enabled_temp_;
    clearing_rotation_allowed_= clearing_rotation_allowed_temp_;
    oscillation_timeout_= oscillation_timeout_temp_;
  }
  stop_at_obstacle_ = stop_at_obstacle_state;

  auto redis = sw::redis::Redis(opts_);
  try
  {
    if (stop_at_obstacle_)
      redis.set(redis_stop_at_obstacle_lim_key_, "true");
    else
      redis.set(redis_stop_at_obstacle_lim_key_, "false");
  }
  catch (const sw::redis::Error& e)
  {
    ROS_ERROR_STREAM("[movel_move_base] Failed to set stop_at_obstacle on redis: " << e.what());
  }

  dynamic_reconfigure::Reconfigure move_base_reconfigure;
  dynamic_reconfigure::BoolParameter set_stop_at_obs;
  dynamic_reconfigure::DoubleParameter set_planning_frequency, set_oscillation_timeout;
  dynamic_reconfigure::IntParameter set_max_planning_retries;
  dynamic_reconfigure::BoolParameter set_recovery_behavior_enabled, set_clearing_rotation_allowed;

  set_stop_at_obs.name = "stop_at_obstacle";
  set_stop_at_obs.value = stop_at_obstacle_;
  set_planning_frequency.name = "planner_frequency";
  set_planning_frequency.value = planner_frequency_;
  set_max_planning_retries.name = "max_planning_retries";
  set_max_planning_retries.value = max_planning_retries_;
  set_recovery_behavior_enabled.name = "recovery_behavior_enabled";
  set_recovery_behavior_enabled.value = recovery_behavior_enabled_;
  set_clearing_rotation_allowed.name = "clearing_rotation_allowed";
  set_clearing_rotation_allowed.value = clearing_rotation_allowed_;
  set_oscillation_timeout.name = "oscillation_timeout";
  set_oscillation_timeout.value = oscillation_timeout_;

  move_base_reconfigure.request.config.bools.push_back(set_stop_at_obs);
  move_base_reconfigure.request.config.doubles.push_back(set_planning_frequency);
  move_base_reconfigure.request.config.ints.push_back(set_max_planning_retries);
  move_base_reconfigure.request.config.bools.push_back(set_recovery_behavior_enabled);
  move_base_reconfigure.request.config.bools.push_back(set_clearing_rotation_allowed);
  move_base_reconfigure.request.config.doubles.push_back(set_oscillation_timeout);

  if(set_move_base_param_.call(move_base_reconfigure))
    { ROS_INFO("[movel_move_base] move_base plan set params..."); }
  else
    { ROS_ERROR("[movel_move_base] Failed move_base plan set params..."); }

  ROS_INFO("Reconfigure Params 2");

  if (use_pebble_)
  {
    ROS_INFO("Reconfigure Params - Pebble");

    dynamic_reconfigure::Reconfigure pebble_reconfigure;
    dynamic_reconfigure::BoolParameter set_obs_avoid;
    set_obs_avoid.name = "local_obstacle_avoidance";
    set_obs_avoid.value = obs_avoid;
    pebble_reconfigure.request.config.bools.push_back(set_obs_avoid);

    if(set_pebble_params_.call(pebble_reconfigure))
      { ROS_INFO("[movel_move_base] Pebble plan set params..."); }
    else
      { ROS_ERROR("[movel_move_base] Failed Pebble plan set params..."); }
  }
}

bool MoveBase::onStopObstacleCheck(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
  if (stop_at_obstacle_) {
    res.success = true;
    res.message = "Stop obstacle enabled";
  }
  else {
    res.success = false;
    res.message = "Stop obstacle not enabled";
  }
  return true;
}

void MoveBase::redisInit()
{
  ros::NodeHandle private_nh("~");

  // redis
  if (private_nh.hasParam("redis_host"))
    private_nh.getParam("redis_host", redis_host_);
  if (private_nh.hasParam("redis_port"))
    private_nh.getParam("redis_port", redis_port_);

  if (private_nh.hasParam("socket_timeout"))
    private_nh.getParam("socket_timeout", socket_timeout_);
  else
    socket_timeout_= 1 ;
    
  opts_.host = redis_host_;
  opts_.port = stoi(redis_port_);
  opts_.socket_timeout = std::chrono::milliseconds(socket_timeout_);

  redis_stop_at_obstacle_lim_key_ = "stop_at_obstacle_enabled";

  auto redis = sw::redis::Redis(opts_);
  auto sub = redis.subscriber();

  auto val_stop_at_obs = redis.get(redis_stop_at_obstacle_lim_key_);
  if(*val_stop_at_obs=="true")
    stop_at_obstacle_ = true;
  else if(*val_stop_at_obs=="false")
    stop_at_obstacle_ = false;

  private_nh.setParam("stop_at_obstacle", stop_at_obstacle_);
}

void MoveBase::goalCB(const geometry_msgs::PoseStamped::ConstPtr& goal)
{
  ROS_DEBUG_NAMED("move_base",
                  "In ROS goal callback, wrapping the PoseStamped in the action message and re-sending to the server.");
  move_base_msgs::MoveBaseActionGoal action_goal;
  action_goal.header.stamp = ros::Time::now();
  action_goal.goal.target_pose = *goal;

  action_goal_pub_.publish(action_goal);
}

void MoveBase::clearCostmapWindows(double size_x, double size_y)
{
  geometry_msgs::PoseStamped global_pose;

  // clear the planner's costmap
  getRobotPose(global_pose, planner_costmap_ros_);

  std::vector<geometry_msgs::Point> clear_poly;
  double x = global_pose.pose.position.x;
  double y = global_pose.pose.position.y;
  geometry_msgs::Point pt;

  pt.x = x - size_x / 2;
  pt.y = y - size_y / 2;
  clear_poly.push_back(pt);

  pt.x = x + size_x / 2;
  pt.y = y - size_y / 2;
  clear_poly.push_back(pt);

  pt.x = x + size_x / 2;
  pt.y = y + size_y / 2;
  clear_poly.push_back(pt);

  pt.x = x - size_x / 2;
  pt.y = y + size_y / 2;
  clear_poly.push_back(pt);

  planner_costmap_ros_->getCostmap()->setConvexPolygonCost(clear_poly, costmap_2d::FREE_SPACE);

  // clear the controller's costmap
  getRobotPose(global_pose, controller_costmap_ros_);

  clear_poly.clear();
  x = global_pose.pose.position.x;
  y = global_pose.pose.position.y;

  pt.x = x - size_x / 2;
  pt.y = y - size_y / 2;
  clear_poly.push_back(pt);

  pt.x = x + size_x / 2;
  pt.y = y - size_y / 2;
  clear_poly.push_back(pt);

  pt.x = x + size_x / 2;
  pt.y = y + size_y / 2;
  clear_poly.push_back(pt);

  pt.x = x - size_x / 2;
  pt.y = y + size_y / 2;
  clear_poly.push_back(pt);

  controller_costmap_ros_->getCostmap()->setConvexPolygonCost(clear_poly, costmap_2d::FREE_SPACE);
}

bool MoveBase::clearCostmapsService(std_srvs::Empty::Request& req, std_srvs::Empty::Response& resp)
{
  // clear the costmaps
  boost::unique_lock<costmap_2d::Costmap2D::mutex_t> lock_controller(
      *(controller_costmap_ros_->getCostmap()->getMutex()));
  controller_costmap_ros_->resetLayers();

  boost::unique_lock<costmap_2d::Costmap2D::mutex_t> lock_planner(*(planner_costmap_ros_->getCostmap()->getMutex()));
  planner_costmap_ros_->resetLayers();
  return true;
}

bool MoveBase::stopObstacleService(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res)
{
  if (req.data && stop_at_obstacle_)
    res.message = "plan_inspector already enabled";
  else if (req.data && !stop_at_obstacle_)
    res.message = "plan_inspector enabled";
  else if (!req.data && !stop_at_obstacle_)
    res.message = "plan_inspector already disabled";
  else if (!req.data && stop_at_obstacle_)
    res.message = "plan_inspector disabled";

  stop_at_obstacle_ = req.data;
  // dynamic_reconfigure::Reconfigure reconf;
  // dynamic_reconfigure::BoolParameter stop_obs;
  // ros::NodeHandle nh_;
  // ros::ServiceClient set_common_params =
  // nh_.serviceClient<dynamic_reconfigure::Reconfigure>("/move_base/set_parameters"); stop_obs.name =
  // "stop_at_obstacle"; stop_obs.value = stop_at_obstacle_; reconf.request.config.bools.push_back(stop_obs);
  // set_common_params.call(reconf);
  res.success = true;
  return true;
}

bool MoveBase::enablePlanInspector(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res)
{
  if (req.data && stop_at_obstacle_)
    res.message = "plan_inspector already enabled";
  else if (req.data && !stop_at_obstacle_)
    res.message = "plan_inspector enabled";
  else if (!req.data && !stop_at_obstacle_)
    res.message = "plan_inspector already disabled";
  else if (!req.data && stop_at_obstacle_)
    res.message = "plan_inspector disabled";

  stop_at_obstacle_ = req.data;

  if (stop_at_obstacle_)
    saveParams();
  reconfigureParams(stop_at_obstacle_);
  // dynamic_reconfigure::Reconfigure reconf;
  // dynamic_reconfigure::BoolParameter stop_obs;
  // ros::NodeHandle nh_;
  // ros::ServiceClient set_common_params =
  // nh_.serviceClient<dynamic_reconfigure::Reconfigure>("/move_base/set_parameters"); stop_obs.name =
  // "stop_at_obstacle"; stop_obs.value = stop_at_obstacle_; reconf.request.config.bools.push_back(stop_obs);
  // set_common_params.call(reconf);
  res.success = true;
  return true;
}

bool MoveBase::planService(nav_msgs::GetPlan::Request& req, nav_msgs::GetPlan::Response& resp)
{
  if (as_->isActive())
  {
    ROS_ERROR("move_base must be in an inactive state to make a plan for an external user");
    return false;
  }
  // make sure we have a costmap for our planner
  if (planner_costmap_ros_ == NULL)
  {
    ROS_ERROR("move_base cannot make a plan for you because it doesn't have a costmap");
    return false;
  }

  geometry_msgs::PoseStamped start;
  // if the user does not specify a start pose, identified by an empty frame id, then use the robot's pose
  if (req.start.header.frame_id.empty())
  {
    geometry_msgs::PoseStamped global_pose;
    if (!getRobotPose(global_pose, planner_costmap_ros_))
    {
      ROS_ERROR("move_base cannot make a plan for you because it could not get the start pose of the robot");
      return false;
    }
    start = global_pose;
  }
  else
  {
    start = req.start;
  }

  if (make_plan_clear_costmap_)
  {
    // update the copy of the costmap the planner uses
    clearCostmapWindows(2 * clearing_radius_, 2 * clearing_radius_);
  }

  // first try to make a plan to the exact desired goal
  std::vector<geometry_msgs::PoseStamped> global_plan;
  if (!planner_->makePlan(start, req.goal, global_plan) || global_plan.empty())
  {
    ROS_DEBUG_NAMED("move_base",
                    "Failed to find a plan to exact goal of (%.2f, %.2f), searching for a feasible goal within "
                    "tolerance",
                    req.goal.pose.position.x, req.goal.pose.position.y);

    // search outwards for a feasible goal within the specified tolerance
    geometry_msgs::PoseStamped p;
    p = req.goal;
    bool found_legal = false;
    float resolution = planner_costmap_ros_->getCostmap()->getResolution();
    float search_increment = resolution * 3.0;
    if (req.tolerance > 0.0 && req.tolerance < search_increment)
      search_increment = req.tolerance;
    for (float max_offset = search_increment; max_offset <= req.tolerance && !found_legal;
         max_offset += search_increment)
    {
      for (float y_offset = 0; y_offset <= max_offset && !found_legal; y_offset += search_increment)
      {
        for (float x_offset = 0; x_offset <= max_offset && !found_legal; x_offset += search_increment)
        {
          // don't search again inside the current outer layer
          if (x_offset < max_offset - 1e-9 && y_offset < max_offset - 1e-9)
            continue;

          // search to both sides of the desired goal
          for (float y_mult = -1.0; y_mult <= 1.0 + 1e-9 && !found_legal; y_mult += 2.0)
          {
            // if one of the offsets is 0, -1*0 is still 0 (so get rid of one of the two)
            if (y_offset < 1e-9 && y_mult < -1.0 + 1e-9)
              continue;

            for (float x_mult = -1.0; x_mult <= 1.0 + 1e-9 && !found_legal; x_mult += 2.0)
            {
              if (x_offset < 1e-9 && x_mult < -1.0 + 1e-9)
                continue;

              p.pose.position.y = req.goal.pose.position.y + y_offset * y_mult;
              p.pose.position.x = req.goal.pose.position.x + x_offset * x_mult;

              if (planner_->makePlan(start, p, global_plan))
              {
                if (!global_plan.empty())
                {
                  if (make_plan_add_unreachable_goal_)
                  {
                    // adding the (unreachable) original goal to the end of the global plan, in case the local planner
                    // can get you there (the reachable goal should have been added by the global planner)
                    global_plan.push_back(req.goal);
                  }

                  found_legal = true;
                  ROS_DEBUG_NAMED("move_base", "Found a plan to point (%.2f, %.2f)", p.pose.position.x,
                                  p.pose.position.y);
                  break;
                }
              }
              else
              {
                ROS_DEBUG_NAMED("move_base", "Failed to find a plan to point (%.2f, %.2f)", p.pose.position.x,
                                p.pose.position.y);
              }
            }
          }
        }
      }
    }
  }

  // copy the plan into a message to send out
  resp.plan.poses.resize(global_plan.size());
  for (unsigned int i = 0; i < global_plan.size(); ++i)
  {
    resp.plan.poses[i] = global_plan[i];
  }

  return true;
}

MoveBase::~MoveBase()
{
  recovery_behaviors_.clear();

  delete dsrv_;

  if (as_ != NULL)
    delete as_;

  if (planner_costmap_ros_ != NULL)
    delete planner_costmap_ros_;

  if (controller_costmap_ros_ != NULL)
    delete controller_costmap_ros_;

  planner_thread_->interrupt();
  planner_thread_->join();

  delete planner_thread_;

  delete planner_plan_;
  delete latest_plan_;
  delete controller_plan_;

  planner_.reset();
  tc_.reset();
}

bool MoveBase::makePlan(const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan)
{
  boost::unique_lock<costmap_2d::Costmap2D::mutex_t> lock(*(planner_costmap_ros_->getCostmap()->getMutex()));

  // make sure to set the plan to be empty initially
  plan.clear();

  // since this gets called on handle activate
  if (planner_costmap_ros_ == NULL)
  {
    ROS_ERROR("Planner costmap ROS is NULL, unable to create global plan");
    return false;
  }

  // get the starting pose of the robot
  geometry_msgs::PoseStamped global_pose;
  if (!getRobotPose(global_pose, planner_costmap_ros_))
  {
    ROS_WARN("Unable to get starting pose of robot, unable to create global plan");
    return false;
  }

  const geometry_msgs::PoseStamped& start = global_pose;

  // if the planner fails or returns a zero length plan, planning failed
  if (!planner_->makePlan(start, goal, plan) || plan.empty())
  {
    ROS_DEBUG_NAMED("move_base", "Failed to find a  plan to point (%.2f, %.2f)", goal.pose.position.x,
                    goal.pose.position.y);
    return false;
  }

  return true;
}

void MoveBase::wakePlanner(const ros::TimerEvent& event)
{
  // we have slept long enough for rate
  planner_cond_.notify_one();
}

void MoveBase::planThread()
{
  ROS_DEBUG_NAMED("move_base_plan_thread", "Starting planner thread...");
  ros::NodeHandle n;
  ros::Timer timer;
  bool wait_for_wake = false;
  boost::unique_lock<boost::recursive_mutex> lock(planner_mutex_);
  while (n.ok())
  {
    // check if we should run the planner (the mutex is locked)
    while (wait_for_wake || !runPlanner_)
    {
      // if we should not be running the planner then suspend this thread
      ROS_DEBUG_NAMED("move_base_plan_thread", "Planner thread is suspending");
      planner_cond_.wait(lock);
      wait_for_wake = false;
    }
    ros::Time start_time = ros::Time::now();

    // time to plan! get a copy of the goal and unlock the mutex
    geometry_msgs::PoseStamped temp_goal = planner_goal_;
    lock.unlock();
    ROS_DEBUG_NAMED("move_base_plan_thread", "Planning...");

    // run planner
    planner_plan_->clear();
    bool gotPlan = n.ok() && makePlan(temp_goal, *planner_plan_);

    bool plan_ok = true;
    if (gotPlan && stop_at_obstacle_)
    {
      BlockageType blockage;
      blockage = plan_inspector_->processNewPlan(*planner_plan_, temp_goal);
      // DEBUG PRINT
      // switch (blockage)
      // {
      //   case BlockageType::FAILED:
      //     std::cout << "[planThread] processNewPlan ok BlockageType::FAILED" << std::endl;
      //     break;
      //   case BlockageType::NEW_GOAL:
      //     std::cout << "[planThread] processNewPlan ok BlockageType::NEW_GOAL" << std::endl;
      //     break;
      //   case BlockageType::PARTIAL:
      //     std::cout << "[planThread] processNewPlan ok BlockageType::PARTIAL" << std::endl;
      //     break;
      //   case BlockageType::FULL:
      //     std::cout << "[planThread] processNewPlan ok BlockageType::FULL" << std::endl;
      //     break;
      // }

      // partial blockage check
      plan_ok =
          (allow_partial_blockage_replan_ && blockage == BlockageType::PARTIAL) || blockage == BlockageType::NEW_GOAL;
      // std::cout << "[planThread] plan_ok == " << (plan_ok ? "true" : "false") << std::endl;
    }

    // if we got a plan and it is valid (passed partial blockage check)
    if (gotPlan && plan_ok)
    {
      ROS_DEBUG_NAMED("move_base_plan_thread", "Got Plan with %zu points!", planner_plan_->size());
      // pointer swap the plans under mutex (the controller will pull from latest_plan_)
      std::vector<geometry_msgs::PoseStamped>* temp_plan = planner_plan_;

      lock.lock();
      planner_plan_ = latest_plan_;
      latest_plan_ = temp_plan;
      last_valid_plan_ = ros::Time::now();
      planning_retries_ = 0;
      new_global_plan_ = true;

      ROS_DEBUG_NAMED("move_base_plan_thread", "Generated a plan from the base_global_planner");

      // make sure we only start the controller if we still haven't reached the goal
      if (runPlanner_)
        state_ = CONTROLLING;
      if (planner_frequency_ <= 0)
        runPlanner_ = false;
      lock.unlock();
    }
    // if we didn't get a valid plan and we are in the planning state (the robot isn't moving)
    else if (state_ == PLANNING)
    {
      ROS_DEBUG_NAMED("move_base_plan_thread", "No Plan...");

      // If we managed to get a plan (gotPlan == true) but it didn't pass the partial blockage check (plan_ok == false)
      // it means we have an obstruction on our last valid path that needs to be cleared
      // maybe use if (gotPlan && !plan_ok) to make it more verbose?
      if (!plan_ok)
      {
        ros::Time attempt_end = last_valid_plan_ + ros::Duration(clearing_timeout_);

        // check if obstacle clearing timeout is reached (means we still don't have new valid plan while the obstacle
        // is still there)
        lock.lock();
        if (runPlanner_ && (ros::Time::now() > attempt_end) && (clearing_timeout_ >= 0.0))
        {
          // we'll abort current goal and prevent best effort navigation
          state_ = FORCING_FAILURE;
          runPlanner_ = false;
          publishZeroVelocity();
        }

        lock.unlock();
      }
      // we failed to get any plan at all (gotPlan == false)
      else
      {
        ros::Time attempt_end = last_valid_plan_ + ros::Duration(planner_patience_);

        // check if we've tried to make a plan for over our time limit or our maximum number of retries
        // issue #496: we stop planning when one of the conditions is true, but if max_planning_retries_
        // is negative (the default), it is just ignored and we have the same behavior as ever
        lock.lock();
        planning_retries_++;
        if (runPlanner_ && (ros::Time::now() > attempt_end || planning_retries_ > uint32_t(max_planning_retries_)))
        {
          // we'll move into our obstacle clearing mode
          state_ = CLEARING;
          runPlanner_ = false;  // proper solution for issue #523
          publishZeroVelocity();
          recovery_trigger_ = PLANNING_R;
        }

        lock.unlock();
      }
    }

    // take the mutex for the next iteration
    lock.lock();

    // setup sleep interface if needed
    if (planner_frequency_ > 0)
    {
      ros::Duration sleep_time = (start_time + ros::Duration(1.0 / planner_frequency_)) - ros::Time::now();
      if (sleep_time > ros::Duration(0.0))
      {
        wait_for_wake = true;
        timer = n.createTimer(sleep_time, &MoveBase::wakePlanner, this);
      }
    }
  }
}

void MoveBase::executeCb(const move_base_msgs::MoveBaseGoalConstPtr& move_base_goal)
{
  if (!isQuaternionValid(move_base_goal->target_pose.pose.orientation))
  {
    as_->setAborted(move_base_msgs::MoveBaseResult(),
                    "Aborting on goal because it was sent with an invalid "
                    "quaternion");
    return;
  }

  geometry_msgs::PoseStamped goal = goalToGlobalFrame(move_base_goal->target_pose);
  publishZeroVelocity();
  // we have a goal so start the planner
  boost::unique_lock<boost::recursive_mutex> lock(planner_mutex_);
  planner_goal_ = goal;
  runPlanner_ = true;
  planner_cond_.notify_one();
  lock.unlock();

  current_goal_pub_.publish(goal);

  ros::Rate r(controller_frequency_);
  if (shutdown_costmaps_)
  {
    ROS_DEBUG_NAMED("move_base", "Starting up costmaps that were shut down previously");
    planner_costmap_ros_->start();
    controller_costmap_ros_->start();
  }

  // we want to make sure that we reset the last time we had a valid plan and control
  last_valid_control_ = ros::Time::now();
  last_valid_plan_ = ros::Time::now();
  last_oscillation_reset_ = ros::Time::now();
  planning_retries_ = 0;
  has_valid_control_ = false;
  plan_obstructed_ = false;

  ros::NodeHandle n;
  while (n.ok())
  {
    if (c_freq_change_)
    {
      ROS_INFO("Setting controller frequency to %.2f", controller_frequency_);
      r = ros::Rate(controller_frequency_);
      c_freq_change_ = false;
    }

    if (as_->isPreemptRequested())
    {
      if (as_->isNewGoalAvailable())
      {
        // if we're active and a new goal is available, we'll accept it, but we won't shut anything down
        move_base_msgs::MoveBaseGoal new_goal = *as_->acceptNewGoal();

        if (!isQuaternionValid(new_goal.target_pose.pose.orientation))
        {
          as_->setAborted(move_base_msgs::MoveBaseResult(),
                          "Aborting on goal because it was sent with an invalid quaternion");
          return;
        }

        goal = goalToGlobalFrame(new_goal.target_pose);

        // we'll make sure that we reset our state for the next execution cycle
        recovery_index_ = 0;
        state_ = PLANNING;

        // we have a new goal so make sure the planner is awake
        lock.lock();
        planner_goal_ = goal;
        runPlanner_ = true;
        planner_cond_.notify_one();
        lock.unlock();

        // publish the goal point to the visualizer
        ROS_DEBUG_NAMED("move_base", "move_base has received a goal of x: %.2f, y: %.2f", goal.pose.position.x,
                        goal.pose.position.y);
        current_goal_pub_.publish(goal);

        // make sure to reset our timeouts and counters
        last_valid_control_ = ros::Time::now();
        last_valid_plan_ = ros::Time::now();
        last_oscillation_reset_ = ros::Time::now();
        planning_retries_ = 0;
        has_valid_control_ = false;
        plan_obstructed_ = false;
      }
      else
      {
        // if we've been preempted explicitly we need to shut things down
        resetState();

        // notify the ActionServer that we've successfully preempted
        ROS_DEBUG_NAMED("move_base", "Move base preempting the current goal");
        as_->setPreempted();

        // we'll actually return from execute after preempting
        return;
      }
    }

    // we also want to check if we've changed global frames because we need to transform our goal pose
    if (goal.header.frame_id != planner_costmap_ros_->getGlobalFrameID())
    {
      goal = goalToGlobalFrame(goal);

      // we want to go back to the planning state for the next execution cycle
      recovery_index_ = 0;
      state_ = PLANNING;

      // we have a new goal so make sure the planner is awake
      lock.lock();
      planner_goal_ = goal;
      runPlanner_ = true;
      planner_cond_.notify_one();
      lock.unlock();

      // publish the goal point to the visualizer
      ROS_DEBUG_NAMED("move_base",
                      "The global frame for move_base has changed, new frame: %s, new goal position x: %.2f, y: %.2f",
                      goal.header.frame_id.c_str(), goal.pose.position.x, goal.pose.position.y);
      current_goal_pub_.publish(goal);

      // make sure to reset our timeouts and counters
      last_valid_control_ = ros::Time::now();
      last_valid_plan_ = ros::Time::now();
      last_oscillation_reset_ = ros::Time::now();
      planning_retries_ = 0;
      has_valid_control_ = false;
      plan_obstructed_ = false;
    }

    // for timing that gives real time even in simulation
    ros::WallTime start = ros::WallTime::now();

    // the real work on pursuing a goal is done here
    bool done = executeCycle(goal);

    // if we're done, then we'll return from execute
    if (done)
      return;

    // check if execution of the goal has completed in some way

    ros::WallDuration t_diff = ros::WallTime::now() - start;
    ROS_DEBUG_NAMED("move_base", "Full control cycle time: %.9f\n", t_diff.toSec());

    r.sleep();
    // make sure to sleep for the remainder of our cycle time
    if (r.cycleTime() > ros::Duration(1 / controller_frequency_) && state_ == CONTROLLING)
      ROS_WARN("Control loop missed its desired rate of %.4fHz... the loop actually took %.4f seconds",
               controller_frequency_, r.cycleTime().toSec());
  }

  // wake up the planner thread so that it can exit cleanly
  lock.lock();
  runPlanner_ = true;
  planner_cond_.notify_one();
  lock.unlock();

  // if the node is killed then we'll abort and return
  as_->setAborted(move_base_msgs::MoveBaseResult(), "Aborting on the goal because the node has been killed");
  return;
}

bool MoveBase::executeCycle(geometry_msgs::PoseStamped& goal)
{
  boost::recursive_mutex::scoped_lock ecl(configuration_mutex_);
  // we need to be able to publish velocity commands
  geometry_msgs::Twist cmd_vel;

  // update feedback to correspond to our curent position
  geometry_msgs::PoseStamped global_pose;
  getRobotPose(global_pose, planner_costmap_ros_);
  const geometry_msgs::PoseStamped& current_position = global_pose;

  // push the feedback out
  move_base_msgs::MoveBaseFeedback feedback;
  feedback.base_position = current_position;
  as_->publishFeedback(feedback);

  // check to see if we've moved far enough or plan is obstructed so robot stays on place to reset our oscillation
  // timeout
  if (distance(current_position, oscillation_pose_) >= oscillation_distance_ ||
      (stop_at_obstacle_ && stop_caused_by_obstacle_))
  {
    last_oscillation_reset_ = ros::Time::now();
    oscillation_pose_ = current_position;

    // if our last recovery was caused by oscillation, we want to reset the recovery index
    if (recovery_trigger_ == OSCILLATION_R)
      recovery_index_ = 0;
  }

  // check that the observation buffers for the costmap are current, we don't want to drive blind
  if (!controller_costmap_ros_->isCurrent())
  {
    ROS_WARN("[%s]:Sensor data is out of date, we're not going to allow commanding of the base for safety",
             ros::this_node::getName().c_str());
    publishZeroVelocity();
    return false;
  }

  // if we have a new plan then grab it and give it to the controller
  if (new_global_plan_)
  {
    // make sure to set the new plan flag to false
    new_global_plan_ = false;

    ROS_DEBUG_NAMED("move_base", "Got a new plan...swap pointers");

    // do a pointer swap under mutex
    std::vector<geometry_msgs::PoseStamped>* temp_plan = controller_plan_;

    boost::unique_lock<boost::recursive_mutex> lock(planner_mutex_);
    controller_plan_ = latest_plan_;
    latest_plan_ = temp_plan;
    lock.unlock();
    ROS_DEBUG_NAMED("move_base", "pointers swapped!");

    if (!tc_->setPlan(*controller_plan_))
    {
      // ABORT and SHUTDOWN COSTMAPS
      ROS_ERROR("Failed to pass global plan to the controller, aborting.");
      resetState();

      // disable the planner thread
      lock.lock();
      runPlanner_ = false;
      lock.unlock();

      as_->setAborted(move_base_msgs::MoveBaseResult(), "Failed to pass global plan to the controller.");
      return true;
    }

    // make sure to reset recovery_index_ since we were able to find a valid plan
    if (recovery_trigger_ == PLANNING_R)
      recovery_index_ = 0;
  }

  // the move_base state machine, handles the control logic for navigation
  switch (state_)
  {
    // if we are in a planning state, then we'll attempt to make a plan
    case PLANNING:
      // local scope
      {
        boost::recursive_mutex::scoped_lock lock(planner_mutex_);
        runPlanner_ = true;
        planner_cond_.notify_one();
      }
      ROS_DEBUG_NAMED("move_base", "Waiting for plan, in the planning state.");
      break;

    // if we're controlling, we'll attempt to find valid velocity commands
    case CONTROLLING:
      ROS_DEBUG_NAMED("move_base", "In controlling state.");
      // check to see if we've reached our goal
      if (tc_->isGoalReached())
      {
        ROS_DEBUG_NAMED("move_base", "Goal reached!");
        resetState();

        // disable the planner thread
        boost::unique_lock<boost::recursive_mutex> lock(planner_mutex_);
        runPlanner_ = false;
        lock.unlock();

        as_->setSucceeded(move_base_msgs::MoveBaseResult(), "Goal reached.");
        return true;
      }

      // check for an oscillation condition
      if (oscillation_timeout_ > 0.0 &&
          last_oscillation_reset_ + ros::Duration(oscillation_timeout_) < ros::Time::now())
      {
        publishZeroVelocity();
        state_ = CLEARING;
        recovery_trigger_ = OSCILLATION_R;
      }

      {  // <scope>

        boost::unique_lock<costmap_2d::Costmap2D::mutex_t> lock(*(controller_costmap_ros_->getCostmap()->getMutex()));

        // plan inspector process path
        bool plan_obstructed = false;
        bool obstruction_within_threshold = false;
        if (stop_at_obstacle_)
        {
          geometry_msgs::PoseStamped obs_location;
          plan_obstructed = plan_inspector_->checkObstruction(*controller_plan_, *controller_costmap_ros_,
                                                              current_position, obs_location);
          obstruction_within_threshold = distance(current_position, obs_location) <= stop_at_obstacle_distance_;

          // if (plan_obstructed && distance(current_position, obs_location) > stop_at_obstacle_distance_)
          //   plan_obstructed = false;

          if (plan_obstructed && !plan_obstructed_)
          {
            // report obstruction
            movel_seirios_msgs::ObstructionStatus report_obs;
            report_obs.reporter = "plan_inspector";
            report_obs.status = "true";
            report_obs.location = obs_location.pose;
            obstruction_status_pub_.publish(report_obs);

            plan_obstructed_ = true;
          }
          else if (!plan_obstructed && plan_obstructed_)
          {
            // report obstruction
            movel_seirios_msgs::ObstructionStatus report_obs;
            report_obs.reporter = "plan_inspector";
            report_obs.status = "false";
            report_obs.location = obs_location.pose;
            obstruction_status_pub_.publish(report_obs);

            plan_obstructed_ = false;
          }
        }

        // std::cout << "[executeCycle] plan_obstructed == " << (plan_obstructed ? "true" : "false") << std::endl;
        // std::cout << "[executeCycle] obstruction_within_threshold == "
        //           << (obstruction_within_threshold ? "true" : "false") << std::endl;

        // if no obstacles anywhere on the plan, proceed to get control from local planner
        stop_caused_by_obstacle_ = false;
        if (!plan_obstructed)
        {
          if (tc_->computeVelocityCommands(cmd_vel))
          {
            ROS_DEBUG_NAMED("move_base", "Got a valid command from the local planner: %.3lf, %.3lf, %.3lf",
                            cmd_vel.linear.x, cmd_vel.linear.y, cmd_vel.angular.z);
            last_valid_control_ = ros::Time::now();
            last_valid_cmd_vel_ = cmd_vel;
            has_valid_control_ = true;
            // std::cout << "CONTROL CMD_VEL \n" << cmd_vel << std::endl;
            // make sure that we send the velocity command to the base
            vel_pub_.publish(cmd_vel);
            if (recovery_trigger_ == CONTROLLING_R)
              recovery_index_ = 0;
          }
          else
          {
            ROS_DEBUG_NAMED("move_base", "The local planner could not find a valid plan.");
            ros::Time attempt_end = last_valid_control_ + ros::Duration(controller_patience_);

            // check if we've tried to find a valid control for longer than our time limit
            if (ros::Time::now() > attempt_end)
            {
              // we'll move into our obstacle clearing mode
              publishZeroVelocity();
              state_ = CLEARING;
              recovery_trigger_ = CONTROLLING_R;
            }
            else
            {
              // otherwise, if we can't find a valid control, we'll go back to planning
              last_valid_plan_ = ros::Time::now();
              planning_retries_ = 0;
              state_ = PLANNING;
              publishZeroVelocity();

              // enable the planner thread in case it isn't running on a clock
              boost::unique_lock<boost::recursive_mutex> lock(planner_mutex_);
              runPlanner_ = true;
              planner_cond_.notify_one();
              lock.unlock();
            }
          }
        }

        // obstruction is found anywhere on the plan
        else
        {
          // obstruction is within the threshold, immediately stop robot
          if (obstruction_within_threshold)
          {
            publishZeroVelocity();
            stop_caused_by_obstacle_ = true;

            // if plan is obstructed, check if we have reached clearing timeout
            ros::Time attempt_end = last_valid_control_ + ros::Duration(clearing_timeout_);

            // clearing timeout reached
            if ((ros::Time::now() > attempt_end) && (clearing_timeout_ >= 0.0))
            {
              if (allow_replan_after_timeout_)
              {
                // if allow replan after timeout, go back to planning
                last_valid_plan_ = ros::Time::now();
                planning_retries_ = 0;
                state_ = PLANNING;
                // publishZeroVelocity();

                // enable the planner thread in case it isn't running on a clock
                boost::unique_lock<boost::recursive_mutex> lock(planner_mutex_);
                runPlanner_ = true;
                planner_cond_.notify_one();
                lock.unlock();
              }
              else
              {
                // if not allow replan after timeout, abort goal and prevent best effort navigation
                ROS_ERROR("Aborting because there appears to be an impassable obstacle obstructing the plan.");
                state_ = FORCING_FAILURE;
                // as_->setAborted(move_base_msgs::MoveBaseResult(), "Plan is obstructed after clearing timeout is
                // passed."); resetState(); return true;
              }
            }

            // waiting for clearance
            else
            {
              if (allow_recovery_during_timeout_ && recovery_behavior_enabled_)
              {
                // we'll move into our obstacle clearing mode
                // publishZeroVelocity();
                state_ = CLEARING;
                recovery_trigger_ = CONTROLLING_R;
              }
            }
          }

          // obstruction is not within threshold, still allow movement
          else
          {
            if (has_valid_control_)
            {
              cmd_vel = last_valid_cmd_vel_;
              vel_pub_.publish(cmd_vel);
            }
          }
        }

      }  // </scope>

      break;

    // we'll try to clear out space with any user-provided recovery behaviors
    case CLEARING:
      ROS_DEBUG_NAMED("move_base", "In clearing/recovery state");
      // if (recovery_trigger_ == CONTROLLING_R)
      // {
      //   std::cout << "[executeCycle] RECOVERY BEHAVIOR TRIGGER CONTROLLING_R" << std::endl;
      // }
      // else if (recovery_trigger_ == PLANNING_R)
      // {
      //   std::cout << "[executeCycle] RECOVERY BEHAVIOR TRIGGER PLANNING_R" << std::endl;
      // }
      // else if (recovery_trigger_ == OSCILLATION_R)
      // {
      //   std::cout << "[executeCycle] RECOVERY BEHAVIOR TRIGGER OSCILLATION_R" << std::endl;
      // }
      // we'll invoke whatever recovery behavior we're currently on if they're enabled
      if (recovery_behavior_enabled_ && recovery_index_ < recovery_behaviors_.size())
      {
        ROS_DEBUG_NAMED("move_base_recovery", "Executing behavior %u of %zu", recovery_index_ + 1,
                        recovery_behaviors_.size());

        move_base_msgs::RecoveryStatus msg;
        msg.pose_stamped = current_position;
        msg.current_recovery_number = recovery_index_;
        msg.total_number_of_recoveries = recovery_behaviors_.size();
        msg.recovery_behavior_name = recovery_behavior_names_[recovery_index_];

        recovery_status_pub_.publish(msg);

        recovery_behaviors_[recovery_index_]->runBehavior();

        // we at least want to give the robot some time to stop oscillating after executing the behavior
        last_oscillation_reset_ = ros::Time::now();

        // we'll check if the recovery behavior actually worked
        ROS_DEBUG_NAMED("move_base_recovery", "Going back to planning state");
        last_valid_plan_ = ros::Time::now();
        planning_retries_ = 0;
        state_ = PLANNING;

        // update the index of the next recovery behavior that we'll try
        recovery_index_++;
      }
      else
      {
        ROS_DEBUG_NAMED("move_base_recovery",
                        "All recovery behaviors have failed, locking the planner and disabling it.");
        // disable the planner thread
        boost::unique_lock<boost::recursive_mutex> lock(planner_mutex_);
        runPlanner_ = false;
        lock.unlock();

        ROS_DEBUG_NAMED("move_base_recovery", "Something should abort after this.");

        if (recovery_trigger_ == CONTROLLING_R)
        {
          ROS_ERROR("Aborting because a valid control could not be found. Even after executing all recovery behaviors");
          as_->setAborted(move_base_msgs::MoveBaseResult(),
                          "Failed to find a valid control. Even after executing recovery behaviors.");
        }
        else if (recovery_trigger_ == PLANNING_R)
        {
          ROS_ERROR("Aborting because a valid plan could not be found. Even after executing all recovery behaviors");
          as_->setAborted(move_base_msgs::MoveBaseResult(),
                          "Failed to find a valid plan. Even after executing recovery behaviors.");
        }
        else if (recovery_trigger_ == OSCILLATION_R)
        {
          ROS_ERROR(
              "Aborting because the robot appears to be oscillating over and over. Even after executing all recovery "
              "behaviors");
          as_->setAborted(move_base_msgs::MoveBaseResult(),
                          "Robot is oscillating. Even after executing recovery behaviors.");
        }
        resetState();
        return true;
      }
      break;
    case FORCING_FAILURE:
      as_->setAborted(move_base_msgs::MoveBaseResult(), "MOVE_BASE_FORCED_FAILURE");
      resetState();
      return true;
    default:
      ROS_ERROR("This case should never be reached, something is wrong, aborting");
      resetState();
      // disable the planner thread
      boost::unique_lock<boost::recursive_mutex> lock(planner_mutex_);
      runPlanner_ = false;
      lock.unlock();
      as_->setAborted(move_base_msgs::MoveBaseResult(),
                      "Reached a case that should not be hit in move_base. This is a bug, please report it.");
      return true;
  }

  // we aren't done yet
  return false;
}

void MoveBase::resetState()
{
  // Disable the planner thread
  boost::unique_lock<boost::recursive_mutex> lock(planner_mutex_);
  runPlanner_ = false;
  lock.unlock();

  // Reset statemachine
  state_ = PLANNING;
  recovery_index_ = 0;
  recovery_trigger_ = PLANNING_R;
  publishZeroVelocity();
  has_valid_control_ = false;
  plan_obstructed_ = false;

  // if we shutdown our costmaps when we're deactivated... we'll do that now
  if (shutdown_costmaps_)
  {
    ROS_DEBUG_NAMED("move_base", "Stopping costmaps");
    planner_costmap_ros_->stop();
    controller_costmap_ros_->stop();
  }
}

};  // namespace move_base