#include <movel_move_base/move_base.h>
#include <move_base_msgs/RecoveryStatus.h>
#include <geometry_msgs/Twist.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <boost/algorithm/string.hpp>
#include <boost/thread.hpp>
#include <cmath>


namespace move_base {

MoveBase::MoveBase(tf2_ros::Buffer& tf) :
  tf_(tf),
  as_(NULL),
  planner_costmap_ros_(NULL), controller_costmap_ros_(NULL),
  bgp_loader_("nav_core", "nav_core::BaseGlobalPlanner"),
  blp_loader_("nav_core", "nav_core::BaseLocalPlanner"),
  recovery_loader_("nav_core", "nav_core::RecoveryBehavior"),
  planner_plan_(NULL), latest_plan_(NULL), controller_plan_(NULL),
  runPlanner_(false), setup_(false), p_freq_change_(false), c_freq_change_(false), new_global_plan_(false) 
{
  as_ = new MoveBaseActionServer(ros::NodeHandle(), "move_base", boost::bind(&MoveBase::executeCb, this, _1), false);

  ros::NodeHandle private_nh("~");
  ros::NodeHandle nh;

  recovery_trigger_ = PLANNING_R;

  //get some parameters that will be global to the move base node
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
  private_nh.param("stop_at_obstacle", stop_at_obstacle_, false);


  //set up plan triple buffer
  planner_plan_ = new std::vector<geometry_msgs::PoseStamped>();
  latest_plan_ = new std::vector<geometry_msgs::PoseStamped>();
  controller_plan_ = new std::vector<geometry_msgs::PoseStamped>();

  //set up the planner's thread
  planner_thread_ = new boost::thread(boost::bind(&MoveBase::planThread, this));

  //for commanding the base
  vel_pub_ = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
  current_goal_pub_ = private_nh.advertise<geometry_msgs::PoseStamped>("current_goal", 0 );

  ros::NodeHandle action_nh("move_base");
  action_goal_pub_ = action_nh.advertise<move_base_msgs::MoveBaseActionGoal>("goal", 1);
  recovery_status_pub_= action_nh.advertise<move_base_msgs::RecoveryStatus>("recovery_status", 1);

  //we'll provide a mechanism for some people to send goals as PoseStamped messages over a topic
  //they won't get any useful information back about its status, but this is useful for tools
  //like nav_view and rviz
  ros::NodeHandle simple_nh("move_base_simple");
  goal_sub_ = simple_nh.subscribe<geometry_msgs::PoseStamped>("goal", 1, boost::bind(&MoveBase::goalCB, this, _1));

  //we'll assume the radius of the robot to be consistent with what's specified for the costmaps
  private_nh.param("local_costmap/inscribed_radius", inscribed_radius_, 0.325);
  private_nh.param("local_costmap/circumscribed_radius", circumscribed_radius_, 0.46);
  private_nh.param("clearing_radius", clearing_radius_, circumscribed_radius_);
  private_nh.param("conservative_reset_dist", conservative_reset_dist_, 3.0);

  private_nh.param("shutdown_costmaps", shutdown_costmaps_, false);
  private_nh.param("clearing_rotation_allowed", clearing_rotation_allowed_, true);
  private_nh.param("recovery_behavior_enabled", recovery_behavior_enabled_, true);

  //create the ros wrapper for the planner's costmap... and initializer a pointer we'll use with the underlying map
  planner_costmap_ros_ = new costmap_2d::Costmap2DROS("global_costmap", tf_);
  planner_costmap_ros_->pause();

  //initialize the global planner
  try {
    planner_ = bgp_loader_.createInstance(global_planner);
    planner_->initialize(bgp_loader_.getName(global_planner), planner_costmap_ros_);
  } catch (const pluginlib::PluginlibException& ex) {
    ROS_FATAL("Failed to create the %s planner, are you sure it is properly registered and that the containing library is built? Exception: %s", global_planner.c_str(), ex.what());
    exit(1);
  }

  //create the ros wrapper for the controller's costmap... and initializer a pointer we'll use with the underlying map
  controller_costmap_ros_ = new costmap_2d::Costmap2DROS("local_costmap", tf_);
  controller_costmap_ros_->pause();

  //create a local planner
  try {
    tc_ = blp_loader_.createInstance(local_planner);
    ROS_INFO("Created local_planner %s", local_planner.c_str());
    tc_->initialize(blp_loader_.getName(local_planner), &tf_, controller_costmap_ros_);
  } catch (const pluginlib::PluginlibException& ex) {
    ROS_FATAL("Failed to create the %s planner, are you sure it is properly registered and that the containing library is built? Exception: %s", local_planner.c_str(), ex.what());
    exit(1);
  }

  // Start actively updating costmaps based on sensor data
  planner_costmap_ros_->start();
  controller_costmap_ros_->start();

  //advertise a service for getting a plan
  make_plan_srv_ = private_nh.advertiseService("make_plan", &MoveBase::planService, this);

  //advertise a service for clearing the costmaps
  clear_costmaps_srv_ = private_nh.advertiseService("clear_costmaps", &MoveBase::clearCostmapsService, this);

  stop_obstacle_srv_ = private_nh.advertiseService("stop_at_obstacle", &MoveBase::stopObstacleService, this);

  //if we shutdown our costmaps when we're deactivated... we'll do that now
  if(shutdown_costmaps_){
    ROS_DEBUG_NAMED("move_base","Stopping costmaps initially");
    planner_costmap_ros_->stop();
    controller_costmap_ros_->stop();
  }

  //load any user specified recovery behaviors, and if that fails load the defaults
  if(!loadRecoveryBehaviors(private_nh)){
    loadDefaultRecoveryBehaviors();
  }

  //initially, we'll need to make a plan
  state_ = PLANNING;

  //we'll start executing recovery behaviors at the beginning of our list
  recovery_index_ = 0;

  //we're all set up now so we can start the action server
  as_->start();

  dsrv_ = new dynamic_reconfigure::Server<movel_move_base::MoveBaseConfig>(ros::NodeHandle("~"));
  dynamic_reconfigure::Server<movel_move_base::MoveBaseConfig>::CallbackType cb = boost::bind(&MoveBase::reconfigureCB, this, _1, _2);
  dsrv_->setCallback(cb);
}


void MoveBase::goalCB(const geometry_msgs::PoseStamped::ConstPtr& goal)
{
  ROS_DEBUG_NAMED("move_base","In ROS goal callback, wrapping the PoseStamped in the action message and re-sending to the server.");
  move_base_msgs::MoveBaseActionGoal action_goal;
  action_goal.header.stamp = ros::Time::now();
  action_goal.goal.target_pose = *goal;

  action_goal_pub_.publish(action_goal);
}


void MoveBase::clearCostmapWindows(double size_x, double size_y){
  geometry_msgs::PoseStamped global_pose;

  //clear the planner's costmap
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

  //clear the controller's costmap
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


bool MoveBase::clearCostmapsService(std_srvs::Empty::Request &req, std_srvs::Empty::Response &resp)
{
  //clear the costmaps
  boost::unique_lock<costmap_2d::Costmap2D::mutex_t> lock_controller(*(controller_costmap_ros_->getCostmap()->getMutex()));
  controller_costmap_ros_->resetLayers();

  boost::unique_lock<costmap_2d::Costmap2D::mutex_t> lock_planner(*(planner_costmap_ros_->getCostmap()->getMutex()));
  planner_costmap_ros_->resetLayers();
  return true;
}

bool MoveBase::stopObstacleService(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res)
{
  if(req.data && stop_at_obstacle_)
    res.message = "plan_inspector already enabled";
  else if(req.data && !stop_at_obstacle_)
    res.message = "plan_inspector enabled";
  else if(!req.data && !stop_at_obstacle_)
    res.message = "plan_inspector already disabled";
  else if(!req.data && stop_at_obstacle_)
    res.message = "plan_inspector disabled";

  stop_at_obstacle_ = req.data;
  // dynamic_reconfigure::Reconfigure reconf;
  // dynamic_reconfigure::BoolParameter stop_obs;
  // ros::NodeHandle nh_;
  // ros::ServiceClient set_common_params = nh_.serviceClient<dynamic_reconfigure::Reconfigure>("/move_base/set_parameters");
  // stop_obs.name = "stop_at_obstacle";
  // stop_obs.value = stop_at_obstacle_;
  // reconf.request.config.bools.push_back(stop_obs);
  // set_common_params.call(reconf);
  res.success = true;
  return true;
}

bool MoveBase::planService(nav_msgs::GetPlan::Request &req, nav_msgs::GetPlan::Response &resp)
{
  if(as_->isActive()){
    ROS_ERROR("move_base must be in an inactive state to make a plan for an external user");
    return false;
  }
  //make sure we have a costmap for our planner
  if(planner_costmap_ros_ == NULL){
    ROS_ERROR("move_base cannot make a plan for you because it doesn't have a costmap");
    return false;
  }

  geometry_msgs::PoseStamped start;
  //if the user does not specify a start pose, identified by an empty frame id, then use the robot's pose
  if(req.start.header.frame_id.empty())
  {
      geometry_msgs::PoseStamped global_pose;
      if(!getRobotPose(global_pose, planner_costmap_ros_)){
        ROS_ERROR("move_base cannot make a plan for you because it could not get the start pose of the robot");
        return false;
      }
      start = global_pose;
  }
  else
  {
      start = req.start;
  }

  if (make_plan_clear_costmap_) {
    //update the copy of the costmap the planner uses
    clearCostmapWindows(2 * clearing_radius_, 2 * clearing_radius_);
  }

  //first try to make a plan to the exact desired goal
  std::vector<geometry_msgs::PoseStamped> global_plan;
  if(!planner_->makePlan(start, req.goal, global_plan) || global_plan.empty()){
    ROS_DEBUG_NAMED("move_base","Failed to find a plan to exact goal of (%.2f, %.2f), searching for a feasible goal within tolerance",
        req.goal.pose.position.x, req.goal.pose.position.y);

    //search outwards for a feasible goal within the specified tolerance
    geometry_msgs::PoseStamped p;
    p = req.goal;
    bool found_legal = false;
    float resolution = planner_costmap_ros_->getCostmap()->getResolution();
    float search_increment = resolution*3.0;
    if(req.tolerance > 0.0 && req.tolerance < search_increment) search_increment = req.tolerance;
    for(float max_offset = search_increment; max_offset <= req.tolerance && !found_legal; max_offset += search_increment) {
      for(float y_offset = 0; y_offset <= max_offset && !found_legal; y_offset += search_increment) {
        for(float x_offset = 0; x_offset <= max_offset && !found_legal; x_offset += search_increment) {

          //don't search again inside the current outer layer
          if(x_offset < max_offset-1e-9 && y_offset < max_offset-1e-9) continue;

          //search to both sides of the desired goal
          for(float y_mult = -1.0; y_mult <= 1.0 + 1e-9 && !found_legal; y_mult += 2.0) {

            //if one of the offsets is 0, -1*0 is still 0 (so get rid of one of the two)
            if(y_offset < 1e-9 && y_mult < -1.0 + 1e-9) continue;

            for(float x_mult = -1.0; x_mult <= 1.0 + 1e-9 && !found_legal; x_mult += 2.0) {
              if(x_offset < 1e-9 && x_mult < -1.0 + 1e-9) continue;

              p.pose.position.y = req.goal.pose.position.y + y_offset * y_mult;
              p.pose.position.x = req.goal.pose.position.x + x_offset * x_mult;

              if(planner_->makePlan(start, p, global_plan)){
                if(!global_plan.empty()){

                  if (make_plan_add_unreachable_goal_) {
                    //adding the (unreachable) original goal to the end of the global plan, in case the local planner can get you there
                    //(the reachable goal should have been added by the global planner)
                    global_plan.push_back(req.goal);
                  }

                  found_legal = true;
                  ROS_DEBUG_NAMED("move_base", "Found a plan to point (%.2f, %.2f)", p.pose.position.x, p.pose.position.y);
                  break;
                }
              }
              else{
                ROS_DEBUG_NAMED("move_base","Failed to find a plan to point (%.2f, %.2f)", p.pose.position.x, p.pose.position.y);
              }
            }
          }
        }
      }
    }
  }

  //copy the plan into a message to send out
  resp.plan.poses.resize(global_plan.size());
  for(unsigned int i = 0; i < global_plan.size(); ++i){
    resp.plan.poses[i] = global_plan[i];
  }

  return true;
}


MoveBase::~MoveBase()
{
  recovery_behaviors_.clear();

  delete dsrv_;

  if(as_ != NULL)
    delete as_;

  if(planner_costmap_ros_ != NULL)
    delete planner_costmap_ros_;

  if(controller_costmap_ros_ != NULL)
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

  //make sure to set the plan to be empty initially
  plan.clear();

  //since this gets called on handle activate
  if(planner_costmap_ros_ == NULL) {
    ROS_ERROR("Planner costmap ROS is NULL, unable to create global plan");
    return false;
  }

  //get the starting pose of the robot
  geometry_msgs::PoseStamped global_pose;
  if(!getRobotPose(global_pose, planner_costmap_ros_)) {
    ROS_WARN("Unable to get starting pose of robot, unable to create global plan");
    return false;
  }

  const geometry_msgs::PoseStamped& start = global_pose;

  //if the planner fails or returns a zero length plan, planning failed
  if(!planner_->makePlan(start, goal, plan) || plan.empty()){
    ROS_DEBUG_NAMED("move_base","Failed to find a  plan to point (%.2f, %.2f)", goal.pose.position.x, goal.pose.position.y);
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
  ROS_DEBUG_NAMED("move_base_plan_thread","Starting planner thread...");
  ros::NodeHandle n;
  ros::Timer timer;
  bool wait_for_wake = false;
  boost::unique_lock<boost::recursive_mutex> lock(planner_mutex_);
  while(n.ok()){
    //check if we should run the planner (the mutex is locked)
    while(wait_for_wake || !runPlanner_){
      //if we should not be running the planner then suspend this thread
      ROS_DEBUG_NAMED("move_base_plan_thread","Planner thread is suspending");
      planner_cond_.wait(lock);
      wait_for_wake = false;
    }
    ros::Time start_time = ros::Time::now();

    //time to plan! get a copy of the goal and unlock the mutex
    geometry_msgs::PoseStamped temp_goal = planner_goal_;
    lock.unlock();
    ROS_DEBUG_NAMED("move_base_plan_thread","Planning...");

    //run planner
    planner_plan_->clear();
    bool gotPlan = n.ok() && makePlan(temp_goal, *planner_plan_);

    if(gotPlan){
      ROS_DEBUG_NAMED("move_base_plan_thread","Got Plan with %zu points!", planner_plan_->size());
      //pointer swap the plans under mutex (the controller will pull from latest_plan_)
      std::vector<geometry_msgs::PoseStamped>* temp_plan = planner_plan_;

      lock.lock();
      planner_plan_ = latest_plan_;
      latest_plan_ = temp_plan;
      last_valid_plan_ = ros::Time::now();
      planning_retries_ = 0;
      new_global_plan_ = true;

      ROS_DEBUG_NAMED("move_base_plan_thread","Generated a plan from the base_global_planner");

      //make sure we only start the controller if we still haven't reached the goal
      if(runPlanner_)
        state_ = CONTROLLING;
      if(planner_frequency_ <= 0)
        runPlanner_ = false;
      lock.unlock();
    }
    //if we didn't get a plan and we are in the planning state (the robot isn't moving)
    else if(state_==PLANNING){
      ROS_DEBUG_NAMED("move_base_plan_thread","No Plan...");
      ros::Time attempt_end = last_valid_plan_ + ros::Duration(planner_patience_);

      //check if we've tried to make a plan for over our time limit or our maximum number of retries
      //issue #496: we stop planning when one of the conditions is true, but if max_planning_retries_
      //is negative (the default), it is just ignored and we have the same behavior as ever
      lock.lock();
      planning_retries_++;
      if(runPlanner_ &&
          (ros::Time::now() > attempt_end || planning_retries_ > uint32_t(max_planning_retries_))){
        //we'll move into our obstacle clearing mode
        state_ = CLEARING;
        runPlanner_ = false;  // proper solution for issue #523
        publishZeroVelocity();
        recovery_trigger_ = PLANNING_R;
      }

      lock.unlock();
    }

    //take the mutex for the next iteration
    lock.lock();

    //setup sleep interface if needed
    if(planner_frequency_ > 0){
      ros::Duration sleep_time = (start_time + ros::Duration(1.0/planner_frequency_)) - ros::Time::now();
      if (sleep_time > ros::Duration(0.0)){
        wait_for_wake = true;
        timer = n.createTimer(sleep_time, &MoveBase::wakePlanner, this);
      }
    }
  }
}


void MoveBase::executeCb(const move_base_msgs::MoveBaseGoalConstPtr& move_base_goal)
{
  if(!isQuaternionValid(move_base_goal->target_pose.pose.orientation)){
    as_->setAborted(move_base_msgs::MoveBaseResult(), "Aborting on goal because it was sent with an invalid quaternion");
    return;
  }

  geometry_msgs::PoseStamped goal = goalToGlobalFrame(move_base_goal->target_pose);

  publishZeroVelocity();
  //we have a goal so start the planner
  boost::unique_lock<boost::recursive_mutex> lock(planner_mutex_);
  planner_goal_ = goal;
  runPlanner_ = true;
  planner_cond_.notify_one();
  lock.unlock();

  current_goal_pub_.publish(goal);

  ros::Rate r(controller_frequency_);
  if(shutdown_costmaps_){
    ROS_DEBUG_NAMED("move_base","Starting up costmaps that were shut down previously");
    planner_costmap_ros_->start();
    controller_costmap_ros_->start();
  }

  //we want to make sure that we reset the last time we had a valid plan and control
  last_valid_control_ = ros::Time::now();
  last_valid_plan_ = ros::Time::now();
  last_oscillation_reset_ = ros::Time::now();
  planning_retries_ = 0;

  ros::NodeHandle n;
  while(n.ok())
  {
    if(c_freq_change_)
    {
      ROS_INFO("Setting controller frequency to %.2f", controller_frequency_);
      r = ros::Rate(controller_frequency_);
      c_freq_change_ = false;
    }

    if(as_->isPreemptRequested()){
      if(as_->isNewGoalAvailable()){
        //if we're active and a new goal is available, we'll accept it, but we won't shut anything down
        move_base_msgs::MoveBaseGoal new_goal = *as_->acceptNewGoal();

        if(!isQuaternionValid(new_goal.target_pose.pose.orientation)){
          as_->setAborted(move_base_msgs::MoveBaseResult(), "Aborting on goal because it was sent with an invalid quaternion");
          return;
        }

        goal = goalToGlobalFrame(new_goal.target_pose);

        //we'll make sure that we reset our state for the next execution cycle
        recovery_index_ = 0;
        state_ = PLANNING;

        //we have a new goal so make sure the planner is awake
        lock.lock();
        planner_goal_ = goal;
        runPlanner_ = true;
        planner_cond_.notify_one();
        lock.unlock();

        //publish the goal point to the visualizer
        ROS_DEBUG_NAMED("move_base","move_base has received a goal of x: %.2f, y: %.2f", goal.pose.position.x, goal.pose.position.y);
        current_goal_pub_.publish(goal);

        //make sure to reset our timeouts and counters
        last_valid_control_ = ros::Time::now();
        last_valid_plan_ = ros::Time::now();
        last_oscillation_reset_ = ros::Time::now();
        planning_retries_ = 0;
      }
      else {
        //if we've been preempted explicitly we need to shut things down
        resetState();

        //notify the ActionServer that we've successfully preempted
        ROS_DEBUG_NAMED("move_base","Move base preempting the current goal");
        as_->setPreempted();

        //we'll actually return from execute after preempting
        return;
      }
    }

    //we also want to check if we've changed global frames because we need to transform our goal pose
    if(goal.header.frame_id != planner_costmap_ros_->getGlobalFrameID()){
      goal = goalToGlobalFrame(goal);

      //we want to go back to the planning state for the next execution cycle
      recovery_index_ = 0;
      state_ = PLANNING;

      //we have a new goal so make sure the planner is awake
      lock.lock();
      planner_goal_ = goal;
      runPlanner_ = true;
      planner_cond_.notify_one();
      lock.unlock();

      //publish the goal point to the visualizer
      ROS_DEBUG_NAMED("move_base","The global frame for move_base has changed, new frame: %s, new goal position x: %.2f, y: %.2f", goal.header.frame_id.c_str(), goal.pose.position.x, goal.pose.position.y);
      current_goal_pub_.publish(goal);

      //make sure to reset our timeouts and counters
      last_valid_control_ = ros::Time::now();
      last_valid_plan_ = ros::Time::now();
      last_oscillation_reset_ = ros::Time::now();
      planning_retries_ = 0;
    }

    //for timing that gives real time even in simulation
    ros::WallTime start = ros::WallTime::now();

    //the real work on pursuing a goal is done here
    bool done = executeCycle(goal);

    //if we're done, then we'll return from execute
    if(done)
      return;

    //check if execution of the goal has completed in some way

    ros::WallDuration t_diff = ros::WallTime::now() - start;
    ROS_DEBUG_NAMED("move_base","Full control cycle time: %.9f\n", t_diff.toSec());

    r.sleep();
    //make sure to sleep for the remainder of our cycle time
    if(r.cycleTime() > ros::Duration(1 / controller_frequency_) && state_ == CONTROLLING)
      ROS_WARN("Control loop missed its desired rate of %.4fHz... the loop actually took %.4f seconds", controller_frequency_, r.cycleTime().toSec());
  }

  //wake up the planner thread so that it can exit cleanly
  lock.lock();
  runPlanner_ = true;
  planner_cond_.notify_one();
  lock.unlock();

  //if the node is killed then we'll abort and return
  as_->setAborted(move_base_msgs::MoveBaseResult(), "Aborting on the goal because the node has been killed");
  return;
}


bool MoveBase::executeCycle(geometry_msgs::PoseStamped& goal)
{
  boost::recursive_mutex::scoped_lock ecl(configuration_mutex_);
  //we need to be able to publish velocity commands
  geometry_msgs::Twist cmd_vel;

  //update feedback to correspond to our curent position
  geometry_msgs::PoseStamped global_pose;
  getRobotPose(global_pose, planner_costmap_ros_);
  const geometry_msgs::PoseStamped& current_position = global_pose;

  //push the feedback out
  move_base_msgs::MoveBaseFeedback feedback;
  feedback.base_position = current_position;
  as_->publishFeedback(feedback);

  //check to see if we've moved far enough to reset our oscillation timeout
  if(distance(current_position, oscillation_pose_) >= oscillation_distance_)
  {
    last_oscillation_reset_ = ros::Time::now();
    oscillation_pose_ = current_position;

    //if our last recovery was caused by oscillation, we want to reset the recovery index
    if(recovery_trigger_ == OSCILLATION_R)
      recovery_index_ = 0;
  }

  //check that the observation buffers for the costmap are current, we don't want to drive blind
  if(!controller_costmap_ros_->isCurrent()){
    ROS_WARN("[%s]:Sensor data is out of date, we're not going to allow commanding of the base for safety",ros::this_node::getName().c_str());
    publishZeroVelocity();
    return false;
  }

  //if we have a new plan then grab it and give it to the controller
  if(new_global_plan_){
    //make sure to set the new plan flag to false
    new_global_plan_ = false;

    ROS_DEBUG_NAMED("move_base","Got a new plan...swap pointers");

    //do a pointer swap under mutex
    std::vector<geometry_msgs::PoseStamped>* temp_plan = controller_plan_;

    boost::unique_lock<boost::recursive_mutex> lock(planner_mutex_);
    controller_plan_ = latest_plan_;
    latest_plan_ = temp_plan;
    lock.unlock();
    ROS_DEBUG_NAMED("move_base","pointers swapped!");

    if(!tc_->setPlan(*controller_plan_)){
      //ABORT and SHUTDOWN COSTMAPS
      ROS_ERROR("Failed to pass global plan to the controller, aborting.");
      resetState();

      //disable the planner thread
      lock.lock();
      runPlanner_ = false;
      lock.unlock();

      as_->setAborted(move_base_msgs::MoveBaseResult(), "Failed to pass global plan to the controller.");
      return true;
    }

    //make sure to reset recovery_index_ since we were able to find a valid plan
    if(recovery_trigger_ == PLANNING_R)
      recovery_index_ = 0;
  }

  //the move_base state machine, handles the control logic for navigation
  switch(state_){
    //if we are in a planning state, then we'll attempt to make a plan
    case PLANNING:
      {
        boost::recursive_mutex::scoped_lock lock(planner_mutex_);
        runPlanner_ = true;
        planner_cond_.notify_one();
      }
      ROS_DEBUG_NAMED("move_base","Waiting for plan, in the planning state.");
      break;

    //if we're controlling, we'll attempt to find valid velocity commands
    case CONTROLLING:
      ROS_DEBUG_NAMED("move_base","In controlling state.");

      //check to see if we've reached our goal
      if(tc_->isGoalReached()){
        ROS_DEBUG_NAMED("move_base","Goal reached!");
        resetState();

        //disable the planner thread
        boost::unique_lock<boost::recursive_mutex> lock(planner_mutex_);
        runPlanner_ = false;
        lock.unlock();

        as_->setSucceeded(move_base_msgs::MoveBaseResult(), "Goal reached.");
        return true;
      }

      //check for an oscillation condition
      if(oscillation_timeout_ > 0.0 &&
          last_oscillation_reset_ + ros::Duration(oscillation_timeout_) < ros::Time::now())
      {
        publishZeroVelocity();
        state_ = CLEARING;
        recovery_trigger_ = OSCILLATION_R;
      }

      {
        boost::unique_lock<costmap_2d::Costmap2D::mutex_t> lock(*(controller_costmap_ros_->getCostmap()->getMutex()));

      if(tc_->computeVelocityCommands(cmd_vel)){
        ROS_DEBUG_NAMED( "move_base", "Got a valid command from the local planner: %.3lf, %.3lf, %.3lf",
                          cmd_vel.linear.x, cmd_vel.linear.y, cmd_vel.angular.z );
        last_valid_control_ = ros::Time::now();
        //make sure that we send the velocity command to the base
        vel_pub_.publish(cmd_vel);
        if(recovery_trigger_ == CONTROLLING_R)
          recovery_index_ = 0;
      }
      else {
        ROS_DEBUG_NAMED("move_base", "The local planner could not find a valid plan.");
        ros::Time attempt_end = last_valid_control_ + ros::Duration(controller_patience_);

        //check if we've tried to find a valid control for longer than our time limit
        
        if(ros::Time::now() > attempt_end){
          //we'll move into our obstacle clearing mode
          publishZeroVelocity();
          state_ = CLEARING;
          recovery_trigger_ = CONTROLLING_R;
        }
        else{
          //otherwise, if we can't find a valid control, we'll go back to planning
          if(!stop_at_obstacle_)
          {
            last_valid_plan_ = ros::Time::now();
            planning_retries_ = 0;
            state_ = PLANNING;
            publishZeroVelocity();

            //enable the planner thread in case it isn't running on a clock
            boost::unique_lock<boost::recursive_mutex> lock(planner_mutex_);
            runPlanner_ = true;
            planner_cond_.notify_one();
            lock.unlock();
          }
        }
      }
      }

      break;

    //we'll try to clear out space with any user-provided recovery behaviors
    case CLEARING:
      ROS_DEBUG_NAMED("move_base","In clearing/recovery state");
      //we'll invoke whatever recovery behavior we're currently on if they're enabled
      if(recovery_behavior_enabled_ && recovery_index_ < recovery_behaviors_.size()){
        ROS_DEBUG_NAMED("move_base_recovery","Executing behavior %u of %zu", recovery_index_+1, recovery_behaviors_.size());

        move_base_msgs::RecoveryStatus msg;
        msg.pose_stamped = current_position;
        msg.current_recovery_number = recovery_index_;
        msg.total_number_of_recoveries = recovery_behaviors_.size();
        msg.recovery_behavior_name =  recovery_behavior_names_[recovery_index_];

        recovery_status_pub_.publish(msg);

        recovery_behaviors_[recovery_index_]->runBehavior();

        //we at least want to give the robot some time to stop oscillating after executing the behavior
        last_oscillation_reset_ = ros::Time::now();

        //we'll check if the recovery behavior actually worked
        ROS_DEBUG_NAMED("move_base_recovery","Going back to planning state");
        last_valid_plan_ = ros::Time::now();
        planning_retries_ = 0;
        state_ = PLANNING;

        //update the index of the next recovery behavior that we'll try
        recovery_index_++;
      }
      else{
        ROS_DEBUG_NAMED("move_base_recovery","All recovery behaviors have failed, locking the planner and disabling it.");
        //disable the planner thread
        boost::unique_lock<boost::recursive_mutex> lock(planner_mutex_);
        runPlanner_ = false;
        lock.unlock();

        ROS_DEBUG_NAMED("move_base_recovery","Something should abort after this.");

        if(recovery_trigger_ == CONTROLLING_R){
          ROS_ERROR("Aborting because a valid control could not be found. Even after executing all recovery behaviors");
          as_->setAborted(move_base_msgs::MoveBaseResult(), "Failed to find a valid control. Even after executing recovery behaviors.");
        }
        else if(recovery_trigger_ == PLANNING_R){
          ROS_ERROR("Aborting because a valid plan could not be found. Even after executing all recovery behaviors");
          as_->setAborted(move_base_msgs::MoveBaseResult(), "Failed to find a valid plan. Even after executing recovery behaviors.");
        }
        else if(recovery_trigger_ == OSCILLATION_R){
          ROS_ERROR("Aborting because the robot appears to be oscillating over and over. Even after executing all recovery behaviors");
          as_->setAborted(move_base_msgs::MoveBaseResult(), "Robot is oscillating. Even after executing recovery behaviors.");
        }
        resetState();
        return true;
      }
      break;
    default:
      ROS_ERROR("This case should never be reached, something is wrong, aborting");
      resetState();
      //disable the planner thread
      boost::unique_lock<boost::recursive_mutex> lock(planner_mutex_);
      runPlanner_ = false;
      lock.unlock();
      as_->setAborted(move_base_msgs::MoveBaseResult(), "Reached a case that should not be hit in move_base. This is a bug, please report it.");
      return true;
  }

  //we aren't done yet
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

  //if we shutdown our costmaps when we're deactivated... we'll do that now
  if(shutdown_costmaps_){
    ROS_DEBUG_NAMED("move_base","Stopping costmaps");
    planner_costmap_ros_->stop();
    controller_costmap_ros_->stop();
  }
}

};   // namespace
