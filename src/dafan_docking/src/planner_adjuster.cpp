#include <dafan_docking/planner_adjuster.hpp>

double quaternionToYaw(geometry_msgs::Quaternion q)
{
  double aa = 2.0*(q.w*q.z + q.y*q.x);
  double bb = 1.0 - 2.0 * (q.y*q.y + q.z*q.z);
  double theta = atan2(aa, bb);
  return theta;
}

void normalizeAngle1(double& angle){
  if (angle > M_PI)
    angle -= 2*M_PI;
  else if (angle < -M_PI)
    angle += 2*M_PI;
}

void normalizeAngle2(double& angle){
  if (angle > 0)
    angle -= 2*M_PI;
  else
    angle += 2*M_PI;
}

double calcTheta(const geometry_msgs::PoseStamped& b, const geometry_msgs::Pose& a) {
  double dy = b.pose.position.y - a.position.y;
  double d_x = b.pose.position.x - a.position.x;
  return atan2(dy, d_x);
}

PlannerAdjuster::PlannerAdjuster() : 
  nh_private("~"), tf_ear_(tf_buffer_), has_goal_(false), controller_stage_(0), dist_feasible(true)
{
  if (!getParams() || !setupTopics())
  {
    ROS_ERROR("Parameter error. Try again");
  }

  ROS_INFO("Waiting for valid time");
  ros::Time::waitForValid();
  t_prev_ = ros::Time::now();

  //set up dynamic reconfigure server
  dyn_server_.reset(new dynamic_reconfigure::Server<dafan_docking::planner_adjusterConfig>(own_mutex_, nh_private)); 
  dynamic_reconfigure::Server<dafan_docking::planner_adjusterConfig>::CallbackType f; 
  f = boost::bind(&PlannerAdjuster::reconfigureCB, this, _1, _2 ); 
  dyn_server_ -> setCallback(f); 

  toggleCameraLED(1);
  prev_pose_th_ = 0;
  ROS_INFO("Finished Initialization");

  while(ros::ok()){
    ros::spinOnce();
  }

  stopNow();
}

////////////////////////////////////////////////
//  Initialization
////////////////////////////////////////////////

bool PlannerAdjuster::getParams()
{
  // orientate_yaw_ = false;
  phase1 = true;
  phase2 = false;
  phase3 = false;
  started_phase3_ = false;
  std::vector<double> angle_gains_init, angle_gains_final, dist_gains;
  if (nh_private.hasParam("angle_gains_init") 
      and nh_private.hasParam("angle_gains_final") 
      and nh_private.hasParam("dist_gains"))
  {
    nh_private.getParam("angle_gains_init", angle_gains_init);
    nh_private.getParam("angle_gains_final", angle_gains_final);
    nh_private.getParam("dist_gains", dist_gains);
    angle_PID_init.setGains(angle_gains_init[0], angle_gains_init[1], angle_gains_init[2]);
    angle_PID_final.setGains(angle_gains_final[0], angle_gains_final[1], angle_gains_final[2]);
    dist_PID.setGains(dist_gains[0], dist_gains[1], dist_gains[2]);
  }
  else
    return false;

  ros::param::param<float>("~angle_tol", angle_tol_, 0.0436332);
  ros::param::param<float>("~dist_tol", dist_tol_, 0.01);
  ros::param::param<float>("~final_angle_tol", final_angle_tol_, 0.017453);

  ros::param::param<bool>("~disable_phase_2", disable_phase_2_, false);
  ros::param::param<bool>("~disable_phase_3", disable_phase_3_, false);
  ros::param::param<float>("~reverse_speed", reverse_speed_, -0.02);
  
  //ROS_INFO("angle_gains %5.2f, %5.2f, %5.2f", angle_gains[0], angle_gains[1], angle_gains[2]);
  //ROS_INFO("dist_gains %5.2f, %5.2f, %5.2f", dist_gains[0], dist_gains[1], dist_gains[2]);
  // ROS_INFO("angle_tolerance %5.2f", angle_tol_);
  // ROS_INFO("dist_tolerance %5.2f", dist_tol_);

  return true;
}

bool PlannerAdjuster::setupTopics()
{
  //Publishers
  cmd_vel_pub_ = nh_private.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
  reached_pub_ = nh_private.advertise<std_msgs::Bool>("/goal/status", 1);
  dock_dist_pub_ = nh_private.advertise<std_msgs::Float32>("/dist_to_dock", 1, true);
  goal2_received_pub_ = nh_private.advertise<std_msgs::Bool>("/goal2_received", 1);
  docking_status_pub_ = nh_private.advertise<std_msgs::Int8>("/docking_status", 1);

  //Subscribers
  goal_sub_ = nh_private.subscribe("/pid_goal", 1, &PlannerAdjuster::goalCb, this);
  goal_sub2_ = nh_private.subscribe("/pid_goal2", 1, &PlannerAdjuster::goalCb2, this);
  odom_sub_ = nh_private.subscribe("/odom", 1, &PlannerAdjuster::odometryCb, this);
  stop_now_sub_ = nh_private.subscribe("/stop_now", 1, &PlannerAdjuster::stopNowCb, this);
  batt_sub_ = nh_private.subscribe("/batt_status", 1, &PlannerAdjuster::battCB, this);

  //services
  toggle_device_client_ = nh_.serviceClient<dafan_msgs::ToggleDevices>("toggle_devices");

  return true;
}


////////////////////////////////////////////////
//  PID Controller
////////////////////////////////////////////////

void PlannerAdjuster::doControl(geometry_msgs::Pose current_pose)
{
  double d_x = calcDist(current_pose, current_goal_.pose);
  double d_t = (ros::Time::now() - t_prev_).toSec();
  t_prev_ = ros::Time::now();
  double d_th;  
  double u_th = 0.0;
  double u_x = 0.0;

  double latest_pose_th = quaternionToYaw(latest_pose_.orientation);

  ///////////////////////////
  // check stage progression
  ///////////////////////////
  if (controller_stage_ == 0)
  {
    d_th = fmod((angle_PID_init.getRef() - latest_pose_th), 2*M_PI);
    ROS_INFO_STREAM("angle_PID_init.getRef(): " << angle_PID_init.getRef() << "latest_pose_th: " << latest_pose_th);
    normalizeAngle1(d_th);
    ROS_INFO("  Stage(%d), ang_error(%5.2f), dist_to_goal(%5.2f)", controller_stage_, d_th, d_x);
    //if the angle tolerance is good, go on to stage 1
    if (fabs(d_th) < angle_tol_)
    {
      controller_stage_ = 1;
      dist_PID.reset();
      dist_PID.setRef(0.0);
      // angle_PID.reset();
      // angle_PID.setRef(0.0);
    }
  }
  else if (controller_stage_ == 1)
  {
    ROS_INFO("  Stage(%d), dist_to_goal(%5.2f)", controller_stage_, d_x);

    double d_th_aux = calcTheta(current_goal_, latest_pose_);
    d_th = fmod((angle_PID_init.getRef() - d_th_aux), 2*M_PI);
    
    normalizeAngle1(d_th);

    //ROS_INFO("d_th_aux %f,pid_ref %f , latest_pose_th %f", fabs(d_th), angle_PID.getRef(), d_th_aux);

    //if the angle tolerance is bad,
    //go back to stage 0
    if(fabs(d_th) > 6*angle_tol_)
    {
      controller_stage_ = 0;
      angle_PID_init.reset();
      angle_PID_init.setRef(d_th_aux);
    }
        
    //if the distance tolerance is good,
    //go on to stage 2
    if (d_x < dist_tol_)
    {
      controller_stage_ = 2;
      angle_PID_final.reset();
      double goal_th = quaternionToYaw(current_goal_.pose.orientation);
      angle_PID_final.setRef(goal_th);
    }
  }
  else if (controller_stage_ == 2)
  {
    double goal_th = quaternionToYaw(current_goal_.pose.orientation);
    d_th = fmod((goal_th - latest_pose_th), 2*M_PI);
    
    normalizeAngle1(d_th);
    ROS_INFO("  Stage(%d), ang_error(%5.2f), dist_to_goal(%5.2f)", controller_stage_, d_th, d_x);

    //If tolerance is off, go back to stage 0s
    if (d_x > 6*dist_tol_)
    {
      double d_th_aux = calcTheta(current_goal_, latest_pose_);

      angle_PID_init.reset();
      angle_PID_init.setRef(d_th_aux);
      controller_stage_ = 0;
      dist_PID.reset();
      dist_PID.setRef(0.0);
    }

    if (fabs(d_th) < angle_tol_) 
    {
      geometry_msgs::Twist cmd_vel;
      cmd_vel_pub_.publish(cmd_vel);
      controller_stage_ = 0;
      std_msgs::Bool reached ;
      reached.data = true;
      reached_pub_.publish(reached);
      has_goal_ = true;
      phase1 = false;
      phase2 = true;
      toggleCameraLED(0);
      ROS_INFO("Phase 1: REACHED GOAL");
      if (disable_phase_2_){
        phase3 = true;
      }
      return;
    }
  }

  ///////////////////////////
  // calculate control input based on stage
  ///////////////////////////
  if (controller_stage_ == 0)
  {
    if (fabs(angle_PID_init.getRef() - latest_pose_th) > M_PI){
      normalizeAngle2(latest_pose_th);
    }
    u_th = angle_PID_init.update(latest_pose_th, d_t);
    // ROS_INFO("stage %d, goal %5.2f, state %5.2f", controller_stage_, angle_PID_init.getRef(), latest_pose_th);
    if (prev_pose_th_ == 0){prev_pose_th_ = latest_pose_th;}
    if (fabs(fabs(prev_pose_th_) - fabs(latest_pose_th)) > 0.2){ROS_ERROR("current and prev pose angle differ by more than 0.2, is it a measurement error?");ROS_INFO_STREAM("prev_pose_th_: " << prev_pose_th_ << "latest_pose_th: " << latest_pose_th);}
  prev_pose_th_ = latest_pose_th;}
  else if (controller_stage_ == 1)
  {
    if (fabs(angle_PID_init.getRef() - latest_pose_th) > M_PI)
    {
      normalizeAngle2(latest_pose_th);
    }
    u_x = dist_PID.update(-d_x, d_t);
    u_th = angle_PID_init.update(latest_pose_th, d_t);
    if (fabs(fabs(prev_pose_th_) - fabs(latest_pose_th)) > 0.2){ROS_ERROR("current and prev pose angle differ by more than 0.2, is it a measurement error?");ROS_INFO_STREAM("prev_pose_th_: " << prev_pose_th_ << "latest_pose_th: " << latest_pose_th);}
    // ROS_INFO("stage %d, goal %5.2f, state %5.2f, angle goal %5.2f, angle %5.2f", controller_stage_, dist_PID.getRef(), d_x, angle_PID_init.getRef(), latest_pose_th);
    prev_pose_th_ = latest_pose_th;
  }
  else if (controller_stage_ == 2)
  {
    if (fabs(angle_PID_final.getRef() - latest_pose_th) > M_PI)
    {
      normalizeAngle2(latest_pose_th);
    }
    u_th = angle_PID_final.update(latest_pose_th, d_t);
    // ROS_INFO("stage %d, goal %5.2f, state %5.2f", controller_stage_, angle_PID_final.getRef(), latest_pose_th);
    if (fabs(fabs(prev_pose_th_) - fabs(latest_pose_th)) > 0.2){ROS_ERROR("current and prev pose angle differ by more than 0.2, is it a measurement error?");ROS_INFO_STREAM("prev_pose_th_: " << prev_pose_th_ << "latest_pose_th: " << latest_pose_th);}
    prev_pose_th_ = latest_pose_th;
  }

  geometry_msgs::Twist cmd_vel;
  cmd_vel.angular.z = u_th;
  ROS_INFO_STREAM("d_th: "<< d_th);
  if (controller_stage_ != 0 && d_x < 0.12 && fabs(d_th) > angle_tol_){cmd_vel.linear.x = reverse_speed_;ROS_INFO("REVERSING ROBOT AT 2CM/S!!!");}
  else{ 
  cmd_vel.linear.x = u_x;}
  ROS_INFO_STREAM("Publishing angular speed:" << u_th);
  ROS_INFO_STREAM("Publishing linear speed:" << cmd_vel.linear.x);
  cmd_vel_pub_.publish(cmd_vel);
}

void PlannerAdjuster::doControl2(geometry_msgs::Pose current_pose){
  geometry_msgs::Twist cmd_vel;
  double u_th = 0.0;
  double d_t = (ros::Time::now() - t_prev_).toSec();
  t_prev_ = ros::Time::now();
  double latest_pose_th = quaternionToYaw(latest_pose_.orientation);
  double goal_th = quaternionToYaw(current_goal_.pose.orientation);
  double d_th = fmod((goal_th - latest_pose_th), 2*M_PI);
  normalizeAngle1(d_th);

  //Stop when it has reached the goal
  if (fabs(d_th) < final_angle_tol_){
    cmd_vel_pub_.publish(cmd_vel);
    has_goal_ = false;
    controller_stage_ = 0;
    std_msgs::Bool reached;
    reached.data = true;
    reached_pub_.publish(reached);
    // orientate_yaw_ = false;
    phase2 = false;
    phase3 = true;
    ROS_INFO("Phase 2: REACHED FINAL GOAL");

    return;
  }

  if (fabs(angle_PID_final.getRef() - latest_pose_th) > M_PI){
    normalizeAngle2(latest_pose_th);
  }
  u_th = angle_PID_final.update(latest_pose_th, d_t);

  cmd_vel.angular.z = u_th;
  cmd_vel_pub_.publish(cmd_vel);

  ROS_INFO("DoControl2: u_th(%f), goal_th(%f), d_th(%f)", u_th, goal_th, d_th);
  ROS_INFO("DoControl2: angle_PID_final.getRef(%5.4f), latest_pose_th(%5.4f)", angle_PID_final.getRef(), latest_pose_th);
  return;
}

////////////////////////////////////////////////
//  Callbacks
////////////////////////////////////////////////

void PlannerAdjuster::odometryCb(nav_msgs::Odometry odom_msg)
{
  latest_odom_ = odom_msg;
  if (has_goal_) 
  {
    std_msgs::Int8 status_msg;
    status_msg.data = 0;
    docking_status_pub_.publish(status_msg);

    // ROS_INFO("odometryCb: Goal frame(%s), Odom frame(%s)", current_goal_.header.frame_id.c_str(), odom_msg.header.frame_id.c_str());
    geometry_msgs::TransformStamped transform = tf_buffer_.lookupTransform(current_goal_.header.frame_id, odom_msg.header.frame_id, ros::Time(0));

    geometry_msgs::Pose pose_in_map;
    tf2::doTransform(odom_msg.pose.pose, pose_in_map, transform);
    latest_pose_ = pose_in_map;
    if (calcDist(latest_pose_, current_goal_.pose) > 1.00)
    {
       has_goal_ = false;
       controller_stage_ = 0;
       geometry_msgs::Twist cmd_vel;
       cmd_vel_pub_.publish(cmd_vel);
       ROS_INFO("Not Feasible. Goal is more than 1 meter away");

       std_msgs::Int8 status_msg;
       status_msg.data = -1;
       docking_status_pub_.publish(status_msg);
    }
    else if(!stop_check) 
    {
      ROS_INFO("Control phase1(%d), phase2(%d), phase3(%d)", phase1, phase2, phase3);
      if (phase2 && !disable_phase_2_){ //orientate_yaw_ && 
        doControl2(pose_in_map);
      }
      else if (phase1) {
        doControl(pose_in_map);
      }
      else{
        ROS_INFO("Planner adjuster: IDLE");
        //do nothing
      }
    }
  }
  odom_received = true;
}

void PlannerAdjuster::goalCb(const geometry_msgs::PoseStamped goal_msg)
{
  //TOCHECK
  if (!phase1 && !odom_received){
    return;
  }

  // ROS_INFO("Phase 1: new goal! %5.2f, %5.2f", goal_msg.pose.position.x, goal_msg.pose.position.y);
  current_goal_ = goal_msg; 
  has_goal_ = true; 
  stop_check = false; 
  t_prev_ = ros::Time::now(); 
  
  // ROS_INFO("goalCb: Goal frame(%s), Odom frame(%s)", current_goal_.header.frame_id.c_str(), latest_odom_.header.frame_id.c_str());
  geometry_msgs::TransformStamped transform = tf_buffer_.lookupTransform(current_goal_.header.frame_id, 
                                                                         latest_odom_.header.frame_id, ros::Time(0));

  tf2::doTransform(latest_odom_.pose.pose, latest_pose_, transform);

  if (calcDist(latest_pose_, current_goal_.pose) > 1.00)
    dist_feasible = false;
  controller_stage_ = 0;
  double d_th_aux = calcTheta(current_goal_, latest_pose_);

  angle_PID_init.reset();
  angle_PID_init.setRef(d_th_aux);
  angle_PID_final.reset();
  angle_PID_final.setRef(d_th_aux);

  // std_msgs::Bool received_msg;
  // received_msg.data = true;
  // goal1_received_pub_.publish(received_msg);
}

void PlannerAdjuster::goalCb2(const geometry_msgs::PoseStamped goal2_msg)
{
  if (!phase2){
    return;
  }

  // orientate_yaw_ = true;
  double th_aux = quaternionToYaw(goal2_msg.pose.orientation);
  // double theta = quaternionToYaw(latest_pose_.orientation);
  // ROS_INFO("Current yaw(%f), goal yaw(%f)", theta, th_aux);

  // ROS_INFO("Phase 2: new goal! x(%5.2f), y(%5.2f), yaw(%5.2f)", goal2_msg.pose.position.x, goal2_msg.pose.position.y, th_aux);
  current_goal_ = goal2_msg;
  has_goal_ = true;
  stop_check = false; 
  t_prev_ = ros::Time::now();

  // ROS_INFO("goalcb2: Goal frame(%s), Odom frame(%s)", current_goal_.header.frame_id.c_str(), latest_odom_.header.frame_id.c_str());
  geometry_msgs::TransformStamped transform = tf_buffer_.lookupTransform(current_goal_.header.frame_id, 
                                                                         latest_odom_.header.frame_id, ros::Time(0));
  tf2::doTransform(latest_odom_.pose.pose, latest_pose_, transform);
  controller_stage_ = 0;

  angle_PID_final.reset();
  angle_PID_final.setRef(th_aux);

  std_msgs::Bool received_msg;
  received_msg.data = true;
  goal2_received_pub_.publish(received_msg);
}

void PlannerAdjuster::battCB(const dafan_msgs::BatteryStatus& msg){
  if (disable_phase_3_){
    return;
  }
  
  // Callback only does work during phase 3
  if (phase3){
    geometry_msgs::Twist cmd_vel; 
    if(msg.is_charging == 0 || msg.is_charging == 1){
      ROS_INFO("  DOCKED: Stopped movement.");
      //stop the robot
      cmd_vel_pub_.publish(cmd_vel);
      phase3 = false;
      
      has_goal_ = false;
      std_msgs::Int8 status_msg;
      status_msg.data = 1;
      docking_status_pub_.publish(status_msg);
    }
    else{
      if (started_phase3_== false){  
        pose_phase3_ = latest_odom_.pose.pose;
        started_phase3_ = true;
      }
     
      geometry_msgs::Pose pose_i = latest_odom_.pose.pose;
      if (calcDist(pose_phase3_, pose_i) < 0.15){
        ROS_INFO("  DOCKING: Moving forward at 0.02 m/s.");
        ROS_INFO_STREAM("pose_phase3_: " << pose_phase3_);
        ROS_INFO_STREAM("pose_i: " << pose_i);
        ROS_INFO_STREAM("dist traveled since start of phase3: " << calcDist(pose_phase3_, pose_i));
        cmd_vel.linear.x = 0.02;
        cmd_vel_pub_.publish(cmd_vel);
      }
      else{
        cmd_vel_pub_.publish(cmd_vel);
        phase3 = false;
        ROS_WARN("Robot has stopped due to charging failure. Please check the charging outlet.");//stop the robot if it has moved more than 23cm since phase 3 started but still not charging

        has_goal_ = false;
        std_msgs::Int8 status_msg;
        status_msg.data = -1;
        docking_status_pub_.publish(status_msg);
      }
    }
  }
  return;
}

void PlannerAdjuster::stopNowCb(const std_msgs::Bool stop_msg)
{
  if (stop_msg.data)
    stopNow();
  else
    stop_check = false;
}

void PlannerAdjuster::reconfigureCB(dafan_docking::planner_adjusterConfig &config, uint32_t level) 
{ 
  ROS_INFO("Planner Adjuster parameters have been dynamically reconfigured");
  angle_PID_init.setGains(config.ang_gains_init_p, 
                          config.ang_gains_init_i, 
                          config.ang_gains_init_d);
  angle_PID_final.setGains(config.ang_gains_final_p, 
                            config.ang_gains_final_i, 
                            config.ang_gains_final_d);
  dist_PID.setGains(config.dist_gains_p, 
                    config.dist_gains_i, 
                    config.dist_gains_d);

  angle_tol_ = config.angle_tol;
  dist_tol_ = config.dist_tol;

}


////////////////////////////////////////////////
//  Helper Functions
////////////////////////////////////////////////

double PlannerAdjuster::calcDist(geometry_msgs::Pose a, geometry_msgs::Pose b)
{
  double dx, dy;
  dx = b.position.x - a.position.x;
  dy = b.position.y - a.position.y;
  double value = sqrt(dx*dx + dy*dy);
  
  std_msgs::Float32 dock_dist_msg;
  dock_dist_msg.data = value;
  dock_dist_pub_.publish(dock_dist_msg);
  return value;
}

void PlannerAdjuster::stopNow()
{
  stop_check = true;
  geometry_msgs::Twist cmd_vel;
  cmd_vel_pub_.publish(cmd_vel);
  has_goal_ = false;
  controller_stage_ = 0;
  ROS_INFO("Stopped the robot.");  
}

void PlannerAdjuster::toggleCameraLED(const int& toggle_){
  //service call to turn on/off LED lights
  dafan_msgs::ToggleDevices srv_;
  srv_.request.dev_num = 9; //Camera LED lights
  srv_.request.toggle = toggle_; //Toggle ON

  if (toggle_device_client_.call(srv_)){
    ROS_INFO("Toggled lights with %d", srv_.response.success);
  }
  else{
    ROS_ERROR("Service call failed to toggle lights");
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "planner_adjuster");

  PlannerAdjuster planner_adjuster;

  return 0;
}
