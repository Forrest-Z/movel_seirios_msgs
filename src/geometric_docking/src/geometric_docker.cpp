#include <geometric_docking/geometric_docker.hpp>

double calcDistance(geometry_msgs::Pose a, geometry_msgs::Pose b)
{
  double dx, dy, dz;
  
  // Calculate delta
  dx = a.position.x - b.position.x;
  dy = a.position.y - b.position.y;
  dz = a.position.z - b.position.z;
  
  //Calculate distance
  return sqrt(dx*dx + dy*dy + dz*dz);
}

GeometricDocker::GeometricDocker(ros::NodeHandle& nh) : active_(false), have_dock_pose_(false)
, tf_ear_(tf_buffer_), nh_(nh)
{
  setupParams();
  setupTopics(nh_);
}

void GeometricDocker::setupParams()
{
  // Setup parameters
  ros::NodeHandle nh_local("~");

  dock_width_ = 1.0;
  if (nh_local.hasParam("dock_width"))
    nh_local.getParam("dock_width", dock_width_);

  dock_offset_ = 1.0;
  if (nh_local.hasParam("dock_offset"))
    nh_local.getParam("dock_offset", dock_offset_);

  update_period_ = 1.0;
  if (nh_local.hasParam("update_period"))
    nh_local.getParam("update_period", update_period_);

  docking_frame_ = "odom";
  if (nh_local.hasParam("docking_frame"))
    nh_local.getParam("docking_frame", docking_frame_);

  d_lost_th_ = 0.05;
  if (nh_local.hasParam("d_lost_th"))
    nh_local.getParam("d_lost_th", d_lost_th_);

  max_lost_count_ = 3;
  if (nh_local.hasParam("max_lost_count"))
    nh_local.getParam("max_lost_count", max_lost_count_);

  lpf_wt_ = 0.5;
  if (nh_local.hasParam("lpf_wt"))
    nh_local.getParam("lpf_wt", lpf_wt_);

  // Setup parameters in dock_detector node
  dock_detector_.setupParams(dock_width_, dock_offset_);

  ROS_INFO("Params OK");
}

void GeometricDocker::setupTopics(ros::NodeHandle& nh_)
{
  // Subscribers
  scan_sub_ = nh_.subscribe("/scan", 1, &GeometricDocker::scanCb, this);
  dock_reached_sub_ = nh_.subscribe("/goal/status", 1, &GeometricDocker::dockReachedCb, this);

  // Publishers
  dock_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/dock_pose", 1);
  controller_ref_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/pid_goal", 1);
  controller_stop_pub_ = nh_.advertise<std_msgs::Bool>("/stop_now", 1);
  ROS_INFO("Topics OK");
}

void GeometricDocker::scanCb(sensor_msgs::LaserScan scan)
{
  // If package doesn't active, do nothing
  if (!active_){
    return;
  }

  // 1.1 x sampling period
  double dt_estimate = 1.1*update_period_;  

  // Calculate time for looping
  if (have_dock_pose_)
  {
    dt_estimate = (ros::Time::now() - t_prev_estimate_).toSec();
  }

  if (dt_estimate < update_period_)
    return;
  else
  {
    t_prev_estimate_ = scan.header.stamp;
  }
  
  // Find dock pose
  geometry_msgs::Pose dockpose;
  if (dock_detector_.findDock(scan, dockpose))
  {
    geometry_msgs::PoseStamped dockpose_stamped;
    dockpose_stamped.pose = dockpose;
    dockpose_stamped.header = scan.header;
    dock_pose_pub_.publish(dockpose_stamped);

    // publish once at transition from not having estimate to having one
    if (!have_dock_pose_)
    {
      // transform to odom frame (map frame works too, but frames on the robot aren't valid)
      geometry_msgs::TransformStamped transform = tf_buffer_.lookupTransform(docking_frame_, scan.header.frame_id, ros::Time(0));
      geometry_msgs::PoseStamped dockpose_odom;
      tf2::doTransform(dockpose_stamped.pose, dockpose_odom.pose, transform);

      dockpose_odom.header.frame_id = docking_frame_;
      ros::Duration(1.0).sleep();                       // Ensure that the first data that send to planner can be reached
      controller_ref_pub_.publish(dockpose_odom);
      dock_pose_estimate_ = dockpose_odom;

      have_dock_pose_ = true;
    }
    else    // Robot has dock pose
    {
      geometry_msgs::TransformStamped transform = tf_buffer_.lookupTransform(docking_frame_, scan.header.frame_id, ros::Time(0));
      geometry_msgs::Pose curr_dock_pose;
      tf2::doTransform(dockpose_stamped.pose, curr_dock_pose, transform);
      double d = calcDistance(dock_pose_estimate_.pose, curr_dock_pose);
      dock_pose_estimate_.header.stamp = scan.header.stamp;
      ROS_INFO("New estimate distance %5.2f m", d);
      
      if (d > d_lost_th_)     // Ignore if the distance between two dockpose is very far.
      {
        ++lost_count_;
        ROS_INFO("Dock lost %d times, d: %5.2f", lost_count_, d);
        if (lost_count_ >= max_lost_count_)
        {
          std_srvs::Trigger emp;
          dockCancelCb(emp.request, emp.response);    // Call dockCancelCb
        }
      }
      else if (d > 0.5*d_lost_th_)        // only update reference if change is sufficiently high
      {
        dock_pose_estimate_.pose.position.x = lpf_wt_* dock_pose_estimate_.pose.position.x  + (1.0 - lpf_wt_) * curr_dock_pose.position.x;
        dock_pose_estimate_.pose.position.y = lpf_wt_* dock_pose_estimate_.pose.position.y  + (1.0 - lpf_wt_) * curr_dock_pose.position.y;
        dock_pose_estimate_.pose.position.z = lpf_wt_* dock_pose_estimate_.pose.position.z  + (1.0 - lpf_wt_) * curr_dock_pose.position.z;
        dock_pose_estimate_.pose.orientation.z = lpf_wt_* dock_pose_estimate_.pose.orientation.z  + (1.0 - lpf_wt_) * curr_dock_pose.orientation.z;
        dock_pose_estimate_.pose.orientation.w = lpf_wt_* dock_pose_estimate_.pose.orientation.w  + (1.0 - lpf_wt_) * curr_dock_pose.orientation.w;
        controller_ref_pub_.publish(dock_pose_estimate_);
        
        lost_count_ = 0;
        // dock_pose_estimate_.pose = curr_dock_pose;
      }
    }
  }
  else
  {
    // if no dock found, it's likely we're asked to dock where there isn't any dock, 
    // unless we've detected it before and it's just by chance that the laser points don't line up well enough
    // the save way to handle this is to cancel docking and let higher level logic evaluate when to try again
    if (!have_dock_pose_)
    {
      ROS_INFO("No dock here");
      active_ = false;  
      ros::Duration(1.0).sleep();                 // Ensure that the docking_status value that send to handler can be reached
      std_srvs::Trigger emp;
      dockCancelCb(emp.request, emp.response);    // Call dockCancelCb
      ros::Duration(1.0).sleep();
    }
    else
    {
      ++lost_count_;      
      if (lost_count_ >= max_lost_count_)
      {
        std_srvs::Trigger emp;
        dockCancelCb(emp.request, emp.response);    // Call dockCancelCb

      }
    }
  }
}  

bool GeometricDocker::dockStartCb(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
  ROS_INFO("Geometric Docker has been started");

  // Activating scanCb 
  active_ = true;
  have_dock_pose_ = false;
  lost_count_ = 0;

  return true;
}

bool GeometricDocker::dockCancelCb(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
  ROS_INFO("Geometric Docker has been canceled");

  // Turn off scanCb
  active_ = false;
  have_dock_pose_ = false;

  // Send cancel message to PID
  std_msgs::Bool cancel_msg;
  cancel_msg.data = true;
  controller_stop_pub_.publish(cancel_msg);   // Stop planner

  std_msgs::UInt8 dock_status;
  dock_status.data = 3;                       // send "failed" message to handler
  dock_status_pub_.publish(dock_status);
  res.success = true;
  return true;
}

void GeometricDocker::dockReachedCb(std_msgs::Bool msg)
{
  ROS_INFO("Docking complete");
  std_msgs::UInt8 dock_status;
  dock_status.data = 2;             // send "completed" message to handler 
  dock_status_pub_.publish(dock_status);

  if (msg.data && active_)        // Stop scanCb
  {
    active_ = false;
    have_dock_pose_ = false;
  }
}
bool GeometricDocker::dockResumeCb(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
  ROS_INFO("Geometric Docking has been resumed");

  active_ = true;
  lost_count_ = 0;

  controller_ref_pub_.publish(dock_pose_estimate_);
  have_dock_pose_ = true;
  res.success = true;
  return true;
}

bool GeometricDocker::dockPauseCb(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
  ROS_INFO("Geometric Docker has been paused");

  // Turn off scanCb
  active_ = false;
  have_dock_pose_ = false;

  // Send cancel message to PID
  std_msgs::Bool cancel_msg;
  cancel_msg.data = true;
  controller_stop_pub_.publish(cancel_msg);   // Stop planner

  res.success = true;
  return true;
}