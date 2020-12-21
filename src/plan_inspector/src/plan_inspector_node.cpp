#include <plan_inspector/plan_inspector.hpp>

PlanInspector::PlanInspector()
: enable_(true), have_plan_(false), have_costmap_(false)
, have_action_status_(false), timer_active_(false), path_obstructed_(false)
, tf_ear_(tf_buffer_)
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

  zero_vel_.linear.x = 0.0;
  zero_vel_.linear.y = 0.0;
  zero_vel_.linear.z = 0.0;
  zero_vel_.angular.x = 0.0;
  zero_vel_.angular.y = 0.0;
  zero_vel_.angular.z = 0.0;
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

  return true;
}

bool PlanInspector::setupTopics()
{
  plan_sub_ = nh_.subscribe(plan_topic_, 1, &PlanInspector::pathCb, this);
  costmap_sub_ = nh_.subscribe(costmap_topic_, 1, &PlanInspector::costmapCb, this);

  string action_status_topic = action_server_name_ + "/status";
  action_status_sub_ = nh_.subscribe(action_status_topic, 1, &PlanInspector::actionStatusCb, this);
  
  string action_cancel_topic = action_server_name_ + "/cancel";
  action_cancel_pub_ = nh_.advertise<actionlib_msgs::GoalID>(action_cancel_topic, 1);

  zerovel_pub_ = nh_.advertise<geometry_msgs::Twist>(cmd_vel_topic_, 1);

  enable_sub_ = nh_.subscribe("/enable_plan_inspector", 1,
                              &PlanInspector::enableCb, this);

  return true;
}

void PlanInspector::pathCb(nav_msgs::Path msg)
{
  // ROS_INFO("new path");
  latest_plan_ = msg;
  have_plan_ = true;

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
  if (have_plan_ && have_costmap_ && enable_)
  {
    bool obstructed = checkObstruction();
    // ROS_INFO("obstruction %d", obstructed);

    // check for rising edge, start timers
    if (!path_obstructed_ && obstructed)
    {
      ROS_INFO("newly obstructed. Stop robot, countdown to abort");
      abort_timer_.setPeriod(ros::Duration(clearing_timeout_));
      abort_timer_.start();
      control_timer_.start();
    }
    // check for falling edge, stop timers
    else if (path_obstructed_ && !obstructed)
    {
      abort_timer_.stop();
      control_timer_.stop();
    }

    path_obstructed_ = obstructed;
  }
}

void PlanInspector::actionStatusCb(actionlib_msgs::GoalStatusArray msg)
{
  if (msg.status_list.size() > 0)
  {
    latest_goal_status_ = msg.status_list[0];
    have_action_status_ = true;
  }
}

void PlanInspector::abortTimerCb(const ros::TimerEvent& msg)
{
  ROS_INFO("obstructed long enough. Abort action");
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
  }
}

void PlanInspector::controlTimerCb(const ros::TimerEvent& msg)
{
  if (path_obstructed_)
    zerovel_pub_.publish(zero_vel_);
}

void PlanInspector::enableCb(std_msgs::Bool msg)
{
  enable_ = msg.data;
  if (enable_)
  {
    ROS_INFO("Plan inspector is now ON");
  }
  else
  {
    ROS_INFO("Plan inspector is now OFF");
  }
  
}

bool PlanInspector::checkObstruction()
{
  if (!have_costmap_ || ! have_plan_)
    return false;

  int max_occupancy = 0;
  
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
      if (occupancy > obstruction_threshold_)
      {
        ROS_INFO_STREAM("pose in same frame as local costmap \n" << pose_costmap.pose.position);
        ROS_INFO_STREAM("pose in local costmap frame \n" << pose_costmap_local.pose.position);
        ROS_INFO_STREAM("row: " << row << " col: " << col << " occ: " << occupancy);
      }
      
      if (occupancy > max_occupancy)
        max_occupancy = occupancy;
    }
  }
  // ROS_INFO("max occupancy %d/%d", max_occupancy, obstruction_threshold_);
  if (max_occupancy >= obstruction_threshold_)
  {
    ROS_INFO("obstruction %d/%d", max_occupancy, obstruction_threshold_);
    return true;
  }
    
  return false;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "plan_inspector");

  PlanInspector pinspector;
  ros::spin();

  return 0;
}