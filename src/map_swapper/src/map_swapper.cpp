#include "map_swapper/map_swapper.h"

MapSwapper::MapSwapper() 
: rate_(2), have_transitions_(false)
, robot_frame_("base_link"), map_frame_("map")
, tf_ear_(tf_buffer_)
{
  loadParams();
  setupTopics();
}

void MapSwapper::loadParams()
{
  ros::NodeHandle np("~");
  if (np.hasParam("rate"))
    np.getParam("rate", rate_);

  if (np.hasParam("robot_frame"))
    np.getParam("robot_frame", robot_frame_);

  if (np.hasParam("map_frame"))
    np.getParam("map_frame", map_frame_);
}

void MapSwapper::setupTopics()
{
  ros::NodeHandle np("~");
  transitions_pub_ = np.advertise<nav_msgs::Path>("transitions", 1, true);
  load_map_srv_ = np.advertiseService("load_map", &MapSwapper::loadMapSrvCb, this);
  swap_map_clt_ = nh_.serviceClient<nav_msgs::LoadMap>("change_map");
  transition_timer_ = nh_.createTimer(ros::Duration(1.0/rate_), &MapSwapper::transitionTimerCb, this, false, false);
}

bool MapSwapper::checkInBounds(geometry_msgs::Pose pose, std::string piece_id)
{
  YAML::Node piece_spec = transitions_[piece_id];
  double x0, x1, y0, y1;
  x0 = piece_spec["origin"]["x"].as<double>();
  x1 = x0 + piece_spec["width"].as<double>();

  y0 = piece_spec["origin"]["y"].as<double>();
  y1 = y0 + piece_spec["height"].as<double>();

  ROS_INFO("bounds check for (%5.2f, %5.2f) against (%5.2f, %5.2f), (%5.2f, %5.2f)",
           pose.position.x, pose.position.y, x0, y0, x1, y1);

  if (pose.position.x > x0 && pose.position.x < x1
      && pose.position.y > y0 && pose.position.y < y1)
      return true;

  return false;
}

void MapSwapper::transitionTimerCb(const ros::TimerEvent &te)
{
  // ROS_INFO("Check for transition timer");
  // get current pose
  geometry_msgs::TransformStamped robot_transform;
  try
  {
    robot_transform = tf_buffer_.lookupTransform(map_frame_, robot_frame_, ros::Time(0));
  }
  catch(tf2::LookupException &e)
  {
    ROS_INFO("failed to lookup transfrom from %s to %s", map_frame_.c_str(), robot_frame_.c_str());
    return;
  }

  geometry_msgs::Pose robot_pose;
  robot_pose.position.x = robot_transform.transform.translation.x;
  robot_pose.position.y = robot_transform.transform.translation.y;

  // check if out of bounds
  bool in_bounds = checkInBounds(robot_pose, piece_id_);

  // if out of bounds, find piece where robot is within bounds
  if (!in_bounds)
  {
    ROS_INFO("robot is not in this piece anymore, finding new piece");
    findInBoundPiece(robot_pose);
    return;
  }

  // check for transition
  YAML::Node piece_trans = transitions_[piece_id_]["transitions"];
  ROS_INFO("%lu transitions possible from here", piece_trans.size());
  // std::cout << piece_trans << std::endl;
}

bool MapSwapper::loadMapSrvCb(movel_seirios_msgs::StringTrigger::Request &req, 
                              movel_seirios_msgs::StringTrigger::Response &res)
{
  map_dir_ = req.input;

  nav_msgs::LoadMap srv;
  srv.request.map_url = map_dir_ + "/scaled.yaml";
  if (swap_map_clt_.call(srv))
  {
    transitions_ = YAML::LoadFile(map_dir_ + "/transitions.yaml");
    have_transitions_ = true;
    piece_id_ = "0_0";
    transition_timer_.start();

    // std::cout << transitions_ << std::endl;
    publishTransitions();
    res.success = true;
    return true;
  }

  if (transition_timer_.hasStarted())
    transition_timer_.stop();

  have_transitions_ = false;
  return false;
}

void MapSwapper::publishTransitions()
{
  nav_msgs::Path transitions_path;
  transitions_path.header.frame_id = map_frame_;
  transitions_path.header.stamp = ros::Time::now();

  YAML::Node piece_trans_specs = transitions_[piece_id_]["transitions"];
  for (int i = 0; i < piece_trans_specs.size(); i++)
  {
    geometry_msgs::PoseStamped pose_i0, pose_i1;
    
    pose_i0.header.frame_id = map_frame_;
    pose_i0.pose.position.x = piece_trans_specs[i]["pt0"]["x"].as<double>();
    pose_i0.pose.position.y = piece_trans_specs[i]["pt0"]["y"].as<double>();

    pose_i1.header.frame_id = map_frame_;
    pose_i1.pose.position.x = piece_trans_specs[i]["pt1"]["x"].as<double>();
    pose_i1.pose.position.y = piece_trans_specs[i]["pt1"]["y"].as<double>();

    double dx, dy;
    dx = pose_i1.pose.position.x - pose_i0.pose.position.x;
    dy = pose_i1.pose.position.y - pose_i0.pose.position.y;

    double th = atan2(dy, dx);
    pose_i0.pose.orientation.z = sin(0.5*th);
    pose_i0.pose.orientation.w = cos(0.5*th);

    pose_i1.pose.orientation.z = sin(0.5*th);
    pose_i1.pose.orientation.w = cos(0.5*th);

    transitions_path.poses.push_back(pose_i0);
    transitions_path.poses.push_back(pose_i1);
  }
  transitions_pub_.publish(transitions_path);
}

bool MapSwapper::findInBoundPiece(geometry_msgs::Pose pose)
{
  for (const auto& transmap : transitions_)
  {
    std::string piece_i = transmap.first.as<std::string>();
    if (checkInBounds(pose, piece_i))
    {
      if (loadMapPiece(piece_i))
        return true;
    }
  }
  return false;
}

bool MapSwapper::loadMapPiece(std::string piece_id)
{
  if (transitions_[piece_id])
  {
    nav_msgs::LoadMap srv;
    srv.request.map_url = map_dir_ + "/" + piece_id + ".yaml";
    if (swap_map_clt_.call(srv))
    {
      piece_id_ = piece_id;
      publishTransitions();
      return true;
    }
  }
  return false;
}