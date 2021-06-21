#include "map_swapper/map_swapper.h"

MapSwapper::MapSwapper() 
: rate_(2), have_transitions_(false)
, robot_frame_("base_link"), map_frame_("map")
, tf_ear_(tf_buffer_), pose_inited_(false)
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

  // ROS_INFO("bounds check for (%5.2f, %5.2f) against (%5.2f, %5.2f), (%5.2f, %5.2f)",
          //  pose.position.x, pose.position.y, x0, y0, x1, y1);

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
  if (!pose_inited_)
  {
    prev_pose_ = robot_pose;
    pose_inited_ = true;
  }

  // check if out of bounds
  bool in_bounds = checkInBounds(robot_pose, piece_id_);

  // if out of bounds, find piece where robot is within bounds
  if (!in_bounds)
  {
    ROS_INFO("robot is not in this piece anymore, finding new piece");
    findInBoundPiece(robot_pose);
    prev_pose_ = robot_pose;
    return;
  }

  // check for transition
  YAML::Node piece_trans = transitions_[piece_id_]["transitions"];
  // ROS_INFO("%lu transitions possible from here", piece_trans.size());
  // std::cout << piece_trans << std::endl;
  Pt2D robot_pt, prev_robot_pt;
  robot_pt.x = robot_pose.position.x;
  robot_pt.y = robot_pose.position.y;
  prev_robot_pt.x = prev_pose_.position.x;
  prev_robot_pt.y = prev_pose_.position.y;

  bool have_secondary_candidate = false;
  std::string best_secondary_candidate = "";
  double secondary_candidate_score = 1000.0;

  // ROS_INFO("pose (%5.2f, %5.2f), prev (%5.2f, %5.2f)", robot_pt.x, robot_pt.y,
          //  prev_robot_pt.x, prev_robot_pt.y);

  for (int i = 0; i < piece_trans.size(); i++)
  {
    Pt2D ln0, ln1;
    ln0.x = piece_trans[i]["pt0"]["x"].as<double>();
    ln0.y = piece_trans[i]["pt0"]["y"].as<double>();
    ln1.x = piece_trans[i]["pt1"]["x"].as<double>();
    ln1.y = piece_trans[i]["pt1"]["y"].as<double>();

    double prev_side = sideCheck(prev_robot_pt, ln0, ln1);
    double curr_side = sideCheck(robot_pt, ln0, ln1);
    double projection = alongCheck(robot_pt, ln0, ln1);

    // ROS_INFO("%d, line (%5.2f, %5.2f), (%5.2f, %5.2f), side, %5.2f, %5.2f, proj %5.2f",
            //  i, ln0.x, ln0.y, ln1.x, ln1.y, prev_side, curr_side, projection);

    if (prev_side > 0 && curr_side < 0)
    {
      std::string next_piece = piece_trans[i]["dst"].as<std::string>();
      if (projection > 0 && projection < 1)
      {
        ROS_INFO("transition to %s", next_piece.c_str());
        have_secondary_candidate = false;
        loadMapPiece(next_piece);
        break;
      }

      have_secondary_candidate = true;
      double secondary_score = projection > 0 ? (projection - 1) : fabs(projection);
      if (secondary_score < secondary_candidate_score)
      {
        secondary_candidate_score = secondary_score;
        best_secondary_candidate = next_piece;
      }
    }
  }
  if (have_secondary_candidate)
  {
    ROS_INFO("transition to secondary candidate %s", best_secondary_candidate.c_str());
    loadMapPiece(best_secondary_candidate);
  }
  // ROS_INFO("---");
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

/* 
 * do cross product between (ln1-ln0) and (pt-ln0)
 * if the result is positive, pt is to the left of the line,
 * if it's negative, pt is to the right
 * zero means pt is on the line
 */
double MapSwapper::sideCheck(Pt2D pt, Pt2D ln0, Pt2D ln1)
{
  double vlx, vly, vpx, vpy;
  vlx = ln1.x - ln0.x;
  vly = ln1.y - ln0.y;
  vpx = pt.x - ln0.x;
  vpy = pt.y - ln0.y;

  return vlx*vpy - vly*vpx;
}

/*
 * do dot product between (ln1-ln0) and (pt-ln0), then normalise against line length
 * if the result is in [0, 1] then the point projection is within the line segment
 * otherwise it's outside (-ve means "behind" ln0, >1 means "beyond" ln1)
 */
double MapSwapper::alongCheck(Pt2D pt, Pt2D ln0, Pt2D ln1)
{
  double vlx, vly, vpx, vpy;
  vlx = ln1.x - ln0.x;
  vly = ln1.y - ln0.y;
  vpx = pt.x - ln0.x;
  vpy = pt.y - ln0.y;

  double mag = vlx*vlx + vly*vly;

  return (vlx*vpx + vly*vpy)/mag;
}