#include <path_recall/path_saver.h>

PathSaver::PathSaver() : start(false), save(false)
{
}

//! Start path saving
bool PathSaver::Start(std::string name)
{
  //! Create file to save the path in
  std::ofstream outfile(yaml_path + "/" + name + ".yaml", std::ofstream::out | std::ofstream::trunc);
  if (outfile.fail())
  {
    return false;
  }
  outfile.close();

  path_name = name;
  count = 1;
  start = true;
  save = true;
  return true;
}

//! Callback for start of path saving
bool PathSaver::onStart(path_recall::PathName::Request& req, path_recall::PathName::Response& res)
{
  if (Start(req.name))
  {
    ROS_INFO("[path saver] Saving to file dir %s", (yaml_path + "/" + req.name + ".yaml").c_str());
    res.success = true;
  }
  else
  {
    ROS_ERROR("[path saver] Fail to create path file");
    res.success = false;
  }
  return true;
}

//! Stop path saving
void PathSaver::Stop()
{
  save = false;
  count = 0;
}

//! Callback for end of path saving
bool PathSaver::onStop(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res)
{
  Stop();
  res.success = true;
  return true;
}

//! Callback for getting current pose of robot to be saved
void PathSaver::onSave(const geometry_msgs::Pose::ConstPtr& msg)
{
  if (save)
  {
    if (start)
    {
      previous_point = *msg;
      start = false;
      writeWaypoint(*msg);
    }
    else
    {
      if (calculateDistance(previous_point, *msg))
      {
        previous_point = *msg;
        count++;
        writeWaypoint(*msg);
      }
    }
  }
}

//! Calculate distance of last saved waypoint to current pose
bool PathSaver::calculateDistance(geometry_msgs::Pose pose1, geometry_msgs::Pose pose2)
{
  double linear_distance =
      sqrt(pow((pose1.position.x - pose2.position.x), 2) + pow((pose1.position.y - pose2.position.y), 2) +
           pow((pose1.position.z - pose2.position.z), 2));

  tf::Quaternion q1(pose1.orientation.x, pose1.orientation.y, pose1.orientation.z, pose1.orientation.w),
      q2(pose2.orientation.x, pose2.orientation.y, pose2.orientation.z, pose2.orientation.w);
  tf::Matrix3x3 m1(q1), m2(q2);
  double r1, p1, y1, r2, p2, y2;
  m1.getRPY(r1, p1, y1);
  m2.getRPY(r2, p2, y2);
  double angular_difference = abs(y1 - y2);

  if (linear_distance > update_min_d || angular_difference > update_min_a)
  {
    return true;
  }
  else
  {
    return false;
  }
}

//! Write waypoint to yaml file
void PathSaver::writeWaypoint(geometry_msgs::Pose pose)
{
  config = YAML::LoadFile(yaml_path + path_name + ".yaml");
  toYAML(config, pose, path_name);
  yaml_file.open(yaml_path + path_name + ".yaml");
  yaml_file << config;
  yaml_file.close();
}

//! Write path to yaml file
bool PathSaver::writePath(std::string name, nav_msgs::Path path)
{
  std::ofstream outfile(yaml_path + name + ".yaml", std::ofstream::out | std::ofstream::trunc);
  if (outfile.fail())
  {
    ROS_ERROR("[path saver] Error saving path file");
    return false;
  }
  outfile.close();

  count = 1;
  config = YAML::LoadFile(yaml_path + name + ".yaml");
  for (size_t i = 0; i < path.poses.size(); i++)
  {
    toYAML(config, path.poses[i].pose, name);
    count++;
  }
  yaml_file.open(yaml_path + name + ".yaml");
  yaml_file << config;
  yaml_file.close();

  if (path.header.frame_id != "map" || path.poses.size() == 0)
  {
    return false;
  }
  return true;
}

//! Format waypoint into yaml data
void PathSaver::toYAML(YAML::Node config, geometry_msgs::Pose pose, std::string name)
{
  config[name]["G" + std::to_string(count)]["position"]["x"] = pose.position.x;
  config[name]["G" + std::to_string(count)]["position"]["y"] = pose.position.y;
  config[name]["G" + std::to_string(count)]["position"]["z"] = pose.position.z;
  config[name]["G" + std::to_string(count)]["orientation"]["x"] = pose.orientation.x;
  config[name]["G" + std::to_string(count)]["orientation"]["y"] = pose.orientation.y;
  config[name]["G" + std::to_string(count)]["orientation"]["z"] = pose.orientation.z;
  config[name]["G" + std::to_string(count)]["orientation"]["w"] = pose.orientation.w;
}

//! Get path to be saved through service call
bool PathSaver::savePath(path_recall::SavePath::Request& req, path_recall::SavePath::Response& res)
{
  if (writePath(req.name, req.path))
  {
    res.success = true;
  }
  else
  {
    res.success = false;
  }
  return true;
}
