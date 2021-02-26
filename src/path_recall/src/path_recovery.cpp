#include <path_recall/path_recovery.h>

PathRecovery::PathRecovery() : start(false), save(false)
{
}

//! Load recovery path
bool PathRecovery::Recovery(std::string name)
{
  path_recall::PathName srv;
  srv.request.name = name + "(recovery)";

  if (load_client_.call(srv))
  {
    if (srv.response.success)
    {
      return true;
    }
    else
    {
      return false;
    }
  }
  else
  {
    ROS_ERROR("Failed to call path loader service.");
    return false;
  }
}

//! Callback for recovery service
bool PathRecovery::onRecovery(path_recall::PathName::Request& req, path_recall::PathName::Response& res)
{
  if (Recovery(req.name))
  {
    res.success = true;
  }
  else
  {
    res.success = false;
  }
  return true;
}

//! Callback for getting original path and its name
void PathRecovery::getOriginalPath(const path_recall::PathInfo::ConstPtr& msg)
{
  original_path.poses.clear();
  path_name = msg->name;
  original_path.poses = msg->path.poses;
}

//! Callback for start and end of path loading
void PathRecovery::onStart(const std_msgs::Bool::ConstPtr& msg)
{
  if (msg->data)
  {
    start = true;
    save = true;
  }
  else if (save)
  {
    save = false;
    saveRecoveryPath(path_name, original_path, actual_path);
    actual_path.poses.clear();
  }
}

//! Save recovery path
void PathRecovery::saveRecoveryPath(std::string name, nav_msgs::Path original_path, nav_msgs::Path actual_path)
{
  //! Set recovery path name
  path_recall::SavePath srv;
  if (name.find("(recovery)") != std::string::npos)
  {
    srv.request.name = name;
  }
  else
  {
    srv.request.name = name + "(recovery)";
  }

  //! Generate recovery path
  nav_msgs::Path path;
  path = comparePath(original_path, actual_path);
  srv.request.path = path;

  //! Call path saving service
  if (save_client_.call(srv))
  {
    display_pub_.publish(path);
  }
  else
  {
    ROS_ERROR("Failed to call /path_saver/save service.");
  }
}

//! Callback for getting current pose to be recorded in actual path travelled
void PathRecovery::getPose(const geometry_msgs::Pose::ConstPtr& msg)
{
  geometry_msgs::PoseStamped pose;
  pose.pose = *msg;
  if (save)
  {
    if (start)
    {
      previous_point = *msg;
      start = false;
      actual_path.poses.push_back(pose);
    }
    else
    {
      if (calculateDistance(previous_point, *msg))
      {
        previous_point = *msg;
        actual_path.poses.push_back(pose);
      }
    }
  }
}

//! Calculate distance between last saved waypoint to current pose
bool PathRecovery::calculateDistance(geometry_msgs::Pose pose1, geometry_msgs::Pose pose2)
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

//! Find uncovered segments
nav_msgs::Path PathRecovery::comparePath(nav_msgs::Path original_path, nav_msgs::Path actual_path)
{
  double distance, min_distance;
  std::vector<geometry_msgs::PoseStamped> output_list;
  nav_msgs::Path output;

  //! Compare original loaded path to actual path travelled
  for (size_t i = 0; i < original_path.poses.size(); i++)
  {
    min_distance = 0;
    for (size_t j = 0; j < actual_path.poses.size(); j++)
    {
      distance = sqrt(pow((actual_path.poses[j].pose.position.x - original_path.poses[i].pose.position.x), 2) +
                      pow((actual_path.poses[j].pose.position.y - original_path.poses[i].pose.position.y), 2) +
                      pow((actual_path.poses[j].pose.position.z - original_path.poses[i].pose.position.z), 2));
      if (j == 0)
      {
        min_distance = distance;
      }
      else
      {
        min_distance = std::min(min_distance, distance);
      }
    }

    if (min_distance > tolerance)
    {
      output_list.push_back(original_path.poses[i]);
    }
  }

  //! If uncovered segments are found
  if (output_list.size() > 0)
  {
    nav_msgs::Path output_path;
    output_path.header.frame_id = "map";

    //! Reverse sequence of waypoints so recovery path starts from end of original path
    std::reverse(output_list.begin(), output_list.end());

    //! Calculate new orientation at each waypoint
    double yaw, dy, dx;
    for (size_t i = 1; i < output_list.size(); i++)
    {
      dy = output_list[i].pose.position.y - output_list[i - 1].pose.position.y;
      dx = output_list[i].pose.position.x - output_list[i - 1].pose.position.x;
      yaw = atan2(dy, dx);
      tf::Quaternion q;
      q.setRPY(0, 0, yaw);
      geometry_msgs::Quaternion quat;
      tf::quaternionTFToMsg(q, quat);
      output_list[i - 1].pose.orientation = quat;
      if (i == output_list.size() - 1)
      {
        output_list[i].pose.orientation = quat;
      }
    }

    output_path.poses = output_list;
    output = output_path;
  }
  return output;
}
