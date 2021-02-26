#include <path_recall/path_load_segments.h>

PathLoadSegments::PathLoadSegments() : start(false), pause(false), cancel(false), end(true), current_index(0)
{
}

//! Load path from YAML file
bool PathLoadSegments::loadYAML(std::string name, nav_msgs::Path& output_path)
{
  path_name = name;
  nav_msgs::Path path;
  path.header.frame_id = "map";

  //! Check if file exists
  try
  {
    config = YAML::LoadFile(yaml_path + path_name + ".yaml");
  }
  catch (YAML::BadFile& e)
  {
    ROS_WARN("No such file for loading. Attempted to open %s", std::string(yaml_path + name + ".yaml").c_str());
    cancel = true;
    end = true;
    return false;
  }

  //! Check if file is empty
  try
  {
    if (config[name])
    {
      if (config[name].size() == 0)
      {
        ROS_WARN("Not a valid path.");
        return false;
      }

      //! Process data from yaml file
      for (size_t count = 1; count <= config[name].size(); count++)
      {
        geometry_msgs::PoseStamped point;
        point.header.frame_id = "map";

        point.pose.position.x = config[name]["G" + std::to_string(count)]["position"]["x"].as<double>();
        point.pose.position.y = config[name]["G" + std::to_string(count)]["position"]["y"].as<double>();
        point.pose.position.z = config[name]["G" + std::to_string(count)]["position"]["z"].as<double>();
        point.pose.orientation.x = config[name]["G" + std::to_string(count)]["orientation"]["x"].as<double>();
        point.pose.orientation.y = config[name]["G" + std::to_string(count)]["orientation"]["y"].as<double>();
        point.pose.orientation.z = config[name]["G" + std::to_string(count)]["orientation"]["z"].as<double>();
        point.pose.orientation.w = config[name]["G" + std::to_string(count)]["orientation"]["w"].as<double>();

        path.poses.push_back(point);
      }
      output_path = path;
      path_recall::PathInfo info;
      info.name = path_name;
      info.path = path;
      info_pub_.publish(info);
      if (loadPath(path))
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
      ROS_WARN("Invalid file. Attempted to open %s", std::string(yaml_path + path_name + ".yaml").c_str());
      return false;
    }
  }
  catch (YAML::BadSubscript& e)
  {
    ROS_WARN("Empty file.");
    return false;
  }
}

//! Load path
bool PathLoadSegments::loadPath(nav_msgs::Path path)
{
  if (path.header.frame_id == "map" && path.poses.size() > 0)
  {
    current_index = 0;
    cancel = false;
    end = false;
    loaded_path = path;
    display_pub_.publish(loaded_path);
    publishPath();
    return true;
  }
  else
  {
    return false;
  }
}

//! Callback for path loading service
bool PathLoadSegments::onLoad(path_recall::PathName::Request& req, path_recall::PathName::Response& res)
{
  nav_msgs::Path path;
  if (loadYAML(req.name, path))
  {
    res.success = true;
  }
  else
  {
    res.success = false;
  }
  return true;
}

//! Check robot pose w.r.t. first waypoint
bool PathLoadSegments::Check(std::string name, float threshold)
{
  try
  {
    config = YAML::LoadFile(yaml_path + name + ".yaml");
  }
  catch (YAML::BadFile& e)
  {
    ROS_WARN("No such file for loading. Attempted to open %s", std::string(yaml_path + name + ".yaml").c_str());
    return false;
  }

  if (config[name])
  {
    //! Get first point from yaml
    geometry_msgs::PoseStamped point;
    point.header.frame_id = "map";

    point.pose.position.x = config[name]["G" + std::to_string(1)]["position"]["x"].as<double>();
    point.pose.position.y = config[name]["G" + std::to_string(1)]["position"]["y"].as<double>();
    point.pose.position.z = config[name]["G" + std::to_string(1)]["position"]["z"].as<double>();

    //! Calculate distance between robot pose and first waypoint
    float dist = sqrt(pow(point.pose.position.x - current_pose.position.x, 2) +
                      pow(point.pose.position.y - current_pose.position.y, 2) +
                      pow(point.pose.position.z - current_pose.position.z, 2));

    if (dist > threshold)
    {
      return false;
    }
    else
    {
      return true;
    }
  }
  else
  {
    ROS_WARN("Invalid file. Attempted to open %s", std::string(yaml_path + name + ".yaml").c_str());
    return false;
  }
}

//! Callback for checking robot pose w.r.t. first waypoint of path
bool PathLoadSegments::onCheck(path_recall::PathCheck::Request& req, path_recall::PathCheck::Response& res)
{
  if (Check(req.name, req.distance_thresh))
  {
    res.pass = true;
  }
  else
  {
    res.pass = false;
  }
  return true;
}

//! Pause path following
void PathLoadSegments::Pause()
{
  pause = true;
  actionlib_msgs::GoalID cancel_path;
  cancel = true;
  cancel_pub_.publish(cancel_path);
}

//! Callback for pausing path loading
bool PathLoadSegments::onPause(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res)
{
  Pause();
  res.success = true;
  return true;
}

//! Resume path following
bool PathLoadSegments::Resume()
{
  if (pause && loaded_path.poses.size() != 0)
  {
    pause = false;
    cancel = false;
    publishPath();
    return true;
  }
  return false;
}

//! Callback for resuming paused path loading
bool PathLoadSegments::onResume(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res)
{
  if (Resume())
  {
    res.success = true;
  }
  else
  {
    res.success = false;
  }
  return true;
}

//! Cancel path following
void PathLoadSegments::Cancel()
{
  actionlib_msgs::GoalID cancel_path;
  cancel = true;
  current_index = 0;
  cancel_pub_.publish(cancel_path);
  std_msgs::Bool boolean;
  boolean.data = false;
  start_pub_.publish(boolean);
  start = false;
}

//! Callback for cancelling path loading
bool PathLoadSegments::onCancel(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res)
{
  Cancel();
  res.success = true;
  return true;
}

//! Alternate direct path loading service
bool PathLoadSegments::getPath(path_recall::SavePath::Request& req, path_recall::SavePath::Response& res)
{
  path_recall::PathInfo info;
  info.name = req.name;
  info.path = req.path;
  info_pub_.publish(info);
  if (loadPath(req.path))
  {
    res.success = true;
  }
  else
  {
    res.success = false;
  }
  return true;
}

//! Publish current waypoint to move_base
void PathLoadSegments::publishPath()
{
  if (!cancel)
  {
    nav_msgs::GetPlan srv;
    populateClient(srv);

    if (plan_client_.call(srv))
    {
      //! Check if plan to waypoint is viable
      if (srv.response.plan.poses.size() != 0)
      {
        path_load_pub_.publish(loaded_path.poses[current_index]);
      }

      //! If not viable, go to the next waypoint
      else if (current_index != loaded_path.poses.size() - 1 && srv.response.plan.poses.size() == 0)
      {
        while (current_index != loaded_path.poses.size() - 1)
        {
          current_index++;
          nav_msgs::GetPlan srv;
          populateClient(srv);
          if (plan_client_.call(srv))
          {
            if (srv.response.plan.poses.size() != 0)
            {
              if (calculateLength(srv.response.plan) > max_plan_length)
              {
                findShortestPath();
              }
              path_load_pub_.publish(loaded_path.poses[current_index]);
              break;
            }
          }
          else
          {
            ROS_ERROR("Failed to call service /move_base/GlobalPlanner/make_plan");
            break;
          }
        }
      }

      //! If not viable and robot is near the end of path, stop path following
      if (current_index >= loaded_path.poses.size() - 2 && srv.response.plan.poses.size() == 0)
      {
        end = true;
        start = false;
        cancel = true;
        current_index = 0;
        actionlib_msgs::GoalID cancel_path;
        cancel_pub_.publish(cancel_path);
        std_msgs::Bool boolean;
        boolean.data = false;
        start_pub_.publish(boolean);
      }
    }
    else
    {
      ROS_ERROR("Failed to call service /move_base/GlobalPlanner/make_plan");
    }
    if (current_index == loaded_path.poses.size() - 1)
    {
      end = true;
    }
  }
}

//! Callback while robot is moving
void PathLoadSegments::onFeedback(const move_base_msgs::MoveBaseActionFeedback::ConstPtr& msg)
{
  if (loaded_path.poses.size() == 0)
  {
    end = true;
    return;
  }

  if (current_index >= 2 && !start && !end)
  {
    start = true;
    std_msgs::Bool boolean;
    boolean.data = true;
    start_pub_.publish(boolean);
  }

  double linear_distance =
      sqrt(pow((msg->feedback.base_position.pose.position.x - loaded_path.poses[current_index].pose.position.x), 2) +
           pow((msg->feedback.base_position.pose.position.y - loaded_path.poses[current_index].pose.position.y), 2) +
           pow((msg->feedback.base_position.pose.position.z - loaded_path.poses[current_index].pose.position.z), 2));

  tf::Quaternion q1(msg->feedback.base_position.pose.orientation.x, msg->feedback.base_position.pose.orientation.y,
                    msg->feedback.base_position.pose.orientation.z, msg->feedback.base_position.pose.orientation.w),
      q2(loaded_path.poses[current_index].pose.orientation.x, loaded_path.poses[current_index].pose.orientation.y,
         loaded_path.poses[current_index].pose.orientation.z, loaded_path.poses[current_index].pose.orientation.w);
  tf::Matrix3x3 m1(q1), m2(q2);
  double r1, p1, y1, r2, p2, y2;
  m1.getRPY(r1, p1, y1);
  m2.getRPY(r2, p2, y2);
  double angular_difference = fabs(y1 - y2);

  //! If next waypoint distance or angular difference is over threshold, keep publishing current waypoint
  if (linear_distance > look_ahead_dist || angular_difference > look_ahead_angle)
  {
    publishPath();

    //! If waypoint distance is small, go to next waypoint
    if (linear_distance < update_min_dist && current_index != loaded_path.poses.size() - 1)
    {
      current_index++;
    }
    ros::Duration(update_time_interval).sleep();
  }

  //! Go to next waypoint if below threshold
  else
  {
    if (current_index != loaded_path.poses.size() - 1)
    {
      current_index++;
    }
  }
}

//! Populate client to call move_base service to get path plan
void PathLoadSegments::populateClient(nav_msgs::GetPlan& srv)
{
  srv.request.start.header.frame_id = "map";
  srv.request.start.pose.position.x = current_pose.position.x;
  srv.request.start.pose.position.y = current_pose.position.y;
  srv.request.start.pose.position.z = current_pose.position.z;
  srv.request.start.pose.orientation.x = current_pose.orientation.x;
  srv.request.start.pose.orientation.y = current_pose.orientation.y;
  srv.request.start.pose.orientation.z = current_pose.orientation.z;
  srv.request.start.pose.orientation.w = current_pose.orientation.w;

  srv.request.goal.header.frame_id = "map";
  srv.request.goal.pose.position.x = loaded_path.poses[current_index].pose.position.x;
  srv.request.goal.pose.position.y = loaded_path.poses[current_index].pose.position.y;
  srv.request.goal.pose.position.z = loaded_path.poses[current_index].pose.position.z;
  srv.request.goal.pose.orientation.x = loaded_path.poses[current_index].pose.orientation.x;
  srv.request.goal.pose.orientation.y = loaded_path.poses[current_index].pose.orientation.y;
  srv.request.goal.pose.orientation.z = loaded_path.poses[current_index].pose.orientation.z;
  srv.request.goal.pose.orientation.w = loaded_path.poses[current_index].pose.orientation.w;

  srv.request.tolerance = 0;
}

//! Get current pose of robot
void PathLoadSegments::getPose(const geometry_msgs::Pose::ConstPtr& msg)
{
  current_pose = *msg;
}

//! Calculate length of path plan
double PathLoadSegments::calculateLength(nav_msgs::Path plan)
{
  double length = 0;
  for (size_t i = 1; i < plan.poses.size(); i++)
  {
    length += sqrt(pow((plan.poses[i - 1].pose.position.x - plan.poses[i].pose.position.x), 2) +
                   pow((plan.poses[i - 1].pose.position.y - plan.poses[i].pose.position.y), 2) +
                   pow((plan.poses[i - 1].pose.position.z - plan.poses[i].pose.position.z), 2));
  }
  return length;
}

//! Find waypoint with the shortest path plan
void PathLoadSegments::findShortestPath()
{
  double length_of_current_path;                  //!< Path length to waypoint
  size_t index_of_shortest_path = current_index;  //!< Start from the current waypoint
  double plan_length = 0;                         //!< Moving threshold to compare path lengths
  bool strike = false;                            //!< Flag for increase in path length

  //! Iterate through remaining waypoints
  while (current_index != loaded_path.poses.size() - 1)
  {
    current_index++;
    nav_msgs::GetPlan srv;
    populateClient(srv);
    if (plan_client_.call(srv))
    {
      if (srv.response.plan.poses.size() != 0)
      {
        if (plan_length != 0)
        {
          length_of_current_path = calculateLength(srv.response.plan);

          //! If path length is lower than threshold, set the lower length as new threshold
          if (length_of_current_path < plan_length)
          {
            strike = false;
            index_of_shortest_path = current_index;
            plan_length = length_of_current_path;
          }
          else
          {
            //! If path lengths has increasing trend, exit loop
            if (strike)
            {
              break;
            }
            strike = true;
          }
        }
        else
        {
          plan_length = calculateLength(srv.response.plan);
        }
      }
    }
    else
    {
      ROS_ERROR("Failed to call service /move_base/GlobalPlanner/make_plan");
      break;
    }
  }
  current_index = index_of_shortest_path;
}

//! Callback for reaching a waypoint
void PathLoadSegments::onGoal(const move_base_msgs::MoveBaseActionResult::ConstPtr& msg)
{
  //! If reached waypoint but there is remaining waypoints, go to next waypoint
  if (!end && (msg->status.status == 3 || msg->status.status == 4))
  {
    if (current_index != loaded_path.poses.size() - 1)
    {
      current_index++;
    }
    publishPath();
  }

  //! Path loading ends once all waypoints are sent to move_base
  else if (end && (msg->status.status == 3 || msg->status.status == 4))
  {
    std_msgs::Bool boolean;
    boolean.data = false;
    start_pub_.publish(boolean);
    start = false;
  }
}
