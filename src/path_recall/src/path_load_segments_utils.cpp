#include <path_recall/path_load_segments.h>


//! Load path from YAML file
bool PathLoadSegments::loadYAML(std::string name, nav_msgs::Path &output_path) 
{
  path_name_ = name;
  nav_msgs::Path path;
  path.header.frame_id = "map";

  //! Check if file exists
  try {
    config_ = YAML::LoadFile(yaml_path_ + path_name_ + ".yaml");
  } 
  catch (YAML::BadFile e) {
    ROS_WARN("[%s] No such file for loading. Attempted to open %s", 
        name_.c_str(), (yaml_path_ + name + ".yaml").c_str());
    cancel_ = true;
    end_ = true;
    return false;
  }

  //! Process data from yaml file
  try {
    //! Check if file is empty
    if (config_[name]) {
      if (config_[name].size() == 0) {
        ROS_WARN("[%s] Not a valid path.", name_.c_str());
        return false;
      }
      //! Process data from yaml file
      for (size_t count = 1; count <= config_[name].size(); count++) {
        std::string s_count = "G" + std::to_string(count);
        geometry_msgs::PoseStamped point;
        point.header.frame_id = "map";
        point.pose.position.x = config_[name][s_count]["position"]["x"].as<double>();
        point.pose.position.y = config_[name][s_count]["position"]["y"].as<double>();
        point.pose.position.z = config_[name][s_count]["position"]["z"].as<double>();
        point.pose.orientation.x = config_[name][s_count]["orientation"]["x"].as<double>();
        point.pose.orientation.y = config_[name][s_count]["orientation"]["y"].as<double>();
        point.pose.orientation.z = config_[name][s_count]["orientation"]["z"].as<double>();
        point.pose.orientation.w = config_[name][s_count]["orientation"]["w"].as<double>();
        path.poses.push_back(point);
      }
      //output_path = path;
      path_recall::PathInfo info;
      info.name = path_name_;
      info.path = path;
      info_pub_.publish(info);
      if (loadPath(path)) {
        return true;
      } 
      else {
        return false;
      }
    } 
    else {
      ROS_WARN("[%s] Invalid file. Attempted to open %s", 
          name_.c_str(), (yaml_path_ + path_name_ + ".yaml").c_str());
      return false;
    }
  } 
  catch (YAML::BadSubscript e) {
    ROS_WARN("[%s] Empty file.", name_.c_str());
    return false;
  }
}


//! Check robot pose w.r.t. first waypoint
bool PathLoadSegments::Check(std::string name, float threshold) 
{
  try {
    config_ = YAML::LoadFile(yaml_path_ + name + ".yaml");
  } 
  catch (YAML::BadFile e) {
    ROS_WARN("[%s] No such file for loading. Attempted to open %s", 
        name_.c_str(), (yaml_path_ + name + ".yaml").c_str());
    return false;
  }
  if (config_[name]) {
    //! Get first point from yaml
    std::string s_count = "G" + std::to_string(1);
    geometry_msgs::PoseStamped point;
    point.header.frame_id = "map";
    point.pose.position.x = config_[name][s_count]["position"]["x"].as<double>();
    point.pose.position.y = config_[name][s_count]["position"]["y"].as<double>();
    point.pose.position.z = config_[name][s_count]["position"]["z"].as<double>();
    //! Calculate distance between robot pose and first waypoint
    float dist = sqrt(pow(point.pose.position.x - current_pose_.position.x, 2) +
                      pow(point.pose.position.y - current_pose_.position.y, 2) +
                      pow(point.pose.position.z - current_pose_.position.z, 2));
    // dist within threshold
    return dist <= threshold;
    // if (dist > threshold) {
    //   return false;
    // } 
    // else {
    //   return true;
    // }
  } 
  else {
    ROS_WARN("[%s] Invalid file. Attempted to open %s", 
        name_.c_str(), (yaml_path_ + name + ".yaml").c_str());
    return false;
  }
}


geometry_msgs::Pose PathLoadSegments::getNearestPseudoPoint()
{
  ROS_INFO("[%s] Requesting pseudo point", name_.c_str());
  geometry_msgs::Pose reachable_point;
  // get reachable plan
  nav_msgs::GetPlan srv;
  srv.request.start.header.frame_id = "map";
  srv.request.start.pose = current_pose_;
  srv.request.goal.header.frame_id = "map";
  srv.request.goal.pose = loaded_path_.poses[current_index_].pose;
  if(!reachable_plan_client_.call(srv)) {
    ROS_ERROR("[%s] Service call to /planner_utils/make_reachable_plan failed",name_.c_str());
  }
  else {
    if(srv.response.plan.poses.size() > 0) {
      ROS_INFO("[%s] Pseudo point found", name_.c_str());
      reachable_point = srv.response.plan.poses.back().pose;
    }
    else {
      ROS_INFO("[%s] No pseudo point found", name_.c_str());
    }
  }
  return reachable_point;
}


//! Calculate length of path plan
double PathLoadSegments::calculateLength(const geometry_msgs::Pose& init_pose, 
                                         const geometry_msgs::Pose& target_pose) 
{
  double length = sqrt(pow((init_pose.position.x - target_pose.position.x), 2) +
                       pow((init_pose.position.y - target_pose.position.y), 2));
                       //pow((init_pose.position.z - target_pose.position.z), 2));
  return length;
}


double PathLoadSegments::calculateAng(const geometry_msgs::Pose& init_pose, 
                                      const geometry_msgs::Pose& target_pose) 
{
  tf::Quaternion q1(init_pose.orientation.x, init_pose.orientation.y,
                    init_pose.orientation.z, init_pose.orientation.w);
  tf::Quaternion q2(target_pose.orientation.x, target_pose.orientation.y,
                    target_pose.orientation.z, target_pose.orientation.w);
  tf::Matrix3x3 m1(q1), m2(q2);
  double r1, p1, y1, r2, p2, y2;
  m1.getRPY(r1, p1, y1);
  m2.getRPY(r2, p2, y2);
  double dtheta = fabs(y1 - y2);
  dtheta = std::min(dtheta, 2.0*M_PI - dtheta);
  // ROS_INFO("yaw_i %5.2f, yaw_* %5.2f, dyaw %5.2f", y1, y2, dtheta);
  return dtheta;
}


void PathLoadSegments::getCostmap(nav_msgs::OccupancyGrid msg)
{
  latest_costmap_ = msg;
  have_costmap_ = true;
}


//! Populate client to call move_base service to get path plan
void PathLoadSegments::populateClient(nav_msgs::GetPlan &srv,
                                      geometry_msgs::Pose target_pose) 
{
  // ROS_INFO("prepping make plan service call, robot pose %5.2f, %5.2f, %5.2f",
  //          current_pose_.position.x, current_pose_.position.y, current_pose_.position.z);
  srv.request.tolerance = 0;
  srv.request.start.header.frame_id = "map";
  srv.request.start.pose.position.x = current_pose_.position.x;
  srv.request.start.pose.position.y = current_pose_.position.y;
  srv.request.start.pose.position.z = current_pose_.position.z;
  srv.request.start.pose.orientation.x = current_pose_.orientation.x;
  srv.request.start.pose.orientation.y = current_pose_.orientation.y;
  srv.request.start.pose.orientation.z = current_pose_.orientation.z;
  srv.request.start.pose.orientation.w = current_pose_.orientation.w;
  srv.request.goal.header.frame_id = "map";
  srv.request.goal.pose.position.x = target_pose.position.x;
  srv.request.goal.pose.position.y = target_pose.position.y;
  srv.request.goal.pose.position.z = target_pose.position.z;
  srv.request.goal.pose.orientation.x = target_pose.orientation.x;
  srv.request.goal.pose.orientation.y = target_pose.orientation.y;
  srv.request.goal.pose.orientation.z = target_pose.orientation.z;
  srv.request.goal.pose.orientation.w = target_pose.orientation.w;
}

void PathLoadSegments::publishObstructionReport(const geometry_msgs::Pose& location, bool status)
{
  movel_seirios_msgs::ObstructionStatus report_obs;
  report_obs.reporter = "path_recall";
  report_obs.status = status ? "true" : "false";
  report_obs.location = location;
  obstruction_status_pub_.publish(report_obs);
}


void PathLoadSegments::publishMoveBaseGoal(const geometry_msgs::Pose& target_pose)
{
  geometry_msgs::PoseStamped target_posestamped;
  target_posestamped.header.frame_id = "map";
  target_posestamped.pose = target_pose;
  move_base_pub_.publish(target_posestamped);
}