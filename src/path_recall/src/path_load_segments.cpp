#include "geometry_msgs/Transform.h"
#include "move_base_msgs/MoveBaseFeedback.h"
#include <bits/posix_opt.h>
#include <cstdlib>
#include <path_recall/path_load_segments.h>
#include <math.h>

PathLoadSegments::PathLoadSegments()
    : start_(false), pause_(false), cancel_(false), end_(true),
      current_index_(0), skip_on_obstruction_(false){}

//! Load path from YAML file
bool PathLoadSegments::loadYAML(std::string name, nav_msgs::Path &output_path) {
  path_name_ = name;
  nav_msgs::Path path;
  path.header.frame_id = "map";

  //! Check if file exists
  try {
    config_ = YAML::LoadFile(yaml_path_ + path_name_ + ".yaml");
  } catch (YAML::BadFile e) {
    ROS_WARN("No such file for loading. Attempted to open %s",
             std::string(yaml_path_ + name + ".yaml").c_str());
    cancel_ = true;
    end_ = true;
    return false;
  }

  //! Check if file is empty
  try {
    if (config_[name]) {
      if (config_[name].size() == 0) {
        ROS_WARN("Not a valid path.");
        return false;
      }

      //! Process data from yaml file
      for (size_t count = 1; count <= config_[name].size(); count++) {
        geometry_msgs::PoseStamped point;
        point.header.frame_id = "map";

        point.pose.position.x =
            config_[name]["G" + std::to_string(count)]["position"]["x"]
                .as<double>();
        point.pose.position.y =
            config_[name]["G" + std::to_string(count)]["position"]["y"]
                .as<double>();
        point.pose.position.z =
            config_[name]["G" + std::to_string(count)]["position"]["z"]
                .as<double>();
        point.pose.orientation.x =
            config_[name]["G" + std::to_string(count)]["orientation"]["x"]
                .as<double>();
        point.pose.orientation.y =
            config_[name]["G" + std::to_string(count)]["orientation"]["y"]
                .as<double>();
        point.pose.orientation.z =
            config_[name]["G" + std::to_string(count)]["orientation"]["z"]
                .as<double>();
        point.pose.orientation.w =
            config_[name]["G" + std::to_string(count)]["orientation"]["w"]
                .as<double>();

        path.poses.push_back(point);
      }
      //output_path = path;
      path_recall::PathInfo info;
      info.name = path_name_;
      info.path = path;
      info_pub_.publish(info);
      if (loadPath(path)) {
        return true;
      } else {
        return false;
      }
    } else {
      ROS_WARN("Invalid file. Attempted to open %s",
               std::string(yaml_path_ + path_name_ + ".yaml").c_str());
      return false;
    }
  } catch (YAML::BadSubscript e) {
    ROS_WARN("Empty file.");
    return false;
  }
}

//! Load path
bool PathLoadSegments::loadPath(nav_msgs::Path path) {
  if (path.header.frame_id == "map" && path.poses.size() > 0) {
    current_index_ = 0;
    cancel_ = false;
    end_ = false;
    loaded_path_ = path;
    display_pub_.publish(loaded_path_);
    publishPath(loaded_path_.poses[current_index_].pose);
    return true;
  } else {
    return false;
  }
}

//! Callback for path loading service
bool PathLoadSegments::onLoad(path_recall::PathName::Request &req,
                              path_recall::PathName::Response &res) {
  nav_msgs::Path path;
  if (loadYAML(req.name, path)) {
    res.success = true;
  } else {
    res.success = false;
  }
  return true;
}

//! Check robot pose w.r.t. first waypoint
bool PathLoadSegments::Check(std::string name, float threshold) {
  try {
    config_ = YAML::LoadFile(yaml_path_ + name + ".yaml");
  } catch (YAML::BadFile e) {
    ROS_WARN("No such file for loading. Attempted to open %s",
             std::string(yaml_path_ + name + ".yaml").c_str());
    return false;
  }

  if (config_[name]) {
    //! Get first point from yaml
    geometry_msgs::PoseStamped point;
    point.header.frame_id = "map";

    point.pose.position.x =
        config_[name]["G" + std::to_string(1)]["position"]["x"].as<double>();
    point.pose.position.y =
        config_[name]["G" + std::to_string(1)]["position"]["y"].as<double>();
    point.pose.position.z =
        config_[name]["G" + std::to_string(1)]["position"]["z"].as<double>();

    //! Calculate distance between robot pose and first waypoint
    float dist = sqrt(pow(point.pose.position.x - current_pose_.position.x, 2) +
                      pow(point.pose.position.y - current_pose_.position.y, 2) +
                      pow(point.pose.position.z - current_pose_.position.z, 2));

    if (dist > threshold) {
      return false;
    } else {
      return true;
    }
  } else {
    ROS_WARN("Invalid file. Attempted to open %s",
             std::string(yaml_path_ + name + ".yaml").c_str());
    return false;
  }
}

//! Callback for checking robot pose w.r.t. first waypoint of path
bool PathLoadSegments::onCheck(path_recall::PathCheck::Request &req,
                               path_recall::PathCheck::Response &res) {
  if (Check(req.name, req.distance_thresh)) {
    res.pass = true;
  } else {
    res.pass = false;
  }
  return true;
}

//! Pause path following
void PathLoadSegments::Pause() {
  pause_ = true;
  actionlib_msgs::GoalID cancel_path;
  cancel_ = true;
  cancel_pub_.publish(cancel_path);
}

//! Callback for pausing path loading
bool PathLoadSegments::onPause(std_srvs::Trigger::Request &req,
                               std_srvs::Trigger::Response &res) {
  Pause();
  res.success = true;
  return true;
}

//! Resume path following
bool PathLoadSegments::Resume() {
  if (pause_ && loaded_path_.poses.size() != 0) {
    pause_ = false;
    cancel_ = false;
    publishPath(loaded_path_.poses[current_index_].pose);
    return true;
  }
  return false;
}

//! Callback for resuming paused path loading
bool PathLoadSegments::onResume(std_srvs::Trigger::Request &req,
                                std_srvs::Trigger::Response &res) {
  if (Resume()) {
    res.success = true;
  } else {
    res.success = false;
  }
  return true;
}

//! Cancel path following
void PathLoadSegments::Cancel() {
  actionlib_msgs::GoalID cancel_path;
  cancel_ = true;
  current_index_ = 0;
  cancel_pub_.publish(cancel_path);
  std_msgs::Bool boolean;
  boolean.data = false;
  start_pub_.publish(boolean);
  start_ = false;
}

//! Callback for cancelling path loading
bool PathLoadSegments::onCancel(std_srvs::Trigger::Request &req,
                                std_srvs::Trigger::Response &res) {
  Cancel();
  res.success = true;
  return true;
}

//! Alternate direct path loading service
bool PathLoadSegments::getPath(path_recall::SavePath::Request &req,
                               path_recall::SavePath::Response &res) {
  path_recall::PathInfo info;
  info.name = req.name;
  info.path = req.path;
  info_pub_.publish(info);
  if (loadPath(req.path)) {
    res.success = true;
  } else {
    res.success = false;
  }
  return true;
}

//! Publish current waypoint to move_base
void PathLoadSegments::publishPath(geometry_msgs::Pose target_pose) {
  if (!cancel_) {
    nav_msgs::GetPlan srv;
    populateClient(srv, target_pose);

    if (plan_client_.call(srv)) {
      //! Check if plan to waypoint is viable
		
      if (srv.response.plan.poses.size() > 0) 
      {
        // ROS_INFO_STREAM("Plan to waypoint is viable, SIZE: " << srv.response.plan.poses.size());
        //for (int i = 0; i < srv.response.plan.poses.size(); i++)
        //ROS_INFO_STREAM("pose: " << srv.response.plan.poses[i]);}
        geometry_msgs::PoseStamped target_posestamped;
        target_posestamped.header.frame_id = "map";
        target_posestamped.pose = target_pose;
        path_load_pub_.publish(target_posestamped);
      }

      //! If not viable, go to the next waypoint and not last segment
      else if (current_index_ < loaded_path_.poses.size() - 1 &&
               skip_on_obstruction_) {
        current_index_++;
        publishPath(loaded_path_.poses[current_index_].pose);
      }
      //! go near to obstacle and not last segment
      else if (current_index_ < loaded_path_.poses.size() - 1 &&
               skip_on_obstruction_ == false) {
        geometry_msgs::Pose pseudo_point = getNearestPseudoPoint();
        geometry_msgs::PoseStamped target_posestamped;
        target_posestamped.header.frame_id = "map";
        target_posestamped.pose = pseudo_point;
        path_load_pub_.publish(target_posestamped);
        Pause();
        ROS_INFO_STREAM("gotten nearest pseudo point, published"<< pseudo_point);
        //publishPath(pseudo_point);
      }
      //! If not viable and robot is at last segment, stop path following
      if (current_index_ >= loaded_path_.poses.size() - 1 &&
          srv.response.plan.poses.size() == 0) {
        end_ = true;
        start_ = false;
        cancel_ = true;
        current_index_ = 0;
        actionlib_msgs::GoalID cancel_path;
        cancel_pub_.publish(cancel_path);
        std_msgs::Bool boolean;
        boolean.data = false;
        start_pub_.publish(boolean);
      }
    } else {
      ROS_ERROR("Failed to call service /move_base/GlobalPlanner/make_plan");
    }
  }
  if (current_index_ == loaded_path_.poses.size() - 1) {
    end_ = true;
  }
}

geometry_msgs::Pose PathLoadSegments::getNearestPseudoPoint() {
  ROS_INFO("Calculating Pseudo point");
  bool nearest = false;
  bool found_viable = false;
  /*double dist = calculateLength(loaded_path_.poses[current_index_ - 1].pose,
                                loaded_path_.poses[current_index_].pose) /
                2;
 double ang = calculateAng(loaded_path_.poses[current_index_ - 1].pose,
                            loaded_path_.poses[current_index_].pose);
  //double dist_diff = dist;*/
  geometry_msgs::Pose estimated_nearby;
  geometry_msgs::Pose estimate_viable = loaded_path_.poses[current_index_ -1].pose;
  double alpha = 0.5;
  while (!nearest || !found_viable) {
    estimated_nearby.position.x =
        loaded_path_.poses[current_index_ - 1].pose.position.x + alpha*(loaded_path_.poses[current_index_].pose.position.x-loaded_path_.poses[current_index_ - 1].pose.position.x);
    estimated_nearby.position.y =
        loaded_path_.poses[current_index_ - 1].pose.position.y + alpha*(loaded_path_.poses[current_index_].pose.position.y-loaded_path_.poses[current_index_ - 1].pose.position.y);
    estimated_nearby.orientation.w = 1;
    /*estimated_nearby.position.z =
        loaded_path_.poses[current_index_ - 1].pose.position.z +
        cos(ang) * dist;
    */
    nav_msgs::GetPlan srv;
    populateClient(srv, estimated_nearby);
    try {
      plan_client_.call(srv);
      //! Check if plan to waypoint is viable
      if (srv.response.plan.poses.size() > 0) {
        if (alpha <= 1.0)
        {alpha = 1.2*alpha;
        found_viable = true;
        estimate_viable = estimated_nearby;}
        else
        {nearest = true;
         break;}}
        //dist += dist_diff / 2;
      else {
        if (!found_viable)
         {if (alpha > 0.08)
        {alpha = 0.8*alpha;}
          else{nearest = true; found_viable = true; break;}
        }
       else
        {nearest = true;
         break;}
        //dist -= dist_diff / 2;
      }
      /*if (dist_diff < update_min_dist_)
        nearest = true;
      break;*/
    } catch (...) {
      ROS_ERROR("Failed to call service /move_base/GlobalPlanner/make_plan");
    }
  }
  return estimate_viable;
}

//! Callback while robot is moving
void PathLoadSegments::onFeedback(
    const move_base_msgs::MoveBaseActionFeedback::ConstPtr &msg) {
  if (loaded_path_.poses.size() == 0) {
    end_ = true;
    return;
  }

  if (current_index_ >= 2 && !start_ && !end_) {
    start_ = true;
    std_msgs::Bool boolean;
    boolean.data = true;
    start_pub_.publish(boolean);
  }

  // ROS_INFO("Current idx: %d", current_index_);
  if (current_index_ == loaded_path_.poses.size() -1 )
  {return;}

  double linear_distance =
      calculateLength(msg->feedback.base_position.pose,
                      loaded_path_.poses[current_index_].pose);
  double angular_difference =
      calculateAng(msg->feedback.base_position.pose,loaded_path_.poses[current_index_].pose);
  //! Go to next waypoint if below threshold
  if (linear_distance < update_min_dist_ &&
      angular_difference < look_ahead_angle_) {
    ROS_INFO_STREAM("Increasing index from feedback: " << current_index_);
    current_index_++;
    }
  
  //! If next waypoint distance or angular difference is over threshold, keep
  //! publishing current waypoint
  //else if (current_index_ != loaded_path_.poses.size() - 1) {
    publishPath(loaded_path_.poses[current_index_].pose);
    //! If waypoint distance is small, go to next waypoint
    ros::Duration(update_time_interval_).sleep();
  }

//! Populate client to call move_base service to get path plan
void PathLoadSegments::populateClient(nav_msgs::GetPlan &srv,
                                      geometry_msgs::Pose target_pose) {
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

  srv.request.tolerance = 0;
}

//! Get current pose of robot
void PathLoadSegments::getPose(const geometry_msgs::Pose::ConstPtr &msg) {
  current_pose_ = *msg;
}

//! Calculate length of path plan
double PathLoadSegments::calculateLength(geometry_msgs::Pose init_pose,
                                         geometry_msgs::Pose target_pose) {
  double length = sqrt(pow((init_pose.position.x - target_pose.position.x), 2) +
                       pow((init_pose.position.y - target_pose.position.y), 2));
                       //pow((init_pose.position.z - target_pose.position.z), 2));
  return length;
}

double PathLoadSegments::calculateAng(geometry_msgs::Pose init_pose,
                                      geometry_msgs::Pose target_pose) {
  tf::Quaternion q1(init_pose.orientation.x, init_pose.orientation.y,
                    init_pose.orientation.z, init_pose.orientation.w),
      q2(target_pose.orientation.x, target_pose.orientation.y,
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

/*
//! Find waypoint with the shortest path plan
void PathLoadSegments::findShortestPath() {
  double length_of_current_path; //!< Path length to waypoint
  size_t index_of_shortest_path =
      current_index_;     //!< Start from the current waypoint
  double plan_length = 0; //!< Moving threshold to compare path lengths
  bool strike = false;    //!< Flag for increase in path length

  //! Iterate through remaining waypoints
  while (current_index_ != loaded_path_.poses.size() - 1) {
    current_index_++;
    nav_msgs::GetPlan srv;
    populateClient(srv);
    if (plan_client_.call(srv)) {
      if (srv.response.plan.poses.size() != 0) {
        if (plan_length != 0) {
          length_of_current_path = calculateLength(srv.response.plan);

          //! If path length is lower than threshold, set the lower length as
          //! new threshold
          if (length_of_current_path < plan_length) {
            strike = false;
            index_of_shortest_path = current_index_;
            plan_length = length_of_current_path;
          } else {
            //! If path lengths has increasing trend, exit loop
            if (strike) {
              break;
            }
            strike = true;
          }
        } else {
          plan_length = calculateLength(srv.response.plan);
        }
      }
    } else {
      ROS_ERROR("Failed to call service /move_base/GlobalPlanner/make_plan");
      break;
    }
  }
  current_index_ = index_of_shortest_path;
}
*/

//! Callback for reaching a waypoint
void PathLoadSegments::onGoal(
    const move_base_msgs::MoveBaseActionResult::ConstPtr &msg) {
  //! If reached waypoint but there is remaining waypoints, go to next
  //! waypoint
  /*if (!end_ && (msg->status.status == 3 || msg->status.status == 4)) {
    ROS_INFO_STREAM("goal status: " << msg->status.status);
    ROS_INFO_STREAM("current_index: " << current_index_);
    //if (current_index_ != loaded_path_.poses.size() - 1) {
    //  current_index_++;
    //}
    //ROS_INFO_STREAM("Increased index to: " << current_index_);
    publishPath(loaded_path_.poses[current_index_].pose);
  }
  */
  //! Path loading ends once all waypoints are sent to move_base
  if (end_ && (msg->status.status == 3 || msg->status.status == 4)) {
    std_msgs::Bool boolean;
    boolean.data = false;
    start_pub_.publish(boolean);
    start_ = false;
  }
}
