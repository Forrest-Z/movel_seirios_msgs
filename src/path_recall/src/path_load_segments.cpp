#include "geometry_msgs/Transform.h"
#include "move_base_msgs/MoveBaseFeedback.h"
#include <bits/posix_opt.h>
#include <cstdlib>
#include <path_recall/path_load_segments.h>
#include <math.h>

PathLoadSegments::PathLoadSegments()
    : start_(false), pause_(false), obstructed_(false), cancel_(false), end_(true)
      , have_pose_(false), current_index_(0), skip_on_obstruction_(false), have_costmap_(false)
      , waiting_for_obstacle_clearance_(false){}

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
    publishPath(loaded_path_.poses[current_index_].pose, true);
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
  // ROS_INFO("Pause %d", pause_);
  actionlib_msgs::GoalID cancel_path;
  cancel_ = true;
  // cancel_pub_.publish(cancel_path);
  cancel_pub_.publish(move_base_goal_id_);
  cancel_pub_.publish(cancel_path);
  ROS_INFO("pausing, cancelling move_base goal");
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
    // ROS_INFO("resume %d", pause_);
    cancel_ = false;
    publishPath(loaded_path_.poses[current_index_].pose, true);
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
void PathLoadSegments::publishPath(geometry_msgs::Pose target_pose, bool execute) 
{
  if (!cancel_) {
    ros::Time t0 = ros::Time::now();
    ros::Time t1;
    double dt;
    while (!have_pose_ || !have_costmap_)
    {
      ros::Duration(0.1).sleep();
      t1 = ros::Time::now();
      dt = (t1 - t0).toSec();
      ROS_INFO("wait for pose, %5.2f", dt);
      ros::spinOnce();

      if (dt > 10.0)
      {
        ROS_INFO("10 [s] is too long. Giving up");
        return;
      }
    }

    if (!checkObstruction(loaded_path_.poses[current_index_]))
    {
      nav_msgs::GetPlan srv;
      populateClient(srv, target_pose);

      if (plan_client_.call(srv)) {
        //! Check if plan to waypoint is viable
        // ROS_INFO("plan length %lu", srv.response.plan.poses.size());
        if (srv.response.plan.poses.size() > 0)
        {
          // ROS_INFO_STREAM("Plan to waypoint is viable, SIZE: " << srv.response.plan.poses.size());
          //for (int i = 0; i < srv.response.plan.poses.size(); i++)
          //ROS_INFO_STREAM("pose: " << srv.response.plan.poses[i]);}
          if (execute)
          {
            // we know this path isn't obstructed, 
            // and there is possibility that obstacle_extractor is still in active
            // so make it certain that it's clear
            movel_seirios_msgs::ObstructionStatus report_obs;
            report_obs.reporter = "path_recall";
            report_obs.status = "false";
            report_obs.location = target_pose;
            obstruction_status_pub_.publish(report_obs);

            geometry_msgs::PoseStamped target_posestamped;
            target_posestamped.header.frame_id = "map";
            target_posestamped.pose = target_pose;
            path_load_pub_.publish(target_posestamped);
          }
        }

        //! If not viable, go to the next waypoint and not last segment
        else if (current_index_ < loaded_path_.poses.size() - 1 &&
                 skip_on_obstruction_) {
          current_index_++;
          publishPath(loaded_path_.poses[current_index_].pose, true);
        }
        //! go near to obstacle and not last segment
        else if (current_index_ <= loaded_path_.poses.size() &&
                 skip_on_obstruction_ == false) {
          ROS_INFO("waypoint obstructed, pausing");
          Pause();

          // Obstruction Status reporting
          movel_seirios_msgs::ObstructionStatus report_obs;
          report_obs.reporter = "path_recall";
          report_obs.status = "true";
          report_obs.location = target_pose;
          obstruction_status_pub_.publish(report_obs);

          geometry_msgs::Pose pseudo_point = getNearestPseudoPoint();
          geometry_msgs::PoseStamped target_posestamped;
          target_posestamped.header.frame_id = "map";
          target_posestamped.pose = pseudo_point;
          path_load_pub_.publish(target_posestamped);
          obstructed_ = true;

          ROS_INFO_STREAM("got nearest pseudo point, published:\n"<< pseudo_point);
          //publishPath(pseudo_point);
        }
        //! If not viable and robot is at last segment, stop path following
        if (current_index_ >= loaded_path_.poses.size() - 1 &&
            srv.response.plan.poses.size() == 0 && skip_on_obstruction_ == true) {

          // Obstruction Status reporting
          movel_seirios_msgs::ObstructionStatus report_obs;
          report_obs.reporter = "path_recall";
          report_obs.status = "true";
          report_obs.location = target_pose;
          obstruction_status_pub_.publish(report_obs);

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
    else
    {
      //! If not viable, go to the next waypoint and not last segment
      if (current_index_ < loaded_path_.poses.size() - 1 &&
               skip_on_obstruction_) {
        current_index_++;
        publishPath(loaded_path_.poses[current_index_].pose, true);
      }
      //! go near to obstacle and not last segment
      else if (current_index_ <= loaded_path_.poses.size() &&
               skip_on_obstruction_ == false) {
        ROS_INFO("waypoint obstructed, pausing");
        Pause();

         // Obstruction Status reporting
        movel_seirios_msgs::ObstructionStatus report_obs;
        report_obs.reporter = "path_recall";
        report_obs.status = "true";
        report_obs.location = target_pose;
        obstruction_status_pub_.publish(report_obs);

        geometry_msgs::Pose pseudo_point = getNearestPseudoPoint();
        geometry_msgs::PoseStamped target_posestamped;
        target_posestamped.header.frame_id = "map";
        target_posestamped.pose = pseudo_point;
        path_load_pub_.publish(target_posestamped);
        obstructed_ = true;

        ROS_INFO_STREAM("got nearest pseudo point, published:\n"<< pseudo_point);
        //publishPath(pseudo_point);
      }
      //! If not viable and robot is at last segment, stop path following
      if (current_index_ >= loaded_path_.poses.size() - 1 && skip_on_obstruction_ == true) {
        // Obstruction Status reporting
        movel_seirios_msgs::ObstructionStatus report_obs;
        report_obs.reporter = "path_recall";
        report_obs.status = "true";
        report_obs.location = target_pose;
        obstruction_status_pub_.publish(report_obs);

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
  double alpha_step = 0.25;
  double dx, dy;
  // dx = loaded_path_.poses[current_index_].pose.position.x-loaded_path_.poses[current_index_ - 1].pose.position.x;
  // dy = loaded_path_.poses[current_index_].pose.position.y-loaded_path_.poses[current_index_ - 1].pose.position.y;
  dx = loaded_path_.poses[current_index_].pose.position.x - current_pose_.position.x;
  dy = loaded_path_.poses[current_index_].pose.position.y - current_pose_.position.y;
  double yaw = atan2(dy, dx);
  estimated_nearby.orientation.w = cos(0.5*yaw);
  estimated_nearby.orientation.z = sin(0.5*yaw);
  int N = 0; //iteration count
  while (!nearest || !found_viable) 
  {
    // estimated_nearby.position.x = loaded_path_.poses[current_index_ - 1].pose.position.x + alpha*dx;
    // estimated_nearby.position.y = loaded_path_.poses[current_index_ - 1].pose.position.y + alpha*dy;
    estimated_nearby.position.x = current_pose_.position.x + alpha*dx;
    estimated_nearby.position.y = current_pose_.position.y + alpha*dy;

    geometry_msgs::PoseStamped estimated_nearby_stamped;
    estimated_nearby_stamped.header.frame_id = "map";
    estimated_nearby_stamped.pose = estimated_nearby;
    if (!checkObstruction(estimated_nearby_stamped))
    {
      nav_msgs::GetPlan srv;
      populateClient(srv, estimated_nearby);
      try {
        plan_client_.call(srv);
        // ROS_INFO("iter %d, plan size %lu", N, srv.response.plan.poses.size());
        //! Check if plan to waypoint is viable
        if (srv.response.plan.poses.size() > 0)
        {
          if (N < 3)
          {
            // alpha = 1.5*alpha;
            alpha += alpha_step;
            alpha_step = 0.5*alpha_step;
            N += 1;
            found_viable = true;
            estimate_viable = estimated_nearby;}
          else
          {
            nearest = true;
            ROS_INFO("found nearest after %d iter, at alpha %5.2f", N, alpha);
            break;
          }
        }
        else
        {
          if (!found_viable)
          {
            if (N < 3)
            {
              // alpha = 0.5*alpha;
              alpha -= alpha_step;
              alpha_step = 0.5*alpha_step;
              N += 1;
            }
            else
            {
              nearest = true;
              found_viable = true;
              ROS_INFO("found nearest after %d iter, at alpha %5.2f", N, alpha);
              break;
            }
          }
          else
          {
            nearest = true;
            ROS_INFO("found nearest after %d iter, at alpha %5.2f", N, alpha);
            break;
          }
        }
      }
      catch (...)
      {
        ROS_ERROR("Failed to call service /move_base/GlobalPlanner/make_plan");
      }
    }
    else
    {
      if (!found_viable)
      {
        if (N < 3)
        {
          // alpha = 0.5*alpha;
          alpha -= alpha_step;
          alpha_step = 0.5*alpha_step;
          N += 1;
        }
        else
        {
          nearest = true;
          found_viable = true;
          ROS_INFO("found nearest after %d iter, at alpha %5.2f", N, alpha);
          break;
        }
      }
      else
      {
        nearest = true;
        ROS_INFO("found nearest after %d iter, at alpha %5.2f", N, alpha);
        break;
      }
    }
  }
  return estimate_viable;
}

//! Callback while robot is moving
void PathLoadSegments::onFeedback(const move_base_msgs::MoveBaseActionFeedback::ConstPtr &msg) 
{
  move_base_goal_id_ = msg->status.goal_id;
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
      calculateLength(msg->feedback.base_position.pose, loaded_path_.poses[current_index_].pose);
  double angular_difference =
      calculateAng(msg->feedback.base_position.pose,loaded_path_.poses[current_index_].pose);
  
  //! Go to next waypoint if below threshold
  if (linear_distance < update_min_dist_ && angular_difference < look_ahead_angle_) 
  {
    ROS_INFO_STREAM("Increasing index from feedback: " << current_index_);
    current_index_++;
    publishPath(loaded_path_.poses[current_index_].pose, true);
  }
  
  if (current_index_ != 0)
  {
    double go_linear = calculateLength(current_pose_, loaded_path_.poses[current_index_-1].pose);

    if  (go_linear < mb_xy_tolerance_)
    {
      publishPath(loaded_path_.poses[current_index_].pose, true);
    }
    else
    {
      publishPath(loaded_path_.poses[current_index_].pose, false);
    }
  }
  
  //! If next waypoint distance or angular difference is over threshold, keep
  //! publishing current waypoint
  //else if (current_index_ != loaded_path_.poses.size() - 1) {

  //publishPath(loaded_path_.poses[current_index_].pose);
    //! If waypoint distance is small, go to next waypoint
  ros::Duration(update_time_interval_).sleep();
}

//! Populate client to call move_base service to get path plan
void PathLoadSegments::populateClient(nav_msgs::GetPlan &srv,
                                      geometry_msgs::Pose target_pose) {
  // ROS_INFO("prepping make plan service call, robot pose %5.2f, %5.2f, %5.2f",
  //          current_pose_.position.x, current_pose_.position.y, current_pose_.position.z);

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
  have_pose_ = true;
  
  if (obstructed_)
  {
    if (!checkObstruction(loaded_path_.poses[current_index_]))
    {
      nav_msgs::GetPlan srv;
      populateClient(srv, loaded_path_.poses[current_index_].pose);
      plan_client_.call(srv);
      if (srv.response.plan.poses.size() > 0)
      {
        ROS_INFO("Path to waypoint is clear, resuming");
      
        // Obstruction Status reporting
        movel_seirios_msgs::ObstructionStatus report_obs;
        report_obs.reporter = "path_recall";
        report_obs.status = "false";
        report_obs.location = current_pose_;
        obstruction_status_pub_.publish(report_obs);

        obstructed_ = false;
        waiting_for_obstacle_clearance_ = false;
        Resume();
      }
      else if (waiting_for_obstacle_clearance_ && ros::Time::now().toSec() - pause_start_time_.toSec() > clearing_timeout_ && clearing_timeout_ > 0)
      {
        ROS_INFO("Waiting for obstacle clearance exceeded timeout, skipping waypoint");
        current_index_++;
        obstructed_ = false;
        waiting_for_obstacle_clearance_ = false;
        Resume();
      }
    }
    else if (waiting_for_obstacle_clearance_ && ros::Time::now().toSec() - pause_start_time_.toSec() > clearing_timeout_ && clearing_timeout_ > 0)
    {
      ROS_INFO("Waiting for obstacle clearance exceeded timeout, skipping waypoint");
      current_index_++;
      obstructed_ = false;
      waiting_for_obstacle_clearance_ = false;
      Resume();
    }
  }
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
void PathLoadSegments::onGoal(const move_base_msgs::MoveBaseActionResult::ConstPtr &msg) 
{
  // in case path_load sticks around after completion for delayed teardown,
  // a new move_base goal (e.g. from nav task) must not re-send the final waypoint
  // of the previously successful path
  if (!start_ || pause_)
  {
    ROS_INFO("Got move_base success, but path load isn't active");
    return;
  }

  if (obstructed_ && (msg->status.status == 3 || msg->status.status == 4))
  {
  //   ROS_INFO("attempting clear");
  //   std_srvs::Empty emp;
  //   clear_costmaps_client_.call(emp);
    ROS_INFO("Completed pseudo goal, waiting for obstacle clearance");
    pause_start_time_ = ros::Time::now();
    waiting_for_obstacle_clearance_ = true;
  }
  //! If reached waypoint but there is remaining waypoints, go to next
  //! waypoint
  if (!end_ && (msg->status.status == 3 || msg->status.status == 4)) 
  {
    // ROS_INFO_STREAM("goal status: " << msg->status.status);
    // ROS_INFO_STREAM("current_index: " << current_index_);

    // validate distance to current waypoint
    double dxy = calculateLength(current_pose_, loaded_path_.poses[current_index_].pose);
    double dyaw = calculateAng(current_pose_, loaded_path_.poses[current_index_].pose);

    if (dxy <= mb_xy_tolerance_ && dyaw <= mb_yaw_tolerance_ && current_index_ != loaded_path_.poses.size() - 1) 
    {
      ROS_INFO_STREAM("move_base goal reached and distances checked out, bumping index to: " << current_index_);
      current_index_++;
      //publishPath(loaded_path_.poses[current_index_].pose);
    }
    else
    {
      ROS_INFO("Got move_base success, but distances don't check out");
      ROS_INFO("linear %5.2f out of %5.2f", dxy, mb_xy_tolerance_);
      ROS_INFO("angular %5.2f of %5.2f", dyaw, mb_yaw_tolerance_);
    }
    publishPath(loaded_path_.poses[current_index_].pose, true);
  }
  
  //! Path loading ends once all waypoints are sent to move_base
  if (end_ && (msg->status.status == 3 || msg->status.status == 4)) {
    // validate distance to current waypoint
    double dxy = calculateLength(current_pose_, loaded_path_.poses[current_index_].pose);
    double dyaw = calculateAng(current_pose_, loaded_path_.poses[current_index_].pose);

    if (dxy <= mb_xy_tolerance_ && dyaw <= mb_yaw_tolerance_ )
    {
        std_msgs::Bool boolean;
        boolean.data = false;
        start_pub_.publish(boolean);
        start_ = false;  
    }
    else
    {
      ROS_INFO("Got move_base success, but distances don't check out (last index)");
      ROS_INFO("linear %5.2f out of %5.2f", dxy, mb_xy_tolerance_);
      ROS_INFO("angular %5.2f of %5.2f", dyaw, mb_yaw_tolerance_);
      publishPath(loaded_path_.poses[current_index_].pose, true);
    }
  }
}

void PathLoadSegments::getCostmap(nav_msgs::OccupancyGrid msg)
{
  latest_costmap_ = msg;
  have_costmap_ = true;
}

bool PathLoadSegments::checkObstruction(geometry_msgs::PoseStamped goal)
{
  if (!have_costmap_)
    return false;

  int max_occupancy = 0;

  geometry_msgs::PoseStamped pose_costmap, pose_costmap_local;

  // transform plan pose to costmap frame
  geometry_msgs::TransformStamped transform =
    tf_buffer_->lookupTransform(latest_costmap_.header.frame_id,
                               goal.header.frame_id, ros::Time(0));

  tf2::doTransform(goal.pose, pose_costmap.pose, transform);

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

  // update maximum occupancy
  if (row < latest_costmap_.info.height && col < latest_costmap_.info.width
      && row >= 0 && col >= 0)
  {
    int idx = row * latest_costmap_.info.width + col;
    int occupancy = latest_costmap_.data[idx];
    if (occupancy > max_occupancy)
    {
      max_occupancy = occupancy;
    }
  }

  if (max_occupancy >= obstruction_threshold_)
  {
    return true;
  }

  return false;
}
