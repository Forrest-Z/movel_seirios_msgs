#include "geometry_msgs/Transform.h"
#include "move_base_msgs/MoveBaseFeedback.h"
#include <bits/posix_opt.h>
#include <cstdlib>
#include <path_recall/path_load_segments.h>
#include <math.h>


PathLoadSegments::PathLoadSegments()
  : start_(false), pause_(false), obstructed_(false), cancel_(false), final_end_point_fail_(false),
    end_(true), have_pose_(false), current_index_(0), skip_on_obstruction_(false), 
    have_costmap_(false), waiting_for_obstacle_clearance_(false), ts_pause_status_(false)
{
}


//! Load path
bool PathLoadSegments::loadPath(nav_msgs::Path path) {
  name_ = "path_load";
  // ROS_INFO("reset ping counter, load path");
  ping_counter_ = 0;

  if (path.header.frame_id == "map" && path.poses.size() > 0) {
    current_index_ = 0;
    cancel_ = false;
    end_ = false;
    final_end_point_fail_ = false;
    loaded_path_ = path;
    display_pub_.publish(loaded_path_);
    start_ = true;
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
  ROS_INFO("[%s] pausing, cancelling move_base goal",name_.c_str());
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
  obstructed_ = false;
  // cancel_pub_.publish(move_base_goal_id_);
  cancel_pub_.publish(cancel_path);
  std_msgs::Bool boolean;
  std_msgs::Bool fail_bool_;
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
      ROS_INFO("[%s] wait for pose, %5.2f", name_.c_str(), dt);
      ros::spinOnce();

      if (dt > 10.0) {
        ROS_INFO("[%s] 10 [s] is too long. Giving up", name_.c_str());
        return;
      }
    }
    // ROS_INFO("[%s] publishPath idx: %d, execute %d", name_.c_str(), current_index_, execute);
    // waypoint not obstructed
    if (!checkObstruction(loaded_path_.poses[current_index_]))
    {
      nav_msgs::GetPlan srv;
      populateClient(srv, target_pose);
      // ROS_INFO("[%s] No obstacle . skip_on_obstruction_ : %d", name_.c_str(), skip_on_obstruction_);
      if (plan_client_.call(srv)) {
        // ROS_INFO("[%s] Path is available ", name_.c_str());
        //! Check if plan to waypoint is viable
        // ROS_INFO("plan length %lu", srv.response.plan.poses.size());
        if (srv.response.plan.poses.size() > 0) {
          // ROS_INFO("reset ping counter, viable plan found");
          ping_counter_ = 0;
          // ROS_INFO_STREAM("Plan to waypoint is viable, SIZE: " << srv.response.plan.poses.size());
          //for (int i = 0; i < srv.response.plan.poses.size(); i++)
          //ROS_INFO_STREAM("pose: " << srv.response.plan.poses[i]);}
          if (execute) {
            // we know this path isn't obstructed, 
            // and there is possibility that obstacle_extractor is still in active
            // so make it certain that it's clear
            publishObstructionReport(target_pose, false);
            publishMoveBaseGoal(target_pose);
          }
        }
        //! If not viable, go to the next waypoint and not last segment
        // else if (current_index_ < loaded_path_.poses.size() - 1 &&
        //          skip_on_obstruction_) 
        // {
        //   //pinging waypoint
        //   if(ping_counter_ < max_ping_count_){
        //     ping_counter_++;
        //     ROS_INFO("[%s] 1.Pinging ......[%d/%d] ", name_.c_str(), ping_counter_, max_ping_count_);
        //     publishMoveBaseGoal(target_pose);
        //   }
        //   else{
        //     // ROS_INFO("reset ping counter, max ping reached");
        //     ping_counter_ = 0;
        //     current_index_++;
        //     ROS_INFO("[%s] waypoint %lu is not obstructed, but no viable path to waypoint. Skipping to %lu",
        //              name_.c_str(), current_index_-1, current_index_);
        //     publishPath(loaded_path_.poses[current_index_].pose, true);
        //   } 
        // }
        //! go near to obstacle and not last segment
        else if (current_index_ <= loaded_path_.poses.size()-1) 
        {
          ROS_INFO("[%s] waypoint obstructed, pausing", name_.c_str());
          Pause();
          ros::Time t_wait = ros::Time::now();

          if(!best_effort_enabled_)
          {
            obstructed_ = true;
            waiting_for_obstacle_clearance_ = true;
            pause_start_time_ = ros::Time::now();
            if (current_index_ == loaded_path_.poses.size() - 1) {
              end_ = true;
            }
            return;
          }
           
          while (true) {
            if (ts_pause_status_ == true) { break; }
            double dt = (ros::Time::now() - t_wait).toSec();
            if (dt > 5.) { break; }
            ros::spinOnce();
          }
          // Obstruction Status reporting
          publishObstructionReport(target_pose, true);
          // pseudo point
          geometry_msgs::Pose pseudo_point = getNearestPseudoPoint();
          publishMoveBaseGoal(pseudo_point);
          obstructed_ = true;
          ROS_INFO_STREAM("[" << name_ << "] 1. got nearest pseudo point, published:\n" << pseudo_point);
          //publishPath(pseudo_point);
        }
        
        //! If not viable and robot is at last segment, stop path following
        // else if (current_index_ >= loaded_path_.poses.size() - 1 &&
        //          srv.response.plan.poses.size() == 0 &&
        //          skip_on_obstruction_ == true) 
        // {
        //   // Obstruction Status reporting
        //   publishObstructionReport(target_pose, true);
        //   end_ = true;
        //   start_ = false;
        //   cancel_ = true;
        //   current_index_ = 0;
        //   actionlib_msgs::GoalID cancel_path;
        //   cancel_pub_.publish(cancel_path);
        //   std_msgs::Bool boolean;
        //   std_msgs::Bool fail_;
        //   boolean.data = false;
        //   fail_.data = true;
        //   ROS_INFO("[%s] completion, final waypoint not obstructed, but not viable",name_.c_str());
        //   fail_pub_.publish(fail_);
        //   start_pub_.publish(boolean);
        // }
      } 
      else {
        ROS_ERROR("[%s] Failed to call service /move_base/GlobalPlanner/make_plan", name_.c_str());
      }
    }
    // waypoint obstructed
    else
    {
      //! If not viable, go to the next waypoint and not last segment
      // if (current_index_ <= loaded_path_.poses.size() - 1 && skip_on_obstruction_) 
      // {
      //   if(ping_counter_ < max_ping_count_)
      //   {
      //     ping_counter_++;
      //     ROS_INFO("[%s] 2.Pinging ...... [%d/%d]", name_.c_str(), ping_counter_, max_ping_count_);
      //     publishMoveBaseGoal(target_pose);
      //   }
      //   else
      //   {
      //     // ROS_INFO("reset ping counter, max ping reached");
      //     ping_counter_ = 0;
      //     current_index_++;
      //     // past final index, declare completion
      //     if (current_index_ >= loaded_path_.poses.size()-1)
      //     {
      //       publishObstructionReport(target_pose, true);
      //       end_ = true;
      //       start_ = false;
      //       cancel_ = true;
      //       current_index_ = 0;
      //       actionlib_msgs::GoalID cancel_path;
      //       cancel_pub_.publish(cancel_path);
      //       std_msgs::Bool boolean;
      //       std_msgs::Bool fail_;
      //       boolean.data = false;
      //       fail_.data = true;
      //       ROS_INFO("completion, final waypoint obstructed, skip is true",name_.c_str());
      //       fail_pub_.publish(fail_);
      //       start_pub_.publish(boolean);
      //       return;
      //     }
      //     // regular skip when intermediate index
      //     ROS_INFO("[%s] waypoint %lu is obstructed. Skipping to %lu",name_.c_str(), current_index_-1, current_index_);
      //     publishPath(loaded_path_.poses[current_index_].pose, true);
      //   }
      // }
      //! go near to obstacle and not last segment
      if (current_index_ <= loaded_path_.poses.size()) {
        ROS_INFO("[%s] waypoint obstructed, pausing", name_.c_str());
        Pause();
        ros::Time t_wait = ros::Time::now();
        
        if(!best_effort_enabled_)
        {
          obstructed_ = true;
          waiting_for_obstacle_clearance_ = true;
          pause_start_time_ = ros::Time::now();
          if (current_index_ == loaded_path_.poses.size() - 1) {
            end_ = true;
          }
          return;
        }
        
        while (true)
        {
          if (ts_pause_status_ == true)
            break;
          double dt = (ros::Time::now() - t_wait).toSec();
          if (dt > 5.)
            break;
          ros::spinOnce();
        }
        // Obstruction Status reporting
        publishObstructionReport(target_pose, true);
        // pseudo point
        geometry_msgs::Pose pseudo_point = getNearestPseudoPoint();
        publishMoveBaseGoal(pseudo_point);
        obstructed_ = true;
        ROS_INFO_STREAM("[" << name_ << "] 2. got nearest pseudo point, published:\n" << pseudo_point);
        //publishPath(pseudo_point);
      }
    }
  }
  if (current_index_ == loaded_path_.poses.size() - 1) {
    end_ = true;
  }
}


//! Callback while robot is moving
void PathLoadSegments::onFeedback(const move_base_msgs::MoveBaseActionFeedback::ConstPtr &msg) 
{
  // ROS_INFO("[%s] on move base feedback", name_.c_str());
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
  if (current_index_ >= loaded_path_.poses.size()) {
    return;
  }
  
  geometry_msgs::Pose current_pose = msg->feedback.base_position.pose;
  geometry_msgs::Pose waypoint = loaded_path_.poses[current_index_].pose;
  
  double linear_distance = calculateLength(current_pose, waypoint);
  double angular_difference = calculateAng(current_pose, waypoint);
  
  //! Go to next waypoint if below threshold
  if (linear_distance < update_min_dist_ && angular_difference < look_ahead_angle_) {
    // ROS_INFO("reset ping counter, increment index from feedback");
    ping_counter_ = 0;
    current_index_++;
    if (current_index_ >= loaded_path_.poses.size())
      return;
    ROS_INFO_STREAM("[" << name_ << "] Increasing index from feedback: " << current_index_);
    publishPath(loaded_path_.poses[current_index_].pose, true);
  }
  
  if (current_index_ != 0) {
    double go_linear = calculateLength(current_pose_, loaded_path_.poses[current_index_-1].pose);
    if (go_linear < mb_xy_tolerance_) {
      publishPath(loaded_path_.poses[current_index_].pose, true);
    }
    else {
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


//! Get current pose of robot
void PathLoadSegments::getPose(const geometry_msgs::Pose::ConstPtr &msg) 
{
  current_pose_ = *msg;
  have_pose_ = true;
  
  if (obstructed_ and !final_end_point_fail_) {
    // check if waypoint is clear
    bool waypoint_clear = false;
    if (!checkObstruction(loaded_path_.poses[current_index_])) {
      nav_msgs::GetPlan srv;
      populateClient(srv, loaded_path_.poses[current_index_].pose);
      plan_client_.call(srv);
      if (srv.response.plan.poses.size() > 0) { waypoint_clear = true; }
    }
    // waypoint clear, resume
    if(waypoint_clear) {
      ROS_INFO("[%s] Path to waypoint is clear, resuming", name_.c_str());
      // Obstruction Status reporting
      publishObstructionReport(current_pose_, false);
      obstructed_ = false;
      waiting_for_obstacle_clearance_ = false;
      Resume();
      return;
    }
    // waypoint not clear, check if waiting has timed out
    else if (waiting_for_obstacle_clearance_ 
             && ros::Time::now().toSec() - pause_start_time_.toSec() > clearing_timeout_ 
             && clearing_timeout_ > 0) 
    {
      current_index_++;
      pause_start_time_ = ros::Time::now();
      // timeout at final index
      if (current_index_ >= loaded_path_.poses.size()) {
        // report obstruction
        publishObstructionReport(loaded_path_.poses[loaded_path_.poses.size()-1].pose, true);
        // finish up path load
        final_end_point_fail_ = true;
        end_ = true;
        start_ = false;
        cancel_ = true;
        current_index_ = 0;
        actionlib_msgs::GoalID cancel_path;
        cancel_pub_.publish(cancel_path);
        std_msgs::Bool boolean;
        std_msgs::Bool fail_;
        boolean.data = false;
        fail_.data = true;
        ROS_INFO("[%s] Final waypoint cannot be reached, clearing timeout",name_.c_str());
        fail_pub_.publish(fail_);
        start_pub_.publish(boolean);
        return;
      }
      // timeout, go to next waypoint
      ROS_INFO("[%s] Waiting for obstacle clearance exceeded timeout (no viable path), skipping waypoint to %lu", name_.c_str(),current_index_);
    }
  }

  // if (obstructed_)
  // {
  //   if (!checkObstruction(loaded_path_.poses[current_index_]))
  //   {
  //     nav_msgs::GetPlan srv;
  //     populateClient(srv, loaded_path_.poses[current_index_].pose);
  //     plan_client_.call(srv);
  //     if (srv.response.plan.poses.size() > 0)
  //     {
  //       ROS_INFO("Path to waypoint is clear, resuming");
      
  //       // Obstruction Status reporting
  //       publishObstructionReport(current_pose_, false);
  //       // movel_seirios_msgs::ObstructionStatus report_obs;
  //       // report_obs.reporter = "path_recall";
  //       // report_obs.status = "false";
  //       // report_obs.location = current_pose_;
  //       // obstruction_status_pub_.publish(report_obs);

  //       obstructed_ = false;
  //       waiting_for_obstacle_clearance_ = false;
  //       Resume();
  //     }
  //     else if (waiting_for_obstacle_clearance_ && ros::Time::now().toSec() - pause_start_time_.toSec() > clearing_timeout_ && clearing_timeout_ > 0)
  //     {
  //       current_index_++;
  //       // timeout at final index
  //       if (current_index_ >= loaded_path_.poses.size())
  //       {
  //         // report obstruction
  //         publishObstructionReport(loaded_path_.poses[loaded_path_.poses.size()-1].pose, true);
  //         // movel_seirios_msgs::ObstructionStatus report_obs;
  //         // report_obs.reporter = "path_recall";
  //         // report_obs.status = "true";
  //         // report_obs.location = loaded_path_.poses[loaded_path_.poses.size()-1].pose;
  //         // obstruction_status_pub_.publish(report_obs);

  //         // finish up path load
  //         end_ = true;
  //         start_ = false;
  //         cancel_ = true;
  //         current_index_ = 0;
  //         actionlib_msgs::GoalID cancel_path;
  //         cancel_pub_.publish(cancel_path);
  //         std_msgs::Bool boolean;
  //         boolean.data = false;
  //         ROS_INFO("Final waypoint cannot be reached, clearing timeout");
  //         start_pub_.publish(boolean);
  //         return;
  //       }
  //       ROS_INFO("Waiting for obstacle clearance exceeded timeout (no viable path), skipping waypoint to %lu", current_index_);
  //       obstructed_ = false;
  //       waiting_for_obstacle_clearance_ = false;
  //       Resume();
  //     }
  //   }
  //   else if (waiting_for_obstacle_clearance_ && ros::Time::now().toSec() - pause_start_time_.toSec() > clearing_timeout_ && clearing_timeout_ > 0)
  //   {
  //     current_index_++;
  //     if (current_index_ >= loaded_path_.poses.size())
  //     {
  //       // report obstruction
  //       publishObstructionReport(loaded_path_.poses[loaded_path_.poses.size()-1].pose, true);
  //       // movel_seirios_msgs::ObstructionStatus report_obs;
  //       // report_obs.reporter = "path_recall";
  //       // report_obs.status = "true";
  //       // report_obs.location = loaded_path_.poses[loaded_path_.poses.size()-1].pose;
  //       // obstruction_status_pub_.publish(report_obs);

  //       // finish up path load
  //       end_ = true;
  //       start_ = false;
  //       cancel_ = true;
  //       current_index_ = 0;
  //       actionlib_msgs::GoalID cancel_path;
  //       cancel_pub_.publish(cancel_path);
  //       std_msgs::Bool boolean;
  //       boolean.data = false;
  //       ROS_INFO("Final waypoint cannot be reached, clearing timeout");
  //       start_pub_.publish(boolean);
  //       return;
  //     }
  //     ROS_INFO("Waiting for obstacle clearance exceeded timeout, skipping waypoint to %lu", current_index_);
  //     obstructed_ = false;
  //     waiting_for_obstacle_clearance_ = false;
  //     Resume();
  //   }
  // }
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
  if (!start_)
  {
    ROS_INFO("[%s] Got move_base success, but path load isn't active",name_.c_str());
    return;
  }
  if (current_index_ >= loaded_path_.poses.size())
    current_index_ = loaded_path_.poses.size()-1;

  if (obstructed_ && (msg->status.status == 3 || msg->status.status == 4))
  {
  //   ROS_INFO("attempting clear");
  //   std_srvs::Empty emp;
  //   clear_costmaps_client_.call(emp);
    ROS_INFO("[%s] Completed pseudo goal, waiting for obstacle clearance",name_.c_str());
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
      // ROS_INFO("reset ping counter, increase index from goal");
      ping_counter_ = 0;
      current_index_++;
      ROS_INFO_STREAM("[" << name_ << "] move_base goal reached and distances checked out, bumping index to: " << current_index_);
      //publishPath(loaded_path_.poses[current_index_].pose);
    }
    else
    {
      ROS_INFO("[%s] 1.Got move_base success, but distances don't check out",name_.c_str());
      ROS_INFO("[%s] 1.linear %5.2f out of %5.2f", name_.c_str(), dxy, mb_xy_tolerance_);
      ROS_INFO("[%s] 1.angular %5.2f of %5.2f", name_.c_str(), dyaw, mb_yaw_tolerance_);
    }
    ROS_INFO("[%s] index value ongoal", name_.c_str());
    publishPath(loaded_path_.poses[current_index_].pose, true);
  }
  
  //! Path loading ends once all waypoints are sent to move_base
  if (end_ && (msg->status.status == 3 || msg->status.status == 4)) {
    // validate distance to current waypoint
    double dxy = calculateLength(current_pose_, loaded_path_.poses[current_index_].pose);
    double dyaw = calculateAng(current_pose_, loaded_path_.poses[current_index_].pose);
    ROS_INFO("[%s] completion check idx %lu, dxy %5.2f/%5.2f, dyaw %5.2f/%5.2f", name_.c_str(),
             current_index_, dxy, mb_xy_tolerance_, dyaw, mb_yaw_tolerance_);

    if (dxy <= mb_xy_tolerance_ && dyaw <= mb_yaw_tolerance_)
    {
      std_msgs::Bool boolean;
      boolean.data = false;
      ROS_INFO("[%s] completion, reached final waypoint", name_.c_str());
      start_pub_.publish(boolean);
      start_ = false;  
    }
    else
    {
      if(final_goal_ping_counter_>5)
      {
          final_end_point_fail_ = true;
          end_ = true;
          start_ = false;
          cancel_ = true;
          current_index_ = 0;
          actionlib_msgs::GoalID cancel_path;
          cancel_pub_.publish(cancel_path);
          std_msgs::Bool boolean;
          std_msgs::Bool fail_;
          boolean.data = false;
          fail_.data = true;
          fail_pub_.publish(fail_);
          start_pub_.publish(boolean);
          ROS_INFO("2.Got move_base success, but distances don't check out (last index)");
          ROS_INFO("2.linear %5.2f out of %5.2f", dxy, mb_xy_tolerance_);
          ROS_INFO("2.angular %5.2f of %5.2f", dyaw, mb_yaw_tolerance_);
          start_pub_.publish(boolean);
          return;   
      }
      ROS_INFO("2.Got move_base success, but distances don't check out (last index)");
      ROS_INFO("2.linear %5.2f out of %5.2f", dxy, mb_xy_tolerance_);
      ROS_INFO("2.angular %5.2f of %5.2f", dyaw, mb_yaw_tolerance_);
      publishPath(loaded_path_.poses[current_index_].pose, true);
      final_goal_ping_counter_++;
      ros::Duration(1).sleep();
    }
  }
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
  if (row < latest_costmap_.info.height 
      && col < latest_costmap_.info.width
      && row >= 0 && col >= 0)
  {
    int idx = row * latest_costmap_.info.width + col;
    int occupancy = latest_costmap_.data[idx];
    if (occupancy > max_occupancy) {
      max_occupancy = occupancy;
    }
  }
  if (max_occupancy >= obstruction_threshold_) {
    return true;
  }
  return false;
}

void PathLoadSegments::onPauseStatus(std_msgs::Bool msg)
{
  ts_pause_status_ = msg.data;
}
