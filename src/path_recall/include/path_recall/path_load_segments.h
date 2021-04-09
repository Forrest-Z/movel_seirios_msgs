/*!
 *  This is a path loading code that loads paths from yaml files, and executes
 * the path by sending waypoints of the path in sequence to move_base node.
 */

#ifndef PATH_LOAD_SEGMENTS_H_
#define PATH_LOAD_SEGMENTS_H_

#include "yaml-cpp/yaml.h"
#include <ros/ros.h>

#include <actionlib_msgs/GoalID.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <move_base_msgs/MoveBaseActionFeedback.h>
#include <move_base_msgs/MoveBaseActionResult.h>
#include <nav_msgs/GetPlan.h>
#include <nav_msgs/Path.h>
#include <path_recall/PathCheck.h>
#include <path_recall/PathInfo.h>
#include <path_recall/PathName.h>
#include <path_recall/SavePath.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
#include <std_srvs/Trigger.h>
#include <tf/tf.h>
#include <movel_seirios_msgs/ObstructionStatus.h>

#include <algorithm>
#include <math.h>
#include <stdlib.h>
#include <string>

class PathLoadSegments {
private:
  size_t current_index_; //!< Keeps track of current waypoint in list of
                         //!< waypoints of the path
  bool start_;  //!< Flag for starting path following (ie reaching the first
                //!< waypoint of path)
  bool pause_;  //!< Flag for service call to pause path following
  bool obstructed_; //!< Flag for when the current waypoint is obstructed
  bool cancel_; //!< Flag for stopping path following before reaching the final
                //!< waypoint
  bool end_;    //!< Flag for reaching the final waypoint
  YAML::Node config_;                //!< Loaded yaml data
  std::string path_name_;            //!< Path name
  geometry_msgs::Pose current_pose_; //!< Robot current pose
  nav_msgs::Path loaded_path_;       //!< Path loaded for execution

  void
  publishPath(geometry_msgs::Pose target_pose); //!< Publish waypoints of path
  void populateClient(
      nav_msgs::GetPlan &srv,
      geometry_msgs::Pose target_pose); //!< Populate service client for calling
                                        //!< move_base/make_plan service
  double calculateLength(geometry_msgs::Pose current_pose,
                         geometry_msgs::Pose target_pose);
  double calculateAng(geometry_msgs::Pose current_pose,
                      geometry_msgs::Pose target_pose);
  void findShortestPath(); //!< Find waypoint with the shortest path from robot
                           //!< position
                           //
  geometry_msgs::Pose getNearestPseudoPoint();

public:
  PathLoadSegments();

  //! Parameters to be loaded
  std::string yaml_path_;   //!< Directory path for saving paths in files
  double max_plan_length_;  //!< Exceeding max length of path plan activates
                            //!< 'findShortestPath' function (when blocked by
                            //! obstacle)
  bool skip_on_obstruction_;
  double update_min_dist_;  //!< Minimum distance of robot from target waypoint
                            //!< before going for next waypoint
  double look_ahead_dist_;  //!< Target waypoint distance from robot position
  double look_ahead_angle_; //!< Targeet waypoint orientation difference from
                            //!< robot orientation
  double update_time_interval_; //!< Time interval for publishing waypoints

  double mb_xy_tolerance_; // xy tolerance from move_base
  double mb_yaw_tolerance_; // yaw tolerance from move_base

  //! Load path
  ros::Publisher display_pub_; //!< Display loaded path
  ros::Publisher info_pub_;    //!< Publish data on the path name and waypoints
  ros::Publisher start_pub_;   //!< Show start and end of path loading
  ros::Publisher path_load_pub_;   //!< Publish goal to move_base
  ros::Publisher obstruction_status_pub_;   //!< Reporting to UI purposes
  ros::ServiceClient plan_client_; //!< Get path plan from move_base
  
  void getPose(
      const geometry_msgs::Pose::ConstPtr &msg); //!< Get current pose of robot
  void onFeedback(const move_base_msgs::MoveBaseActionFeedback::ConstPtr
                      &msg); //!< Get feedback from move_base
  void onGoal(const move_base_msgs::MoveBaseActionResult::ConstPtr
                  &msg); //!< Callback for reaching a waypoint
  bool loadYAML(std::string name,
                nav_msgs::Path &output_path); //!< Load path from YAML file
  bool loadPath(nav_msgs::Path path);         //!< Load path
  bool onLoad(path_recall::PathName::Request &req,
              path_recall::PathName::Response
                  &res); //!< ROS callback for loading path by name
  bool
  getPath(path_recall::SavePath::Request &req,
          path_recall::SavePath::Response
              &res); //!< ROS callback for loading path given in service request

  //! Pause loading
  void Pause(); //!< Pause path following
  bool onPause(std_srvs::Trigger::Request &req,
               std_srvs::Trigger::Response &res); //!< ROS callback

  //! Resume loading
  bool Resume(); //!< Resume path following
  bool onResume(std_srvs::Trigger::Request &req,
                std_srvs::Trigger::Response &res); //!< ROS callback

  //! Cancel loading
  ros::Publisher cancel_pub_; //!< Publisher to topic for cancelling goal
  void Cancel();              //!< Cancel path following
  bool onCancel(std_srvs::Trigger::Request &req,
                std_srvs::Trigger::Response &res); //!< ROS callback

  //! Check current pose of robot
  bool Check(std::string name,
             float threshold); //!< Check robot pose w.r.t. first waypoint
  bool onCheck(path_recall::PathCheck::Request &req,
               path_recall::PathCheck::Response &res); //!< ROS callback
};

#endif
