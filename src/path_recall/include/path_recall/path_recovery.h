/*!
 *  This code checks for uncovered path segments after path loading is done, and saves the segments as a path in a yaml
 * file to be loaded later if needed.
 */

#ifndef PATH_RECOVERY_H_
#define PATH_RECOVERY_H_

#include <std_msgs/String.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Empty.h>
#include <tf/tf.h>
#include <math.h>
#include <stdlib.h>
#include <path_recall/PathName.h>
#include <path_recall/PathInfo.h>
#include <path_recall/SavePath.h>
#include <std_msgs/Bool.h>
#include <ros/ros.h>

class PathRecovery
{
private:
  bool start;                          //!< Flag for recording the first waypoint
  bool save;                           //!< Flag for path recording activation
  std::string path_name;               //!< Loaded path's name
  geometry_msgs::Pose previous_point;  //!< Last recorded point
  nav_msgs::Path original_path;        //!< Path to be loaded
  nav_msgs::Path actual_path;          //!< Actual path travelled by the robot

  bool calculateDistance(geometry_msgs::Pose pose1,
                         geometry_msgs::Pose pose2);  //!< Calculate distance of last saved waypoint to current pose

public:
  PathRecovery();

  //! Parameters to be loaded
  double update_min_d;  //!< Threshold linear distance from last recorded waypoint
  double update_min_a;  //!< Threshold angular difference from last recorded waypoint
  double tolerance;     //!< Tolerance distance from original path to consider the path segment as covered

  //! ROS callbacks for inputs on original path and actual path travelled
  void getOriginalPath(const path_recall::PathInfo::ConstPtr& msg);  //!< Callback for getting original path and its
                                                                     //! name
  void onStart(const std_msgs::Bool::ConstPtr& msg);                 //!< Callback for start and end of path loading
  void getPose(const geometry_msgs::Pose::ConstPtr& msg);  //!< Callback to save current pose as waypoint of actual path

  //! Generate and save recovery path
  ros::Publisher display_pub_;      //!< For publishing saved recovery path for display
  ros::ServiceClient save_client_;  //!< For calling path saving service
  nav_msgs::Path comparePath(nav_msgs::Path original_path, nav_msgs::Path actual_path);  //!< Compare original path and
                                                                                         //! actual path to find
                                                                                         //! uncovered segments
  void saveRecoveryPath(std::string name, nav_msgs::Path original_path,
                        nav_msgs::Path actual_path);  //!< Save recovery path to yaml file using path_saver

  //! Load recovery path
  ros::ServiceClient load_client_;  //!< For calling path loading service
  bool Recovery(std::string name);  //!< Load recovery path from yaml file using path_load_segments
  bool onRecovery(path_recall::PathName::Request& req, path_recall::PathName::Response& res);  //!< ROS callback
};

#endif
