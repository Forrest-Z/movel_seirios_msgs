/*!
 *  This is a path saving code that saves each path in individual yaml files in a specified directory.
 */

#ifndef PATH_SAVER_H_
#define PATH_SAVER_H_

#include "yaml-cpp/yaml.h"
#include <iostream>
#include <fstream>
#include <geometry_msgs/Pose.h>
#include <path_recall/PathName.h>
#include <std_srvs/Trigger.h>
#include <tf/tf.h>
#include <math.h>
#include <stdlib.h>
#include <string>
#include <path_recall/PathInfo.h>
#include <path_recall/SavePath.h>

class PathSaver
{
private:
  YAML::Node config;                   //!< Path to be saved in yaml format
  std::ofstream yaml_file;             //!< File to be saved in
  bool start;                          //!< Flag for the start of saving the first point
  bool save;                           //!< Flag for activating path saving
  geometry_msgs::Pose previous_point;  //!< Last saved point for comparison with current position
  std::string path_name;               //!< Path name from user input
  int count;                           //!< Index for saving waypoints in sequence

  bool calculateDistance(geometry_msgs::Pose pose1, geometry_msgs::Pose pose2);  //!< Calculate distance and angular
                                                                                 //! difference between last saved
                                                                                 //! waypoint to current pose
  void writeWaypoint(geometry_msgs::Pose pose);                                  //!< Write waypoint to yaml file
  void toYAML(YAML::Node config, geometry_msgs::Pose pose, std::string name);    //!< Format waypoint into YAML data

public:
  PathSaver();

  //! Parameters to be loaded
  double update_min_d;    //!< Threshold linear distance from last saved point
  double update_min_a;    //!< Threshold angular difference from last saved point
  std::string yaml_path;  //!< File directory for saving yaml files

  //! Start path saving
  bool Start(std::string name);                                                             //!< Start path saving
  bool onStart(path_recall::PathName::Request& req, path_recall::PathName::Response& res);  //!< ROS callback
  void onSave(const geometry_msgs::Pose::ConstPtr& msg);  //!< Get and save current pose to file while path saving is
                                                          //! active

  //! Stop path saving
  void Stop();                                                                       //!< Stop path saving
  bool onStop(std_srvs::TriggerRequest& request, std_srvs::Trigger::Response& res);  //!< ROS callback

  //! Save generated path without going through teleoperation
  bool writePath(std::string name, nav_msgs::Path path);  //!< Write path to yaml file
  bool savePath(path_recall::SavePath::Request& req, path_recall::SavePath::Response& res);  //!< ROS callback
};

#endif
