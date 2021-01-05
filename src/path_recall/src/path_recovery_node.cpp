/*!
 *  USAGE:
 *  1. Recovery service (/path_recovery/recovery) : loads recovery path for uncovered segments of loaded path with name
 * of loaded path as input
 *  2. Display topic (/path_recovery/display)     : shows recovery path to be saved
 */

#include <path_recall/path_recovery.h>
#include <movel_hasp_vendor/license.h>

#include <ros/console.h>
#include <ros_utils/ros_utils.h>

PathRecovery Recovery;

bool loadParams(ros::NodeHandle& nh_private_)
{
  ros_utils::ParamLoader loader(nh_private_);
  loader.get_required("update_min_d", Recovery.update_min_d);
  loader.get_required("update_min_a", Recovery.update_min_a);
  loader.get_required("check_tolerance", Recovery.tolerance);
  return loader.params_valid();
}

int main(int argc, char** argv)
{
#ifdef MOVEL_LICENSE
  MovelLicense ml(30);
  if (!ml.login())
    return 1;
#endif

  std::string node_name_ = "path_recovery";
  ros::init(argc, argv, node_name_);

  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_("~");

  ros::Time::waitForValid();
  if (!loadParams(nh_private_))
  {
    ROS_FATAL("Error during parameter loading. Shutting down.");
    return 0;
  }
  ROS_INFO("All parameters loaded. Launching.");

  Recovery.display_pub_ = nh_private_.advertise<nav_msgs::Path>("display", 1);
  Recovery.save_client_ = nh_.serviceClient<path_recall::SavePath>("/path_saver/save");
  Recovery.load_client_ = nh_.serviceClient<path_recall::PathName>("/path_load/load");

  ros::Subscriber path_recovery_load_ =
      nh_.subscribe("/path_load/path_info", 1, &PathRecovery::getOriginalPath, &Recovery);
  ros::Subscriber path_recovery_start_ = nh_.subscribe("/path_load/start", 1, &PathRecovery::onStart, &Recovery);
  ros::Subscriber path_recovery_sub_ = nh_.subscribe("/pose", 1, &PathRecovery::getPose, &Recovery);

  ros::ServiceServer recovery_srv_ = nh_private_.advertiseService("recovery", &PathRecovery::onRecovery, &Recovery);

  ros::spin();
#ifdef MOVEL_LICENSE
  ml.logout();
#endif
}
