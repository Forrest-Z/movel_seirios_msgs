#include <ros/ros.h>
#include <geometric_docking/geometric_docker.hpp>
#include <movel_hasp_vendor/license.h>

int main(int argc, char** argv)
{
#ifdef MOVEL_LICENSE
  MovelLicense ml(21);
  if (!ml.login())
    return 1;
#endif
  ros::init(argc, argv, "geometric_docking_node");
  ros::NodeHandle nh("~");

  GeometricDocker gd(nh);
  
  gd.dock_start_sub_ = nh.advertiseService("startDocking", &GeometricDocker::dockStartCb, &gd);
  gd.dock_cancel_sub_ = nh.advertiseService("stopDocking", &GeometricDocker::dockCancelCb, &gd);
  gd.dock_resume_sub_ = nh.advertiseService("resumeDocking", &GeometricDocker::dockResumeCb, &gd);
  gd.dock_pause_sub_ = nh.advertiseService("pauseDocking", &GeometricDocker::dockPauseCb, &gd);

  gd.dock_status_pub_ = nh.advertise<std_msgs::UInt8>("status", 1);
  ros::spin();

#ifdef MOVEL_LICENSE
  ml.logout();
#endif
  return 0;
}