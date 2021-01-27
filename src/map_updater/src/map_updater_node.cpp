#include <ros/ros.h>
#include "map_updater/map_updater.h"
#include <movel_hasp_vendor/license.h>

int main(int argc, char** argv)
{
#ifdef MOVEL_LICENSE
  MovelLicense ml(21);
  if (!ml.login())
    return 1;
#endif

  ros::init(argc, argv, "map_updater");
  MapUpdater map_updater;
  ros::spin();

#ifdef MOVEL_LICENSE
  ml.logout();
#endif

  return 0;
}
