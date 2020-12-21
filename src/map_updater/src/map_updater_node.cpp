#include <ros/ros.h>
#include "map_updater/map_updater.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "map_updater");
  MapUpdater map_updater;
  ros::spin();

  return 0;
}