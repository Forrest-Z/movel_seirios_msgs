#include <ros/ros.h>
#include "map_splitter/map_splitter.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "map_splitter");

  MapSplitter ms;

  ros::spin();

  return 0;
}