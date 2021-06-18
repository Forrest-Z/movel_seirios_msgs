#include "map_swapper/map_swapper.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "map_swapper");
  MapSwapper ms;

  ros::spin();

  return 0;
}