#include <ros/ros.h>
#include "map_splitter/map_splitter.h"
#include <movel_hasp_vendor/license.h>

int main(int argc, char** argv)
{
#ifdef MOVEL_LICENSE                                                                                                    
  MovelLicense ml;                                                                                                   
  if (!ml.login())                                                                                                      
    return 1;                                                                                                           
#endif
  ros::init(argc, argv, "map_splitter");

  MapSplitter ms;

  ros::spin();

  return 0;
}