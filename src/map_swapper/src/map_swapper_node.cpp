#include "map_swapper/map_swapper.h"
#include <movel_hasp_vendor/license.h>

int main(int argc, char** argv)
{
#ifdef MOVEL_LICENSE                                                                                                    
  MovelLicense ml(34);                                                                                                   
  if (!ml.login())                                                                                                      
    return 1;                                                                                                           
#endif

  ros::init(argc, argv, "map_swapper");
  MapSwapper ms;

  ros::spin();

  return 0;
}