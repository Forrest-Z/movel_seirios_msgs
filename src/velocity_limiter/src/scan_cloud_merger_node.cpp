#include <point_cloud_merger/point_cloud_merger.h>
#include <movel_hasp_vendor/license.h>

int main(int argc, char **argv)
{
  #ifdef MOVEL_LICENSE
  MovelLicense ml(8);                                                                                                   
  if (!ml.login())
    return 1;                                                                                                           
  #endif

  ros::init(argc, argv, "scan_cloud_merger");
  PointCloudMerger point_cloud_merger(0);

  #ifdef MOVEL_LICENSE                                                                                                    
  ml.logout();          
  #endif    

  return 0;
}
