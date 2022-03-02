#include <point_cloud_concatenation/point_cloud_concatenation.h>
#include <movel_hasp_vendor/license.h>

using namespace std;

int main(int argc, char** argv)
{
#ifdef MOVEL_LICENSE
  MovelLicense ml;
  if (!ml.login())
    return 1;
#endif
  ros::init(argc, argv, "camera_docking");

  FusedPcl* node = new FusedPcl();
  ros::spin();

#ifdef MOVEL_LICENSE
  ml.logout();
#endif

  return 0;
}
