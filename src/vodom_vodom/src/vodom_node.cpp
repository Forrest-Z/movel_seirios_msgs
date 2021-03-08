#include <vodom_vodom/vodom.hpp>
#include <opencv2/features2d.hpp>
#include <iostream>
#include <movel_hasp_vendor/license.h>

int main(int argc, char** argv)
{
#ifdef MOVEL_LICENSE
  MovelLicense ml(6);
  if (!ml.login())
    return 1;
#endif

  ros::init(argc, argv, "vodom_vodom");
  ros::NodeHandle nh;
  ROS_INFO("hey");
  RGBDVodom vodomateur(nh);
  ROS_INFO("vodomateur ready");

#ifdef MOVEL_LICENSE
  ml.logout();
#endif

  return 0;
}
