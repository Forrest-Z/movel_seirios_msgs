#include <vodom_vodom/vodom.hpp>
#include <opencv2/features2d.hpp>
#include <iostream>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "vodom_vodom");
  ros::NodeHandle nh;
  ROS_INFO("hey");
  RGBDVodom vodomateur(nh);
  ROS_INFO("vodomateur ready");

  return 0;
}