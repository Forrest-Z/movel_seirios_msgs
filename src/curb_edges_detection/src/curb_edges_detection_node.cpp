#include <curb_edges_detection/curb_edges_detection.h>

int main(int argc, char** argv)
{
#ifdef MOVEL_LICENSE
  MovelLicense ml;
  if (!ml.login())
    return 1;
#endif
    ros::init(argc, argv, "movel_ramp_detection");
    CurbEdgesDetection curb_edges;
    ros::spin();
#ifdef MOVEL_LICENSE
  ml.logout();
#endif
    return 0;
}
