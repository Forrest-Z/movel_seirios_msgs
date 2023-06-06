#include <movel_zone_coverage/zone_coverage_server.h>

int main(int argc, char** argv)
{
#ifdef MOVEL_LICENSE
  MovelLicense ml;
  if (!ml.login())
    return 1;
#endif

  ros::init(argc, argv, "zone_coverage_server");
  ros::NodeHandle nh;
  ZoneCoverageServer zcs(nh);

#ifdef MOVEL_LICENSE
  ml.logout();
#endif

  return (0);
}