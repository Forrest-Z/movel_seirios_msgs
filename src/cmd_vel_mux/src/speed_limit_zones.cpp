#include <cstdlib>
#include <cmd_vel_mux/speed_limit_zones.h>
#include <movel_hasp_vendor/license.h>

SpeedLimitZones::SpeedLimitZones() {
  if (!setupTopics()) {
    ROS_INFO("failed to setup topics");
    return;
  }
}

bool SpeedLimitZones::setupTopics() {
  return true;
}


int main(int argc, char **argv) {
  #ifdef MOVEL_LICENSE
    MovelLicense ml;
    if (!ml.login())
      return 1;
  #endif

  ros::init(argc, argv, "speed_limit_zones");
  
  SpeedLimitZones spd_zone;
  ros::spin();
 
  #ifdef MOVEL_LICENSE
    ml.logout();
  #endif

  return 0;
}