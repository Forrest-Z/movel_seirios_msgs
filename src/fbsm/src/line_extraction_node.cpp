#include "fbsm/line_extraction_ros.h"
#include <ros/console.h>
#include <movel_hasp_vendor/license.h>

int main(int argc, char** argv)
{
#ifdef MOVEL_LICENSE
  MovelLicense ml;
  if (!ml.login())
    return 1;
#endif

  if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug))
  {
    ros::console::notifyLoggerLevelsChanged();
  }

  ROS_DEBUG("Starting line_extraction_node.");

  ros::init(argc, argv, "line_extraction_node");
  line_extraction::LineExtractionROS line_extractor;

#ifdef MOVEL_LICENSE
  ml.logout();
#endif
}
