#ifndef RACK_ANGULAR_FILTER_H
#define RACK_ANGULAR_FILTER_H

#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <laser_geometry/laser_geometry.h>

#include <movel_laser_filters/RackAngularFilterUpdate.h>

#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

namespace movel_laser_filters
{
typedef std::vector<float> AngularBounds;
class RackAngularFilter
{
public:
  RackAngularFilter();

private:
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  ros::Subscriber input_scan_sub_;
  ros::Publisher filtered_scan_pub_;
  ros::ServiceServer update_param_srv_;

  bool filter_enabled_;

  void onInputScan(const sensor_msgs::LaserScan &input_scan);
  bool onUpdateParam(movel_laser_filters::RackAngularFilterUpdateRequest &req, movel_laser_filters::RackAngularFilterUpdateResponse &res);

  std::vector<AngularBounds> bounds_;
};
} // namespace movel_laser_filters

#endif
