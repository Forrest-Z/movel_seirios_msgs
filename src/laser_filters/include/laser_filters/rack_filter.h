#ifndef RACK_FILTER_H
#define RACK_FILTER_H

#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <laser_geometry/laser_geometry.h>

#include <laser_filters/RackFilterUpdate.h>

#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

namespace laser_filters
{
typedef std::vector<tf::Point> Box;
class RackFilter
{
public:
  RackFilter();

private:
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  ros::Subscriber input_scan_sub_;
  ros::Publisher filtered_scan_pub_;
  ros::ServiceServer update_param_srv_;

  bool filter_enabled_;

  void onInputScan(const sensor_msgs::LaserScan &input_scan);
  bool onUpdateParam(laser_filters::RackFilterUpdateRequest &req, laser_filters::RackFilterUpdateResponse &res);

  bool inBox(tf::Point &point, Box& box);
  std::string base_frame_;
  laser_geometry::LaserProjection projector_;

  // tf listener to transform scans into the box_frame
  tf::TransformListener tf_;

  // defines two opposite corners of the box
  std::vector<Box> boxes_;
};
} // namespace laser_filters

#endif
