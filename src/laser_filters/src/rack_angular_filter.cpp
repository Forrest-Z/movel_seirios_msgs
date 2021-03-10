#include <laser_filters/rack_angular_filter.h>
#include <ros/ros.h>
#include <movel_hasp_vendor/license.h>
#include <laser_filters/RackLegAngular.h>

namespace laser_filters
{
  RackAngularFilter::RackAngularFilter() : filter_enabled_(false),
  nh_private_("~")
  {
    input_scan_sub_ = nh_private_.subscribe("scan_in", 1, &RackAngularFilter::onInputScan, this);
    filtered_scan_pub_ = nh_private_.advertise<sensor_msgs::LaserScan>("scan_out", 1);
    update_param_srv_ = nh_.advertiseService("/update_rack_angular_filter", &RackAngularFilter::onUpdateParam, this);

    ros::spin();
  }

  bool RackAngularFilter::onUpdateParam(RackAngularFilterUpdateRequest &req, RackAngularFilterUpdateResponse &res)
  {
    filter_enabled_ = req.enable;
    bounds_.clear();

    if (!filter_enabled_)
    {
      ROS_INFO("Rack filter disabled");
    }

    else
    {
      ROS_INFO("Rack filter enabled");

      for (RackLegAngular leg : req.legs)
      {
        AngularBounds bounds({leg.lower_angle, leg.upper_angle, leg.range});
        bounds_.push_back(bounds);

        ROS_INFO("Rack leg angular bounds: {lower: %f, upper: %f}", bounds[0] * 180.0 / M_PI, bounds[1] * 180.0 / M_PI);
      }
    }

    res.status = true;

    return true;
  }

  void RackAngularFilter::onInputScan(const sensor_msgs::LaserScan &input_scan)
  {
    sensor_msgs::LaserScan filtered_scan = input_scan;

    if (filter_enabled_)
    {
      filtered_scan = input_scan; //copy entire message

      double current_angle = input_scan.angle_min;
      unsigned int count = 0;
      //loop through the scan and remove ranges at angles between lower_angle_ and upper_angle_
      for(unsigned int i = 0; i < input_scan.ranges.size(); ++i){
        for (AngularBounds bound : bounds_)
        {
          if ((current_angle > bound[0]) && (current_angle < bound[1]) && input_scan.ranges[i] < bound[2])
          {
            filtered_scan.ranges[i] = input_scan.range_max + 1.0;
            if(i < filtered_scan.intensities.size()){
              filtered_scan.intensities[i] = 0.0;
            }
            count++;
          } 
        }
        current_angle += input_scan.angle_increment;
      }

      ROS_DEBUG("Filtered out %u points from the laser scan.", count);
    }

    filtered_scan_pub_.publish(filtered_scan);
  }
}

int main(int argc, char **argv)
{
  #ifdef MOVEL_LICENSE
  MovelLicense ml(23);
  if (!ml.login())
    return 1;
  #endif

  ros::init(argc, argv, "rack_angular_filter");
  laser_filters::RackAngularFilter node;
  #ifdef MOVEL_LICENSE
  ml.logout();
  #endif
  return 0;
}
