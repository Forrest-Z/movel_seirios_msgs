#include "laser_filters/rack_filter.h"
#include <ros/ros.h>
#include <movel_hasp_vendor/license.h>
#include <laser_filters/RackLeg.h>

laser_filters::RackFilter::RackFilter() : filter_enabled_(false),
                                          nh_private_("~")
{
  input_scan_sub_ = nh_private_.subscribe("scan_in", 1, &laser_filters::RackFilter::onInputScan, this);
  filtered_scan_pub_ = nh_private_.advertise<sensor_msgs::LaserScan>("scan_out", 1);
  update_param_srv_ = nh_.advertiseService("/update_rack_filter", &laser_filters::RackFilter::onUpdateParam, this);

  ros::spin();
}

bool laser_filters::RackFilter::onUpdateParam(laser_filters::RackFilterUpdateRequest &req, laser_filters::RackFilterUpdateResponse &res)
{
  filter_enabled_ = req.enable;
  boxes_.clear();
  
  if (!filter_enabled_)
  {
    //TODO: Uncomment when services is implemented. Currently too noisy for topic-based cb.
    //ROS_INFO("Rack filter disabled");
  }

  else
  {
    //ROS_INFO("Rack filter enabled");

    base_frame_ = req.base_frame;
    //ROS_INFO("base_frame: %s", base_frame_.c_str());

    for (laser_filters::RackLeg &leg : req.legs)
    {
      tf::Point min;
      min.setX(leg.min_x);
      min.setY(leg.min_y); 

      tf::Point max;
      max.setX(leg.max_x);
      max.setY(leg.max_y); 

      //ROS_INFO("x: {min: %f, max: %f}, y: {min: %f, max: %f}", min.getX(), max.getX(), min.getY(), max.getY());

      Box box({min, max});
      boxes_.push_back(box);
    }
  }

  res.status = true;

  return true;
}

void laser_filters::RackFilter::onInputScan(const sensor_msgs::LaserScan &input_scan)
{
  sensor_msgs::LaserScan output_scan = input_scan;

  if (filter_enabled_)
  {
    sensor_msgs::PointCloud2 laser_cloud;

    std::string error_msg;

    bool success = tf_.waitForTransform(
      base_frame_, 
      input_scan.header.frame_id,
      input_scan.header.stamp + ros::Duration().fromSec(input_scan.ranges.size() * input_scan.time_increment),
      ros::Duration(1.0),
      ros::Duration(0.01),
      &error_msg
    );
    if (!success)
    {
      ROS_WARN("Could not get transform, irgnoring laser scan! %s", error_msg.c_str());
      return;
    }

    try
    {
      projector_.transformLaserScanToPointCloud(base_frame_, input_scan, laser_cloud, tf_);
    }
    catch (tf::TransformException &ex)
    {
      ROS_WARN_THROTTLE(1, "Dropping Scan: Tansform unavailable %s", ex.what());
      return;
    }
    const int i_idx_c = sensor_msgs::getPointCloud2FieldIndex(laser_cloud, "index");
    const int x_idx_c = sensor_msgs::getPointCloud2FieldIndex(laser_cloud, "x");
    const int y_idx_c = sensor_msgs::getPointCloud2FieldIndex(laser_cloud, "y");
    const int z_idx_c = sensor_msgs::getPointCloud2FieldIndex(laser_cloud, "z");

    if (i_idx_c == -1 || x_idx_c == -1 || y_idx_c == -1 || z_idx_c == -1)
      ROS_INFO_THROTTLE(.3, "x, y, z and index fields are required, skipping scan");

    const int i_idx_offset = laser_cloud.fields[i_idx_c].offset;
    const int x_idx_offset = laser_cloud.fields[x_idx_c].offset;
    const int y_idx_offset = laser_cloud.fields[y_idx_c].offset;
    const int z_idx_offset = laser_cloud.fields[z_idx_c].offset;

    const int pstep = laser_cloud.point_step;
    const long int pcount = laser_cloud.width * laser_cloud.height;
    const long int limit = pstep * pcount;

    int i_idx, x_idx, y_idx, z_idx;
    for (i_idx = i_idx_offset, x_idx = x_idx_offset, y_idx = y_idx_offset, z_idx = z_idx_offset;
         x_idx < limit;
         i_idx += pstep, x_idx += pstep, y_idx += pstep, z_idx += pstep)
    {
      // TODO works only for float data types and with an index field
      // I'm working on it, see https://github.com/ros/common_msgs/pull/78
      float x = *((float *)(&laser_cloud.data[x_idx]));
      float y = *((float *)(&laser_cloud.data[y_idx]));
      float z = *((float *)(&laser_cloud.data[z_idx]));
      int index = *((int *)(&laser_cloud.data[i_idx]));

      tf::Point point(x, y, z);

      for (Box box : boxes_) //use vector of boxes to remove points in it
      {
        if (inBox(point, box))
          output_scan.ranges[index] = std::numeric_limits<float>::quiet_NaN();
      }
    }
  }

  filtered_scan_pub_.publish(output_scan);
}

bool laser_filters::RackFilter::inBox(tf::Point &point, Box& box)
{
  tf::Point min = box[0];
  tf::Point max = box[1];
  return point.x() < max.x() && point.x() > min.x() &&
         point.y() < max.y() && point.y() > min.y();
}

int main(int argc, char **argv)
{
  #ifdef MOVEL_LICENSE                                                                                                    
  MovelLicense ml(23);                                                                                                   
  if (!ml.login())                                                                                                      
    return 1;                                                                                                           
  #endif

  ros::init(argc, argv, "rack_filter");
  laser_filters::RackFilter node;
  #ifdef MOVEL_LICENSE
  ml.logout();
  #endif
  return 0;
}
