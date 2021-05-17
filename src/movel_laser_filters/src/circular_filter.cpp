/*
 *  Software License Agreement (BSD License)
 *
 *  Robot Operating System code by the University of Osnabrück
 *  Copyright (c) 2015, University of Osnabrück
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   1. Redistributions of source code must retain the above
 *      copyright notice, this list of conditions and the following
 *      disclaimer.
 *
 *   2. Redistributions in binary form must reproduce the above
 *      copyright notice, this list of conditions and the following
 *      disclaimer in the documentation and/or other materials provided
 *      with the distribution.
 *
 *   3. Neither the name of the copyright holder nor the names of its
 *      contributors may be used to endorse or promote products derived
 *      from this software without specific prior written permission.
 *
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
 *  TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 *  PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 *  CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 *  EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 *  PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 *  OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 *  WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 *  OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
 *  ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *
 *
 *  circular_filter.cpp
 *
 *  author: Nursharmin Ramli <nursharminramli@gmail.com>
 *  adapted from circular_filter.cpp
 */

#include "movel_laser_filters/circular_filter.h"
#include <ros/ros.h>

movel_laser_filters::LaserScanCircularFilter::LaserScanCircularFilter(){}

bool movel_laser_filters::LaserScanCircularFilter::configure()
{
  up_and_running_ = true;

  //[1.0.2]
  ros::NodeHandle private_nh("~" + getName()); //create private namespace
  dyn_server_.reset(new dynamic_reconfigure::Server<movel_laser_filters::CircularFilterConfig>(own_mutex_, private_nh)); 
  dynamic_reconfigure::Server<movel_laser_filters::CircularFilterConfig>::CallbackType f; 
  f = boost::bind(&movel_laser_filters::LaserScanCircularFilter::reconfigureCB, this, _1, _2 ); 
  dyn_server_ -> setCallback(f); 

  bool circle_frame_set = getParam("circle_frame", circle_frame_);
  bool segment_set = getParam("segment", segment_);

  double circle_x, circle_y, circle_r;
  bool circle_x_set = getParam("circle_x", circle_x);
  bool circle_y_set = getParam("circle_y", circle_y);
  bool circle_r_set = getParam("circle_r", circle_r);

  center_.setX(circle_x);
  center_.setY(circle_y);
  radius_squared_ = pow(circle_r, 2.0);

  if(!circle_frame_set){
    ROS_ERROR("circle_frame is not set!");
  }
  if(!segment_set){
    ROS_ERROR("segment is not set!");
  }
  if(!circle_x_set){
    ROS_ERROR("circle_x is not set!");
  }
  if(!circle_y_set){
    ROS_ERROR("circle_y is not set!");
  }
  if(!circle_r_set){
    ROS_ERROR("circle_r is not set!");
  }

  param_config.segment = segment_;
  param_config.circle_x = circle_x;
  param_config.circle_y = circle_y;
  param_config.circle_r = circle_r;
  dyn_server_->updateConfig(param_config);

  return circle_frame_set && segment_set &&
    circle_x_set && circle_y_set && circle_r_set;
}

//[1.0.2] Added Dynamic Reconfigure callback
void movel_laser_filters::LaserScanCircularFilter::reconfigureCB(movel_laser_filters::CircularFilterConfig &config, uint32_t level) 
{ 
  center_.setX(config.circle_x);
  center_.setY(config.circle_y);
  radius_squared_ = pow(config.circle_r , 2.0);
  segment_ = config.segment ;
}

bool movel_laser_filters::LaserScanCircularFilter::update(
    const sensor_msgs::LaserScan& input_scan,
    sensor_msgs::LaserScan &output_scan)
{
  //[1.0.2]
  boost::recursive_mutex::scoped_lock lock(own_mutex_);

  output_scan = input_scan;
  sensor_msgs::PointCloud2 laser_cloud;

  std::string error_msg;

  bool success = tf_.waitForTransform(
    circle_frame_,
    input_scan.header.frame_id,
    input_scan.header.stamp + ros::Duration().fromSec(input_scan.ranges.size()*input_scan.time_increment),
    ros::Duration(1.0),
    ros::Duration(0.01),
    &error_msg
  );
  if(!success){
    ROS_WARN("Could not get transform, ignoring laser scan! %s", error_msg.c_str());
    return false;
  }

  try
  {
    projector_.transformLaserScanToPointCloud(circle_frame_, input_scan, laser_cloud, tf_);
  }
  catch(tf::TransformException& ex)
  {
    if(up_and_running_)
    {
      ROS_WARN_THROTTLE(1, "Dropping Scan: Transform unavailable %s", ex.what());
      return true;
    }
    else
      ROS_INFO_THROTTLE(.3, "Ignoring Scan: Waiting for TF");

    return false;
  }
  const int i_idx_c = sensor_msgs::getPointCloud2FieldIndex(laser_cloud, "index");
  const int x_idx_c = sensor_msgs::getPointCloud2FieldIndex(laser_cloud, "x");
  const int y_idx_c = sensor_msgs::getPointCloud2FieldIndex(laser_cloud, "y");
  const int z_idx_c = sensor_msgs::getPointCloud2FieldIndex(laser_cloud, "z");

  if(i_idx_c == -1 || x_idx_c == -1 || y_idx_c == -1 || z_idx_c == -1)
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
    float x = *((float*)(&laser_cloud.data[x_idx]));
    float y = *((float*)(&laser_cloud.data[y_idx]));
    float z = *((float*)(&laser_cloud.data[z_idx]));
    int index = *((int*)(&laser_cloud.data[i_idx]));

    tf::Point point(x, y, z);

    if(segment_ == "inside")
    {
      if(!inCircle(point))
      {
        output_scan.ranges[index] = std::numeric_limits<float>::quiet_NaN();
      }
    }
    else if (segment_ == "outside")
    {
      if(inCircle(point))
      {
        output_scan.ranges[index] = std::numeric_limits<float>::quiet_NaN();
      }
    }
  }
  up_and_running_ = true;
  return true;
}

bool movel_laser_filters::LaserScanCircularFilter::inCircle(tf::Point &point)
{
  return
    pow(point.x() - center_.x(), 2.0) + pow(point.y() - center_.y(), 2.0) <= radius_squared_;
}

