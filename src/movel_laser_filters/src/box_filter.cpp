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
 *  box_filter.cpp
 *
 *  author: Sebastian Pütz <spuetz@uni-osnabrueck.de>
 */

#include "movel_laser_filters/box_filter.h"
#include <ros/ros.h>

movel_laser_filters::LaserScanBoxFilter::LaserScanBoxFilter(){}

bool movel_laser_filters::LaserScanBoxFilter::configure(){
  up_and_running_ = true;
  double min_x = 0, min_y = 0, min_z = 0, max_x = 0, max_y = 0, max_z = 0;
  // switch_ = true; //[1.0.2] filter switch

  //[1.0.2]
  ros::NodeHandle private_nh("~" + getName()); //create private namespace
  dyn_server_.reset(new dynamic_reconfigure::Server<movel_laser_filters::BoxFilterConfig>(own_mutex_, private_nh)); 
  dynamic_reconfigure::Server<movel_laser_filters::BoxFilterConfig>::CallbackType f; 
  f = boost::bind(&movel_laser_filters::LaserScanBoxFilter::reconfigureCB, this, _1, _2 ); 
  dyn_server_ -> setCallback(f); 

  bool box_frame_set = getParam("box_frame", box_frame_); 
  bool x_max_set = getParam("max_x", max_x);
  bool y_max_set = getParam("max_y", max_y);
  bool z_max_set = getParam("max_z", max_z);
  bool x_min_set = getParam("min_x", min_x);
  bool y_min_set = getParam("min_y", min_y);
  bool z_min_set = getParam("min_z", min_z);
  bool invert_set = getParam("invert", invert_filter);
  
  ROS_INFO("Box filter(%s) started", getName().c_str());

  max_.setX(max_x); max_.setY(max_y); max_.setZ(max_z);
  min_.setX(min_x); min_.setY(min_y); min_.setZ(min_z);
  
  if(!box_frame_set){
    ROS_ERROR("box_frame is not set!");
  }
  if(!x_max_set){
    ROS_ERROR("max_x is not set!");
  }
  if(!y_max_set){
    ROS_ERROR("max_y is not set!");
  }
  if(!z_max_set){
    ROS_ERROR("max_z is not set!");
  }
  if(!x_min_set){
    ROS_ERROR("min_x is not set!");
  }
  if(!y_min_set){
    ROS_ERROR("min_y is not set!");
  }
  if(!z_min_set){
    ROS_ERROR("min_z is not set!");
  }
  if(!invert_set){
    ROS_INFO("invert filter not set, assuming false");
    invert_filter=false;
  }

  //[1.0.2] Generate 3D polygon for polygonstamped msg
  polygon_ = make3DPolygon();
  polygon_frame_ = box_frame_;

  //[1.0.2] update dynamic config parameters
  param_config.min_x = min_x;
  param_config.min_y = min_y;
  param_config.min_z = min_z;
  param_config.max_x = max_x;
  param_config.max_y = max_y;
  param_config.max_z = max_z;
  param_config.invert = invert_filter;
  if(getParam("switch_",switch_)) { //[1.0.2] Added switch to turn filter on/off
    param_config.switch_ = switch_;
  }
  dyn_server_->updateConfig(param_config);

  //[1.0.2] Added polygon publisher
  polygon_pub_ = private_nh.advertise<geometry_msgs::PolygonStamped>("box_polygon", 1);

  return box_frame_set && x_max_set && y_max_set && z_max_set &&
    x_min_set && y_min_set && z_min_set;
}

//[1.0.2] Generate 3D point
geometry_msgs::Point32 movel_laser_filters::LaserScanBoxFilter::Point3D(float &&x, float &&y, float &&z) 
{
  geometry_msgs::Point32 point;
  point.x = x; point.y = y; point.z = z; 
  return point;
}

//[1.0.2] Generate 2D polygon message
geometry_msgs::Polygon movel_laser_filters::LaserScanBoxFilter::make2DPolygon() 
{
  geometry_msgs::Polygon polygon;
  geometry_msgs::Point32 point;
  //Top surface of box
  point = Point3D( max_.x() , max_.y() , max_.z() ); //Point A 
  polygon.points.push_back(point);
  point = Point3D( min_.x() , max_.y() , max_.z() ); //Point B
  polygon.points.push_back(point);
  point = Point3D( min_.x() , min_.y() , max_.z() ); //C
  polygon.points.push_back(point);
  point = Point3D( max_.x() , min_.y() , max_.z() ); //D
  polygon.points.push_back(point);
  return polygon;
}

//[1.0.2] Generate 3D wiremesh of box
geometry_msgs::Polygon movel_laser_filters::LaserScanBoxFilter::make3DPolygon() 
{
  geometry_msgs::Polygon polygon;
  geometry_msgs::Point32 point;

  //Top surface of box
  point = Point3D( max_.x() , max_.y() , max_.z() ); //Point A 
  polygon.points.push_back(point);
  point = Point3D( min_.x() , max_.y() , max_.z() ); //Point B
  polygon.points.push_back(point);
  point = Point3D( min_.x() , min_.y() , max_.z() ); //C
  polygon.points.push_back(point);
  point = Point3D( max_.x() , min_.y() , max_.z() ); //D
  polygon.points.push_back(point);
  //Side of box
  point = Point3D( max_.x() , min_.y() , min_.z() ); //E
  polygon.points.push_back(point);
  point = Point3D( max_.x() , max_.y() , min_.z() ); //F
  polygon.points.push_back(point);
  point = Point3D( max_.x() , max_.y() , max_.z() ); //A 
  polygon.points.push_back(point);
  point = Point3D( max_.x() , max_.y() , min_.z() ); //F
  polygon.points.push_back(point);
  point = Point3D( min_.x() , max_.y() , min_.z() ); //G
  polygon.points.push_back(point);
  point = Point3D( min_.x() , max_.y() , max_.z() ); //B
  polygon.points.push_back(point);
  point = Point3D( min_.x() , max_.y() , min_.z() ); //G
  polygon.points.push_back(point);
  point = Point3D( min_.x() , min_.y() , min_.z() ); //H
  polygon.points.push_back(point);
  point = Point3D( min_.x() , min_.y() , max_.z() ); //C
  polygon.points.push_back(point);
  point = Point3D( min_.x() , min_.y() , min_.z() ); //H
  polygon.points.push_back(point);
  point = Point3D( max_.x() , min_.y() , min_.z() ); //E
  polygon.points.push_back(point);
  point = Point3D( max_.x() , min_.y() , max_.z() ); //D
  polygon.points.push_back(point);

  return polygon;
}

//[1.0.2] Added Dynamic Reconfigure callback
void movel_laser_filters::LaserScanBoxFilter::reconfigureCB(movel_laser_filters::BoxFilterConfig &config, uint32_t level) 
{ 
  min_.setX(config.min_x);
  min_.setY(config.min_y);
  min_.setZ(config.min_z);
  max_.setX(config.max_x);
  max_.setY(config.max_y);
  max_.setZ(config.max_z);
  invert_filter = param_config.invert;
  switch_ = config.switch_;
  polygon_ = make3DPolygon();
}

bool movel_laser_filters::LaserScanBoxFilter::update( const sensor_msgs::LaserScan& input_scan, sensor_msgs::LaserScan &output_scan)
{
  //[1.0.2]
  boost::recursive_mutex::scoped_lock lock(own_mutex_);

  //[1.0.2] Generate PolygonStamped msg and publish it
  geometry_msgs::PolygonStamped polygon_stamped;
  polygon_stamped.header.frame_id = polygon_frame_;
  polygon_stamped.header.stamp = ros::Time::now();
  polygon_stamped.polygon = polygon_;
  polygon_pub_.publish(polygon_stamped);

  //[1.0.2] switch on/off filter
  if (!switch_)
  {
    output_scan = input_scan;
    up_and_running_ = true;
    return true;
  }

  output_scan = input_scan;
  sensor_msgs::PointCloud2 laser_cloud;
  
  std::string error_msg;



  bool success = tf_.waitForTransform(
    box_frame_,
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

  try{
    projector_.transformLaserScanToPointCloud(box_frame_, input_scan, laser_cloud, tf_);
  }
  catch(tf::TransformException& ex){
    if(up_and_running_){
      ROS_WARN_THROTTLE(1, "Dropping Scan: Tansform unavailable %s", ex.what());
      return true;
    }
    else
    {
      ROS_INFO_THROTTLE(.3, "Ignoring Scan: Waiting for TF");
    }
    return false;
  }
  const int i_idx_c = sensor_msgs::getPointCloud2FieldIndex(laser_cloud, "index");
  const int x_idx_c = sensor_msgs::getPointCloud2FieldIndex(laser_cloud, "x");
  const int y_idx_c = sensor_msgs::getPointCloud2FieldIndex(laser_cloud, "y");
  const int z_idx_c = sensor_msgs::getPointCloud2FieldIndex(laser_cloud, "z");

  if(i_idx_c == -1 || x_idx_c == -1 || y_idx_c == -1 || z_idx_c == -1){
      ROS_INFO_THROTTLE(.3, "x, y, z and index fields are required, skipping scan");

  }

  const int i_idx_offset = laser_cloud.fields[i_idx_c].offset;
  const int x_idx_offset = laser_cloud.fields[x_idx_c].offset;
  const int y_idx_offset = laser_cloud.fields[y_idx_c].offset;
  const int z_idx_offset = laser_cloud.fields[z_idx_c].offset;

  const int pstep = laser_cloud.point_step;
  const long int pcount = laser_cloud.width * laser_cloud.height;
  const long int limit = pstep * pcount;

  int i_idx, x_idx, y_idx, z_idx;  
  for(
    i_idx = i_idx_offset,
    x_idx = x_idx_offset,
    y_idx = y_idx_offset,
    z_idx = z_idx_offset;

    x_idx < limit;

    i_idx += pstep,
    x_idx += pstep,
    y_idx += pstep,
    z_idx += pstep)
  {

    // TODO works only for float data types and with an index field
    // I'm working on it, see https://github.com/ros/common_msgs/pull/78 
    float x = *((float*)(&laser_cloud.data[x_idx]));
    float y = *((float*)(&laser_cloud.data[y_idx]));
    float z = *((float*)(&laser_cloud.data[z_idx]));
    int index = *((int*)(&laser_cloud.data[i_idx]));

    tf::Point point(x, y, z);

    if(!invert_filter){
      if(inBox(point)){
        output_scan.ranges[index] = std::numeric_limits<float>::quiet_NaN();
      }
    }
    else{
      if(!inBox(point)){
        output_scan.ranges[index] = std::numeric_limits<float>::quiet_NaN();
      }
    }

  }
  up_and_running_ = true;
  return true;
}

bool movel_laser_filters::LaserScanBoxFilter::inBox(tf::Point &point){
  return
    point.x() < max_.x() && point.x() > min_.x() && 
    point.y() < max_.y() && point.y() > min_.y() &&
    point.z() < max_.z() && point.z() > min_.z();
}

