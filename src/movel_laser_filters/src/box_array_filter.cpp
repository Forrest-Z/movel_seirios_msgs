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

#include "movel_laser_filters/box_array_filter.h"
#include <ros/ros.h>

typedef std::vector<tf::Point> Box;

movel_laser_filters::LaserScanBoxArrayFilter::LaserScanBoxArrayFilter(){}

bool movel_laser_filters::LaserScanBoxArrayFilter::configure(){
  up_and_running_ = true;

  //[1.0.2]
  ros::NodeHandle private_nh("~" + getName()); //create private namespace
  dyn_server_.reset(new dynamic_reconfigure::Server<movel_laser_filters::BoxArrayFilterConfig>(own_mutex_, private_nh)); 
  dynamic_reconfigure::Server<movel_laser_filters::BoxArrayFilterConfig>::CallbackType f; 
  f = boost::bind(&movel_laser_filters::LaserScanBoxArrayFilter::reconfigureCB, this, _1, _2 ); 
  dyn_server_ -> setCallback(f); 

  bool box_frame_set = getParam("box_frame", box_frame_); 
  bool box_array_set = getParam("box_array", box_array_xmlrpc);
  bool invert_set = getParam("invert", invert_filter);
  makeBoxesFromXMLRPC(box_array_xmlrpc) ;
  
  ROS_INFO("Box array filter started");
  
  if(!box_frame_set){
    ROS_ERROR("box_frame is not set!");
  }
  if(!box_array_set){
    ROS_ERROR("box_array is not set!");
  }
  if(!invert_set){
    ROS_INFO("invert filter not set, assuming false");
    invert_filter=false;
  }

  param_config.box_array = boxesToString();
  param_config.invert = invert_filter;
  if(getParam("switch_",switch_)) { //[1.0.2] Added switch to turn filter on/off
    param_config.switch_ = switch_;
  }
  dyn_server_->updateConfig(param_config);

  //[1.0.2] Added polygon publisher
  // polygon_pub_ = private_nh.advertise<geometry_msgs::PolygonStamped>("box_polygon", 1);

  return box_frame_set && box_array_set;
}

std::string movel_laser_filters::LaserScanBoxArrayFilter::boxesToString()
{
  std::string polygon_string = "[";
  bool first = true;
  for (Box box : boxes_) {
    if (!first) 
    {
      polygon_string += ", ";
    }
    first = false;
    polygon_string += "[" + std::to_string(box[0].x()) + ", " + std::to_string(box[0].y()) + "]";
    polygon_string += ", ";
    polygon_string += "[" + std::to_string(box[1].x()) + ", " + std::to_string(box[1].y()) + "]";
  }
  polygon_string += "]";
  return polygon_string;
}

void movel_laser_filters::LaserScanBoxArrayFilter::makeBoxesFromXMLRPC(XmlRpc::XmlRpcValue box_array_xmlrpc_) 
{
  boxes_.clear();
  //parse the array
  if (box_array_xmlrpc_.getType() != XmlRpc::XmlRpcValue::TypeArray ||
      box_array_xmlrpc_.size() == 1)
  {
    ROS_ERROR("Please provide at least two coordinates to form a box for box filter array");
    throw std::runtime_error("Please provide at least two coordinates to form a box for box filter array");
  }
  tf::Point min;
  tf::Point max;
  //make a vector of boxes
  for (int i=0; i<box_array_xmlrpc.size(); i++)
  {
    XmlRpc::XmlRpcValue point = box_array_xmlrpc_[i];
    if (i%2 == 0)
    {
      min.setX(getNumberFromXMLRPC(point[0]));
      min.setY(getNumberFromXMLRPC(point[1])); 
    }
    else 
    {
      max.setX(getNumberFromXMLRPC(point[0]));
      max.setY(getNumberFromXMLRPC(point[1])); 
      Box box({min, max});
      boxes_.push_back(box);
    }
  }
}

double movel_laser_filters::LaserScanBoxArrayFilter::getNumberFromXMLRPC(XmlRpc::XmlRpcValue& value)
{
  // Make sure that the value we're looking at is either a double or an int.
  if (value.getType() != XmlRpc::XmlRpcValue::TypeInt && value.getType() != XmlRpc::XmlRpcValue::TypeDouble)
  {
    std::string& value_string = value;
    ROS_FATAL("Values in the box array specification must be numbers. Found value %s.", value_string.c_str());
    throw std::runtime_error("Values in the box array specification must be numbers");
  }
  return value.getType() == XmlRpc::XmlRpcValue::TypeInt ? (int)(value) : (double)(value);
}

void movel_laser_filters::LaserScanBoxArrayFilter::makeBoxesFromString(const std::string& box_array_string)
{
  std::string error;
  std::vector<std::vector<float> > box_vector = parseVVF(box_array_string, error);

    if (error != "")
    {
      ROS_ERROR("Error parsing box parameter: '%s'", error.c_str());
      ROS_ERROR("box string was '%s'.", box_array_string.c_str());
      return;
    }

    geometry_msgs::Polygon polygon;
    geometry_msgs::Point32 point;

    // convert box_vector into points.
    if (box_vector.size() == 1 || box_vector.size()%2 != 0 )
    {
      ROS_WARN("You must specify a pair of point coordinates for each box");
      return;
    }

  tf::Point min;
  tf::Point max;
  boxes_.clear();
  for (int i=0; i<box_vector.size(); i++)
  {
    if (box_vector[ i ].size() == 2)
    {
      if (i%2 == 0)
      {
        min.setX(box_vector[ i ][ 0 ]);
        min.setY(box_vector[ i ][ 1 ]); 
      }
      else 
      {
        max.setX(box_vector[ i ][ 0 ]);
        max.setY(box_vector[ i ][ 1 ]); 
        Box box({min, max});
        boxes_.push_back(box);
      }
    }
    else
    {
      ROS_ERROR("Points in the box specification must be pairs of numbers. Found a point with %d numbers.",
                  int(box_vector[ i ].size()));
      return;
    }
  }
}

std::vector<std::vector<float> > movel_laser_filters::LaserScanBoxArrayFilter::parseVVF(const std::string& input, std::string& error_return)
{  // Source: https://github.com/ros-planning/navigation/blob/melodic-devel/costmap_2d/src/array_parser.cpp
  std::vector<std::vector<float> > result;

  std::stringstream input_ss(input);
  int depth = 0;
  std::vector<float> current_vector;
  while (!!input_ss && !input_ss.eof())
  {
    switch (input_ss.peek())
    {
    case EOF:
      break;
    case '[':
      depth++;
      if (depth > 2)
      {
        error_return = "Array depth greater than 2";
        return result;
      }
      input_ss.get();
      current_vector.clear();
      break;
    case ']':
      depth--;
      if (depth < 0)
      {
        error_return = "More close ] than open [";
        return result;
      }
      input_ss.get();
      if (depth == 1)
      {
        result.push_back(current_vector);
      }
      break;
    case ',':
    case ' ':
    case '\t':
      input_ss.get();
      break;
    default:  // All other characters should be part of the numbers.
      if (depth != 2)
      {
        std::stringstream err_ss;
        err_ss << "Numbers at depth other than 2. Char was '" << char(input_ss.peek()) << "'.";
        error_return = err_ss.str();
        return result;
      }
      float value;
      input_ss >> value;
      if (!!input_ss)
      {
        current_vector.push_back(value);
      }
      break;
    }
  }

  if (depth != 0)
  {
    error_return = "Unterminated vector string.";
  }
  else
  {
    error_return = "";
  }

  return result;
}

void movel_laser_filters::LaserScanBoxArrayFilter::reconfigureCB(movel_laser_filters::BoxArrayFilterConfig &config, uint32_t level) 
{ 
  makeBoxesFromString(config.box_array);
  invert_filter = config.invert;
  switch_ = config.switch_;
  //TO DO: add polygon msg
}

bool movel_laser_filters::LaserScanBoxArrayFilter::update( const sensor_msgs::LaserScan& input_scan, sensor_msgs::LaserScan &output_scan)
{
  //[1.0.2]
  boost::recursive_mutex::scoped_lock lock(own_mutex_);

  //[1.0.2] Generate PolygonStamped msg and publish it
  // geometry_msgs::PolygonStamped polygon_stamped;
  // polygon_stamped.header.frame_id = polygon_frame_;
  // polygon_stamped.header.stamp = ros::Time::now();
  // polygon_stamped.polygon = polROS_INFOygon_;
  // polygon_pub_.publish(polygon_stamped);

  output_scan = input_scan;
  //[1.0.2] switch on/off filter
  if (!switch_)
  {
    up_and_running_ = true;
    return true;
  }
  
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
  for(i_idx = i_idx_offset, x_idx = x_idx_offset, y_idx = y_idx_offset, z_idx = z_idx_offset;
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

    //invert filter
    if(!invert_filter)
    {
      for (Box box : boxes_)
      {
        if(inBox(point, box))
        {
          output_scan.ranges[index] = std::numeric_limits<float>::quiet_NaN();
        }
      }
    }
    else
    {
      for (Box box : boxes_)
      {
        if(!inBox(point, box))
        {
          output_scan.ranges[index] = std::numeric_limits<float>::quiet_NaN();
        }
      }
    }

  }
  up_and_running_ = true;
  return true;
}

bool movel_laser_filters::LaserScanBoxArrayFilter::inBox(tf::Point &point, Box& box)
{
  tf::Point min = box[0];
  tf::Point max = box[1];
  return point.x() < max.x() && point.x() > min.x() &&
         point.y() < max.y() && point.y() > min.y();
}



//[1.0.2] Generate 3D point
// geometry_msgs::Point32 movel_laser_filters::LaserScanBoxArrayFilter::Point2D(float &&x, float &&y) 
// {
//   geometry_msgs::Point32 point;
//   point.x = x; point.y = y; 
//   return point;
// }

//[1.0.2] Generate 2D polygon message
// geometry_msgs::Polygon movel_laser_filters::LaserScanBoxArrayFilter::make2DPolygon() 
// {
//   geometry_msgs::Polygon polygon;
//   geometry_msgs::Point32 point;
//   //Top surface of box
//   point = Point3D( max_.x() , max_.y() , max_.z() ); //Point A 
//   polygon.points.push_back(point);
//   point = Point3D( min_.x() , max_.y() , max_.z() ); //Point B
//   polygon.points.push_back(point);
//   point = Point3D( min_.x() , min_.y() , max_.z() ); //C
//   polygon.points.push_back(point);
//   point = Point3D( max_.x() , min_.y() , max_.z() ); //D
//   polygon.points.push_back(point);
//   return polygon;
// }

