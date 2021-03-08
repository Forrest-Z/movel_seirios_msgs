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
 *  box_filter.h
 *
 *  author: John Tan <john@movel.ai>
 */
#ifndef BOXARRAYFILTER_H
#define BOXARRAYFILTER_H

#include <filters/filter_base.h>

#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <laser_geometry/laser_geometry.h>
#include <geometry_msgs/Polygon.h>
#include <geometry_msgs/PolygonStamped.h>

#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

#include <laser_filters/BoxArrayFilterConfig.h>
#include <dynamic_reconfigure/server.h>

namespace laser_filters
{
typedef std::vector<tf::Point> Box;
class LaserScanBoxArrayFilter : public filters::FilterBase<sensor_msgs::LaserScan>
{
  public:
    LaserScanBoxArrayFilter();
    bool configure();
    bool update(const sensor_msgs::LaserScan& input_scan, sensor_msgs::LaserScan& filtered_scan);
    double getNumberFromXMLRPC(XmlRpc::XmlRpcValue& value);
    std::vector<std::vector<float> > parseVVF(const std::string& input, std::string& error_return);
    void makeBoxesFromXMLRPC(XmlRpc::XmlRpcValue box_array_xmlrpc_);
    void makeBoxesFromString(const std::string& box_array_string);
    std::string boxesToString();

    //Dynamic reconfigure
    std::shared_ptr<dynamic_reconfigure::Server<laser_filters::BoxArrayFilterConfig>> dyn_server_;
    boost::recursive_mutex own_mutex_;
    BoxArrayFilterConfig param_config;
    void reconfigureCB(laser_filters::BoxArrayFilterConfig &config, uint32_t level);
    
    std::string polygon_frame_;
    ros::Publisher polygon_pub_;
    geometry_msgs::Polygon box_;
    // geometry_msgs::Polygon make2DPolygon(); //function to generate 2D polygon
    geometry_msgs::Point32 Point2D(float &&x, float &&y); //function to build 2D point

  private:
    std::string box_frame_;
    laser_geometry::LaserProjection projector_;

    tf::TransformListener tf_; // tf listener to transform scans into the box_frame 
    
    bool invert_filter;
    bool up_and_running_;
    bool switch_;       // Enable or disable filters

    std::vector<Box> boxes_;
    XmlRpc::XmlRpcValue box_array_xmlrpc;
    bool inBox(tf::Point &point,Box& box);
    
};
}

#endif /* box_filter.h */
