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
 *  circular_filter.h
 *
 *  author: Nursharmin Ramli <nursharminramli@gmail.com>
 *  adapted from box_filter.h
 */

#ifndef CIRCULARFILTER_H
#define CIRCULARFILTER_H

#include <filters/filter_base.h>

#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <laser_geometry/laser_geometry.h>

#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

#include <laser_filters/CircularFilterConfig.h>
#include <dynamic_reconfigure/server.h>

namespace laser_filters
{
/**
 * @brief This is a filter that removes points in a laser scan inside of a circular region.
 */
class LaserScanCircularFilter : public filters::FilterBase<sensor_msgs::LaserScan>
{
  public:
    LaserScanCircularFilter();

    //[1.0.2]
    std::shared_ptr<dynamic_reconfigure::Server<laser_filters::CircularFilterConfig>> dyn_server_;
    boost::recursive_mutex own_mutex_;
    CircularFilterConfig param_config;

    void reconfigureCB(laser_filters::CircularFilterConfig &config, uint32_t level);

    bool configure();

    bool update(
      const sensor_msgs::LaserScan& input_scan,
      sensor_msgs::LaserScan& filtered_scan);

  private:
    bool inCircle(tf::Point &point);
    std::string circle_frame_, segment_;
    laser_geometry::LaserProjection projector_;

    // tf listener to transform scans into the circle_frame
    tf::TransformListener tf_;

    // defines the center of the circular region
    tf::Point center_;
    // defines the radius of the circular region
    double radius_squared_;
    bool up_and_running_;
};
}
#endif /* circular_filter.h */
