/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2009, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of Willow Garage, Inc. nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*
* Author: Kevin Hallenbeck
*********************************************************************/
#ifndef LASER_SCAN_ANGULAR_BOUNDS_FILTER_IN_PLACE_H
#define LASER_SCAN_ANGULAR_BOUNDS_FILTER_IN_PLACE_H

#include <filters/filter_base.h>
#include <sensor_msgs/LaserScan.h>

//[1.0.2]
#include <movel_laser_filters/AngularBoundsFilterInPlaceConfig.h>
#include <dynamic_reconfigure/server.h>

namespace movel_laser_filters
{
  class LaserScanAngularBoundsFilterInPlace : public filters::FilterBase<sensor_msgs::LaserScan>
  {
    public:
      double lower_angle_;
      double upper_angle_;
      bool switch_; //[1.0.2] added switch_ variable for turning the filters on and off 
      
      //[1.0.2] Added dynamic reconfigure server
      std::shared_ptr<dynamic_reconfigure::Server<movel_laser_filters::AngularBoundsFilterInPlaceConfig>> dyn_server_;
      boost::recursive_mutex own_mutex_;
      AngularBoundsFilterInPlaceConfig param_config;

      bool configure()
      {
        lower_angle_ = 0;
        upper_angle_ = 0;

        //[1.0.2]
        ros::NodeHandle private_nh("~" + getName());
        dyn_server_.reset(new dynamic_reconfigure::Server<movel_laser_filters::AngularBoundsFilterInPlaceConfig>(own_mutex_, private_nh));
        dynamic_reconfigure::Server<movel_laser_filters::AngularBoundsFilterInPlaceConfig>::CallbackType f;
        f = boost::bind(&movel_laser_filters::LaserScanAngularBoundsFilterInPlace::reconfigureCB, this, _1, _2 ); //bind reconfigure callback
        dyn_server_ -> setCallback(f);

        if(!getParam("lower_angle", lower_angle_) || !getParam("upper_angle", upper_angle_)){
          ROS_ERROR("Both the lower_angle and upper_angle parameters must be set to use this filter.");
          return false;
        }

        param_config.lower_angle = lower_angle_;
        param_config.upper_angle = upper_angle_;
        if(getParam("switch_",switch_)) { //[1.0.2] Added switch to turn filter on/off
          param_config.switch_ = switch_;
        }
        dyn_server_->updateConfig(param_config);

        ROS_INFO("Angular bounds filter in place started");

        return true;
      }

      //[1.0.2] Added Dynamic Reconfigure callback
      void reconfigureCB(AngularBoundsFilterInPlaceConfig &config, uint32_t level) 
      {
        lower_angle_ = config.lower_angle;
        upper_angle_ = config.upper_angle;
        switch_ = config.switch_;
      }

      virtual ~LaserScanAngularBoundsFilterInPlace(){}

      bool update(const sensor_msgs::LaserScan& input_scan, sensor_msgs::LaserScan& filtered_scan)
      {
        //[1.0.2] filter switch
        if (!switch_)
        {
          filtered_scan = input_scan;
          return true;
        }

        //[1.0.2]
        boost::recursive_mutex::scoped_lock lock(own_mutex_);

        filtered_scan = input_scan; //copy entire message

        double current_angle = input_scan.angle_min;
        unsigned int count = 0;
        //loop through the scan and remove ranges at angles between lower_angle_ and upper_angle_
        for(unsigned int i = 0; i < input_scan.ranges.size(); ++i){
          if((current_angle > lower_angle_) && (current_angle < upper_angle_)){
            filtered_scan.ranges[i] = input_scan.range_max + 1.0;
            if(i < filtered_scan.intensities.size()){
              filtered_scan.intensities[i] = 0.0;
            }
            count++;
          }
          current_angle += input_scan.angle_increment;
        }

        ROS_DEBUG("Filtered out %u points from the laser scan.", count);

        return true;

      }
  };
};
#endif
