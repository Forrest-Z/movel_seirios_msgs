#include "ros/ros.h"
#include <dynamic_reconfigure/DoubleParameter.h>
#include <dynamic_reconfigure/IntParameter.h>
#include <dynamic_reconfigure/BoolParameter.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <cstdlib>
#include <std_srvs/SetBool.h>


ros::ServiceClient set_pebble_params_;
ros::ServiceServer enable_plan_;
bool enableCb(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res)
{
  if(req.data )
    res.message = "stop_feature  enabled";
  else if(!req.data)
    res.message = "stop_feature  disabled";
  
  dynamic_reconfigure::Reconfigure reconfigure_common;
  dynamic_reconfigure::BoolParameter set_obs_check;
  set_obs_check.name = "enable_obsctacle_check";
  set_obs_check.value = req.data;
  reconfigure_common.request.config.bools.push_back(set_obs_check);

  if(set_pebble_params_.call(reconfigure_common))
    { ROS_INFO("Pebble plan set params..."); }
  else{
    ROS_ERROR("Failed Pebble plan set params...");
    return false;
  }


  res.success = true;
  return true;

} 

int main(int argc, char **argv)
{
  ros::init(argc, argv, "obstacle");
  ros::NodeHandle nh_;
  enable_plan_ = nh_.advertiseService("enable_plan_inspector", enableCb);
  set_pebble_params_ = nh_.serviceClient<dynamic_reconfigure::Reconfigure>("/move_base/PebbleLocalPlanner/set_parameters");
  
  ros::spin();
  return 0;
}