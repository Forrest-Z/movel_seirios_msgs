#include <ros_utils/ros_utils.h>

namespace ros_utils
{
ParamLoader::ParamLoader() : nh_("~"), all_params_valid_(true)
{
  ROS_WARN_STREAM("No NodeHandle specified. Loading params from private namespace.");
}

ParamLoader::ParamLoader(const ros::NodeHandle& nh) : all_params_valid_(true)
{
  nh_ = nh;
}

bool ParamLoader::params_valid()
{
  return all_params_valid_;
}

}  // namespace ros_utils
