#include <ros_utils/ros_utils.h>

namespace ros_utils
{
ParamLoader::ParamLoader(const ros::NodeHandle& nh) : all_params_valid_(true)
{
  nh_ = nh;
}

bool ParamLoader::params_valid()
{
  return all_params_valid_;
}

}  // namespace ros_utils
