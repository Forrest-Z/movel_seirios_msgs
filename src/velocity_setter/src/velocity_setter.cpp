#include <velocity_setter/velocity_setter.h>

// Assign velocity
bool VelocitySetter::setVelocity(double velocity)
{
  dynamic_reconfigure::Reconfigure reconfigure;
  dynamic_reconfigure::DoubleParameter velocity_param;
  velocity_param.name = parameter_name_linear_;
  velocity_param.value = velocity;
  reconfigure.request.config.doubles.push_back(velocity_param);

  if(set_client_.call(reconfigure))
  {
    return true;
  }
  else
  {
    return false;
  }
}


bool VelocitySetter::setSpeed(double v_linear, double v_angular)
{
  dynamic_reconfigure::Reconfigure reconfigure;
  dynamic_reconfigure::DoubleParameter velocity_param_linear, velocity_param_angular;
  
  velocity_param_linear.name = parameter_name_linear_;
  velocity_param_linear.value = v_linear;
  velocity_param_angular.name = parameter_name_angular_;
  velocity_param_angular.value = v_angular;
  reconfigure.request.config.doubles.push_back(velocity_param_linear);
  reconfigure.request.config.doubles.push_back(velocity_param_angular);

  bool success = set_client_.call(reconfigure);
  return success;
}


// Callback of set_velocity service
bool VelocitySetter::onSetVelocity(movel_seirios_msgs::SetVelocity::Request &req, 
                                   movel_seirios_msgs::SetVelocity::Response &res)
{
  double velocity;

  // If user input velocity name exists in loaded parameters from config file
  if(velocities_.find(req.name) != velocities_.end())
  {
    velocity = velocities_[req.name];
    if(setVelocity(velocity))
    {
      ROS_INFO("[velocity_setter] Set to '%s' velocity of %s ", req.name.c_str(), local_planner_.c_str());
      res.success = true;
    }
    else
    {
      ROS_ERROR("[velocity_setter] Failed to call '/move_base/%s/set_parameters' service", local_planner_.c_str());
      res.success = false;
    }
  }
  else
  {
    ROS_ERROR("[velocity_setter] Velocity value for '%s' was not set", req.name.c_str());
    res.success = false;
  }
  return true;
}


bool VelocitySetter::onSetSpeed(movel_seirios_msgs::SetSpeed::Request &req, 
                                movel_seirios_msgs::SetSpeed::Response &res)
{
  if(setSpeed(req.linear, req.angular)) {
    ROS_INFO("[velocity_setter] The linear velocity has been set to: %.2f", req.linear);
    ROS_INFO("[velocity_setter] The angular velocity has been set to: %.2f", req.angular);
    last_set_linear_ = req.linear;
    last_set_angular_ = req.angular;
    res.success = true;
  }
  else {
    ROS_ERROR("[velocity_setter] Failed to call '/move_base/%s/set_parameters' service", local_planner_.c_str());
    res.success = false;
  }
  return true;
}


bool VelocitySetter::onGetSpeed(movel_seirios_msgs::GetSpeed::Request &req, 
                                movel_seirios_msgs::GetSpeed::Response &res)
{
  if(last_set_linear_ == 0.0 || last_set_angular_ == 0.0) {
    ROS_INFO("[velocity_setter] No cached velocities to get");
    res.success = false;
  }
  else {
    ROS_INFO("[velocity_setter] Get cached linear velocity: %.2f", last_set_linear_);
    ROS_INFO("[velocity_setter] Get cached angular velocity: %.2f", last_set_angular_);
    res.linear = last_set_linear_;
    res.angular = last_set_angular_;
    res.success = true;
  }
  return true;
}