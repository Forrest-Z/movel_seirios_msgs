#include <cstdlib>
#include <speed_limit_zone/throttle_speed.h>
#include <movel_hasp_vendor/license.h>

ThrottleSpeed::ThrottleSpeed() {
  if (!setupTopics()) {
    ROS_ERROR("failed to setup topics");
  return;
  }
}

bool ThrottleSpeed::setupParams() {
  nh.getParam("/move_base/base_local_planner", local_planner);
  //x.substr(x.find(":") + 1);
  // actual param is e.g. teb_local_planner/TebLocalPlannerROS
  // but i only want TebLocalPlannerROS
  local_planner = local_planner.substr(local_planner.find("/")+1);  
  move_base_name = "/move_base/" + local_planner;
  linear_topic = move_base_name + "/max_vel_x"; // linear.x
  angular_topic = move_base_name + "/max_vel_theta"; // angular.z
  //ROS_INFO("Params are %s, %s, %s", local_planner.c_str(), move_base_name.c_str(), linear_topic.c_str());

  nh.getParam(linear_topic, linear_speed_default);
  nh.getParam(angular_topic, angular_speed_default);
  ROS_INFO("Default speeds are linear: %f, angular: %f", linear_speed_default, angular_speed_default);
  return true;
}

bool ThrottleSpeed::setupTopics() {
  speed_limiter_serv_ = nh.advertiseService("limit_robot_speed", &ThrottleSpeed::onThrottleSpeed, this);
  set_throttled_speed = nh.serviceClient<dynamic_reconfigure::Reconfigure>(move_base_name + "/set_parameters");
  return true;
}

bool ThrottleSpeed::onThrottleSpeed(movel_seirios_msgs::ThrottleSpeed::Request& req, movel_seirios_msgs::ThrottleSpeed::Response& res) {
  bool should_limit_speed = req.set_throttle;
  if(should_limit_speed){
    double throttle_percentage = req.percentage;
    reconfigureSpeed(true, throttle_percentage);
    res.success = true;
    res.message = "Speed reduced by " + std::to_string(throttle_percentage); 
  }
  else {
    reconfigureSpeed(false);
    res.success = false;
    res.message = "Speed limiter off";
  }
  return true;
}

bool ThrottleSpeed::reconfigureSpeed(bool should_reconfigure, double percentage) {
  dynamic_reconfigure::Reconfigure reconfigure_speeds;
  dynamic_reconfigure::DoubleParameter linear_x, angular_z;

  linear_x.name = "max_vel_x";
  angular_z.name = "max_vel_theta";

  if(should_reconfigure) {
    linear_x.value = percentage * linear_speed_default;
    angular_z.value = percentage * angular_speed_default;
  }
  else {
    linear_x.value = linear_speed_default;
    angular_z.value = angular_speed_default;
  }

  reconfigure_speeds.request.config.doubles.push_back(linear_x);
  reconfigure_speeds.request.config.doubles.push_back(angular_z);

  if(set_throttled_speed.call(reconfigure_speeds)) {
    // debug prints
    double new_linear_x, new_angular_z;
    nh.getParam(linear_topic, new_linear_x);
    nh.getParam(angular_topic, new_angular_z);
    ROS_INFO("New speeds are linear: %f, angular: %f", new_linear_x, new_angular_z);
  }
  else {
    return false;
  }
  return true;
}

int main(int argc, char **argv) {
  #ifdef MOVEL_LICENSE
    MovelLicense ml;
    if (!ml.login())
      return 1;
  #endif

  ros::init(argc, argv, "speed_limit_zones");
  
  ThrottleSpeed t_spd;
  ros::spin();
 
  #ifdef MOVEL_LICENSE
    ml.logout();
  #endif

  return 0;
}