/*
 *  USAGE:
 *  "/velocity_setter/set_velocity" service: Take velocity name (eg cruising,
 * cleaning) as input and set velocity according to config file
 */

#include <movel_hasp_vendor/license.h>
#include <ros_utils/ros_utils.h>
#include <velocity_setter/velocity_setter.h>
#include <string>


VelocitySetter setter;

// Load config file
bool loadParams(ros::NodeHandle &nh_private_) {
  ros_utils::ParamLoader loader(nh_private_);
  loader.get_required("move_base_params/base_local_planner", setter.local_planner_);
  loader.get_required("parameter_name_linear", setter.parameter_name_linear_);
  loader.get_required("parameter_name_angular", setter.parameter_name_angular_);
  std::string delimiter_1 = "::";
  std::string delimiter_2 = "/";

  if (setter.local_planner_.find(delimiter_1) != std::string::npos) {
      std::cout << "delimiter_1!" << '\n';
      setter.local_planner_ = setter.local_planner_.substr(setter.local_planner_.find(delimiter_1)+ delimiter_1.length());
      std::cout<< setter.local_planner_ ; 
  }
  else if (setter.local_planner_.find(delimiter_2) != std::string::npos){
      std::cout << "delimiter_2!" << '\n';
      setter.local_planner_ = setter.local_planner_.substr(setter.local_planner_.find(delimiter_2)+ delimiter_2.length());
      std::cout<< setter.local_planner_ ; 
  }

  std::cout<< setter.local_planner_ ; 
  std::map<std::string, double> params;
  nh_private_.getParam("velocities", params);
  if (params.size() == 0) {
    return false;
  }
  setter.velocities_ = params;

  if (nh_private_.hasParam("/move_base/" + setter.local_planner_ + "/" + setter.parameter_name_linear_))
    nh_private_.getParam("/move_base/" + setter.local_planner_ + "/" + setter.parameter_name_linear_, setter.task_linear_speed_);
  else
  {
    ROS_WARN("[velocity_setter] Could not get default linear speed, set to 0.2");
    setter.task_linear_speed_ = 0.2;
  }

  if (nh_private_.hasParam("/move_base/" + setter.local_planner_ + "/" + setter.parameter_name_angular_))
    nh_private_.getParam("/move_base/" + setter.local_planner_ + "/" + setter.parameter_name_angular_, setter.task_angular_speed_);
  else
  {
    ROS_WARN("[velocity_setter] Could not get default angular speed, set to 0.2");
    setter.task_angular_speed_ = 0.2;
  }

  return loader.params_valid();
}

int main(int argc, char **argv) {
  #ifdef MOVEL_LICENSE
    MovelLicense ml;
    if (!ml.login())
      return 1;
  #endif

  std::string node_name_ = "velocity_setter_node";
  ros::init(argc, argv, node_name_);

  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_("~");

  ros::Time::waitForValid();

  if (!loadParams(nh_private_)) {
    ROS_FATAL(
        "[velocity_setter] Error during parameter loading. Shutting down.");
    return 1;
  }
  ROS_INFO("[velocity_setter] All parameters loaded. Launching.");
  // reconfigure
  std::string service_name = "/move_base/" + setter.local_planner_ + "/set_parameters";
  setter.set_client_ = nh_.serviceClient<dynamic_reconfigure::Reconfigure>(service_name);
  // set services
  ros::ServiceServer set_srv_velocity_ = 
    nh_private_.advertiseService("set_velocity", &VelocitySetter::onSetVelocity, &setter);
  ros::ServiceServer set_srv_speed_ = 
    nh_private_.advertiseService("set_speed", &VelocitySetter::onSetSpeed, &setter);
  // get services
  ros::ServiceServer get_srv_speed_ = 
    nh_private_.advertiseService("get_speed", &VelocitySetter::onGetSpeed, &setter);

  ros::ServiceServer zone_speed_ =
    nh_private_.advertiseService("zone_speed", &VelocitySetter::onZoneSpeed, &setter);

  ros::spin();
  
  #ifdef MOVEL_LICENSE
    ml.logout();
  #endif

  return 0;
}
