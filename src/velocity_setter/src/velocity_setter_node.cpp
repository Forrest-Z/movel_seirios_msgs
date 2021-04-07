/*
 *  USAGE:
 *  "/velocity_setter/set_velocity" service: Take velocity name (eg cruising,
 * cleaning) as input and set velocity according to config file
 */

#include <movel_hasp_vendor/license.h>
#include <ros_utils/ros_utils.h>
#include <velocity_setter/velocity_setter.h>

VelocitySetter setter;

// Load config file
bool loadParams(ros::NodeHandle &nh_private_) {
  ros_utils::ParamLoader loader(nh_private_);
  loader.get_required("local_planner", setter.local_planner_);
  loader.get_required("parameter_name_linear", setter.parameter_name_linear_);
  loader.get_required("parameter_name_angular", setter.parameter_name_angular_);

  std::map<std::string, double> params;
  nh_private_.getParam("velocities", params);
  if (params.size() == 0) {
    return false;
  }
  setter.velocities_ = params;
  return loader.params_valid();
}

int main(int argc, char **argv) {
#ifdef MOVEL_LICENSE
  MovelLicense ml(11);
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

  std::string service_name =
      "/move_base/" + setter.local_planner_ + "/set_parameters";
  setter.set_client_ =
      nh_.serviceClient<dynamic_reconfigure::Reconfigure>(service_name);
  ros::ServiceServer set_srv_velocity_ = nh_private_.advertiseService(
      "set_velocity", &VelocitySetter::onSetVelocity, &setter);
  ros::ServiceServer set_srv_speed_ = nh_private_.advertiseService(
      "set_speed", &VelocitySetter::onSetSpeed, &setter);

  ros::spin();
#ifdef MOVEL_LICENSE
  ml.logout();
#endif

  return 0;
}