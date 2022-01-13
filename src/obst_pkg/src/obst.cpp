#include "obst_pkg/obst.hpp"

// Constructor
ObstClass::ObstClass(ros::NodeHandle* nodehandle):n_(*nodehandle){
    name_ = "obst_node";
    peripheral_angle_ = 0.6;
    detect_range_ = 0.5;
    if (!loadParams()){
        ROS_FATAL("[%s] Error during parameter loading. Shutting down.", name_.c_str());
    }

    obst_pub_ = n_.advertise<std_msgs::Bool>("/obst", 10);
    laser_scan_sub_ = n_.subscribe("/scan/filtered", 1, &ObstClass::laserScanCB, this);
}

template <typename param_type>
bool ObstClass::load_param_util(string param_name, param_type& output)
{
  if (!n_.getParam(param_name, output)) {
    ROS_ERROR("[%s] Failed to load parameter: %s", name_.c_str(), param_name.c_str());
    return false;
  }
  else {  
    if (is_same<param_type, bool>::value) {   // bool
      ROS_INFO_STREAM("[" << name_ << "] " << param_name << ": " << output ? "true" : "false");
    }
    else {   // all others 
      ROS_INFO_STREAM("[" << name_ << "] " << param_name << ": " << output);
    }
    return true;
  }
}

bool ObstClass::loadParams(){
  ROS_WARN("[%s] Loading of plugin parameters by ros_utils has not been implemented. Loading directly from Parameter "
           "Server instead.", name_.c_str());
  
  return true;
}

void ObstClass::laserScanCB(const sensor_msgs::LaserScan::ConstPtr& scan_msg){
    std_msgs::Bool obstacle_msg;
    obstacle_msg.data = false;
    for(int i = 0; i < scan_msg->ranges.size(); i++){
        float current_angle = scan_msg->angle_min + (i * scan_msg->angle_increment);
        if(current_angle > M_PI){
            current_angle = current_angle - (2 * M_PI);
        }
        if(current_angle <= (peripheral_angle_/2) && current_angle >= -(peripheral_angle_/2) && scan_msg->ranges[i] <= detect_range_ && scan_msg->ranges[i] > scan_msg->range_min){
            obstacle_msg.data = true;
            break;
        }
    }
    obst_pub_.publish(obstacle_msg);
}