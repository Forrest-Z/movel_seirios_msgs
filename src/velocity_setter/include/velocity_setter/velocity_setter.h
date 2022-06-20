/*
 *  This node sets the velocity of the robot through dynamic reconfigure of 
 *  the max velocity parameter of move_base node.
 */

#ifndef VELOCITY_SETTER_H_
#define VELOCITY_SETTER_H_

#include <ros/ros.h>
#include <ros/console.h>
#include <movel_seirios_msgs/SetVelocity.h>
#include <movel_seirios_msgs/SetSpeed.h>
#include <movel_seirios_msgs/GetSpeed.h>
#include <dynamic_reconfigure/DoubleParameter.h>
#include <dynamic_reconfigure/Reconfigure.h>

class VelocitySetter
{
public:
  /**
    *  @brief Assign velocity through dynamic reconfigure client (set velocity by name)
    */
  bool setVelocity(double velocity);

    /**
    *  @brief Assign velocity through dynamic reconfigure client (set velocity by number)
    */
  bool setSpeed(double v_linear, double v_angular);

  // ROS params
  std::string local_planner_;
  std::string parameter_name_linear_, parameter_name_angular_;
  std::map<std::string,double> velocities_;

  // Service client for setting move_base parameters
  ros::ServiceClient set_client_;

  /**
    *  @brief Service callback for setting velocity by name
    */
  bool onSetVelocity(movel_seirios_msgs::SetVelocity::Request &req, movel_seirios_msgs::SetVelocity::Response &res);

  /**
    *  @brief Service callback for setting velocity by number
    */
  bool onSetSpeed(movel_seirios_msgs::SetSpeed::Request &req, movel_seirios_msgs::SetSpeed::Response &res);

  // get speed service
  // cache velocities for get Services
  double last_set_linear_ = 0.2;   // 0.2 is a hack for initializing the velocities for speed zone
  double last_set_angular_ = 0.2;
  /**
    *  @brief Service callback for getting cached velocity
    */
  bool onGetSpeed(movel_seirios_msgs::GetSpeed::Request &req, movel_seirios_msgs::GetSpeed::Response &res);
  
};

#endif
