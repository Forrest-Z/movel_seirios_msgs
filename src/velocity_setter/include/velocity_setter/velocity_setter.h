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
  VelocitySetter();

  // Bookkeeping
  double task_linear_speed_;
  double task_angular_speed_;
  double zone_linear_speed_;
  double zone_angular_speed_;

  /**
    *  @brief Assign velocity through dynamic reconfigure client (set velocity by name)
    */
  bool setVelocity(double velocity);

  /**
    *  @brief Assign velocity through dynamic reconfigure client (set velocity by number)
    */
  bool setSpeed(double v_linear, double v_angular);

  /**
    *  @brief Select speed betweem task speed and zone speed
    */
  bool selectSpeed();

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
    *  @brief Service callback for setting velocity by number from task
    */
  bool onSetSpeed(movel_seirios_msgs::SetSpeed::Request &req, movel_seirios_msgs::SetSpeed::Response &res);

  /**
    *  @brief Service callback for setting velocity by number from speed limit zone
    */
  bool onZoneSpeed(movel_seirios_msgs::SetSpeed::Request &req, movel_seirios_msgs::SetSpeed::Response &res);
};

#endif
