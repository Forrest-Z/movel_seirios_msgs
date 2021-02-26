#ifndef plan_measurer_hpp
#define plan_measurer_hpp

#include <geometry_msgs/Pose.h>
#include <movel_seirios_msgs/GetPlanLength.h>
#include <movel_seirios_msgs/GetPlanLengthSimple.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/GetPlan.h>
#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>

class PlanMeasurer
{
public:
  PlanMeasurer();
  ~PlanMeasurer(){};

  // parameters
  std::string planner_service_name_;
  std::string robot_frame_;
  std::string map_frame_;
  ros::ServiceClient planner_service_;

  // 
  nav_msgs::Path getPlan(geometry_msgs::Pose start, geometry_msgs::Pose goal);
  double measurePlan(nav_msgs::Path plan);
  bool getRobotPose(geometry_msgs::Pose &robot_pose);

  // service callbacks
  bool measQrySimpleCb(movel_seirios_msgs::GetPlanLengthSimple::Request &req,
                       movel_seirios_msgs::GetPlanLengthSimple::Response &res);
  
  bool measQryCb(movel_seirios_msgs::GetPlanLength::Request &req,
                 movel_seirios_msgs::GetPlanLength::Response &res);

private:
  ros::NodeHandle nh_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_ear_;
};

#endif