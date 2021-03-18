#ifndef plan_decimator_hpp
#define plan_decimator_hpp

#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <movel_seirios_msgs/SetPlanDecimation.h>
#include <movel_seirios_msgs/GetDecimatedPlan.h>

class PlanDecimator
{
public:
  // constructor/destructor
  PlanDecimator();
  ~PlanDecimator(){};

  // parameters
  double d_decimation_; // minimum distance for waypoint separation
  std::string global_planner_; // name of global planner used by move_base

  // bookkeeping
  nav_msgs::Path latest_path_raw_;
  nav_msgs::Path latest_path_decimated_;

  // service servers and publishers
  ros::ServiceServer get_decimated_plan_srv_;
  ros::ServiceServer set_decimation_srv_;
  ros::Publisher decimated_plan_pub_;

  // callbacks
  void globalPlanCb(nav_msgs::Path msg);
  bool getDecimatedPlanCb(movel_seirios_msgs::GetDecimatedPlan::Request &req,
                          movel_seirios_msgs::GetDecimatedPlan::Response &res);
  bool setDecimationCb(movel_seirios_msgs::SetPlanDecimation::Request &req,
                       movel_seirios_msgs::SetPlanDecimation::Response &res);

  // utilities
  void decimatePlan(nav_msgs::Path &in_plan, nav_msgs::Path &out_plan, double decimation);
  double calcDistance(geometry_msgs::Pose a, geometry_msgs::Pose b);
  double calcAngle(geometry_msgs::Pose a, geometry_msgs::Pose b);

private:
  ros::NodeHandle nh_;


};

#endif
