#include "movel_fms_utils/plan_measurer.hpp"
#include "movel_fms_utils/plan_measurer_node.hpp"
#include <movel_hasp_vendor/license.h>

PlanMeasurerNode::PlanMeasurerNode()
{
  setupParams();
  setupTopics();
}

void PlanMeasurerNode::setupParams()
{
  ros::NodeHandle nl("~");
  planner_service_name_ = "/move_base/make_plan";
  if (nl.hasParam("planner_service_name"))
    nl.getParam("planner_service_name", planner_service_name_);

  std::string robot_frame = "base_link";
  if (nl.hasParam("robot_frame"))
    nl.getParam("robot_frame", robot_frame);
  pm_.robot_frame_ = robot_frame;

  std::string map_frame = "map";
  if (nl.hasParam("map_frame"))
    nl.getParam("map_frame", map_frame);
  pm_.map_frame_ = map_frame;

  ROS_INFO("[Plan Measurer] Params OK");
}

void PlanMeasurerNode::setupTopics()
{
  ros::service::waitForService(planner_service_name_);
  pm_.planner_service_ = nh_.serviceClient<nav_msgs::GetPlan>(planner_service_name_);
  ros::NodeHandle nh_local("~");
  measurer_service_ = nh_local.advertiseService("measure_plan", &PlanMeasurer::measQryCb, &pm_);
  simple_measurer_service_ = nh_local.advertiseService("measure_plan_simple", &PlanMeasurer::measQrySimpleCb, &pm_);

  ROS_INFO("[Plan Measurer] Services OK");
}

int main(int argc, char** argv)
{
#ifdef MOVEL_LICENSE
  MovelLicense ml(22);
  if (!ml.login())
    return 1;
#endif

  ros::init(argc, argv, "plan_measurer");
  PlanMeasurerNode measure;

  ros::spin();

#ifdef MOVEL_LICENSE
  ml.logout();
#endif

  return 0;
}
