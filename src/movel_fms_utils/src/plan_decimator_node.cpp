#include "movel_fms_utils/plan_decimator_node.hpp"
#include <movel_hasp_vendor/license.h>

PlanDecimatorNode::PlanDecimatorNode()
{
  setupParams();
  setupTopics();
}

void PlanDecimatorNode::setupParams()
{
  ros::NodeHandle nh_local("~");

  pd_.d_decimation_ = 1.0;
  if (nh_local.hasParam("d_decimation"))
    nh_local.getParam("d_decimation", pd_.d_decimation_);
}

void PlanDecimatorNode::setupTopics()
{
  pd_.decimated_plan_pub_ = nh_.advertise<nav_msgs::Path>("decimated_plan", 1);
  pd_.set_decimation_srv_ = nh_.advertiseService("set_plan_decimation", &PlanDecimator::setDecimationCb, &pd_);
  pd_.get_decimated_plan_srv_ = nh_.advertiseService("get_decimated_plan", &PlanDecimator::getDecimatedPlanCb, &pd_);

  plan_sub_ = nh_.subscribe("/move_base/NavfnROS/plan", 1, &PlanDecimator::globalPlanCb, &pd_);
}

int main(int argc, char** argv)
{
#ifdef MOVEL_LICENSE
  MovelLicense ml(22);
  if (!ml.login())
    return 1;
#endif

  ros::init(argc, argv, "plan_decimator_node");

  PlanDecimatorNode pdn;
  ros::spin();

#ifdef MOVEL_LICENSE
  ml.logout();
#endif

  return 0;
}
