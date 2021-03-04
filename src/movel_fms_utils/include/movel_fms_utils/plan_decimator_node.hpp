#ifndef plan_decimator_node_hpp
#define plan_decimator_node_hpp

#include "movel_fms_utils/plan_decimator.hpp"

class PlanDecimatorNode
{
public:
  PlanDecimatorNode();
  ~PlanDecimatorNode(){}

  ros::NodeHandle nh_;
  PlanDecimator pd_;

  void setupParams();
  void setupTopics(); // and services

private:
  ros::Subscriber plan_sub_;

};

#endif