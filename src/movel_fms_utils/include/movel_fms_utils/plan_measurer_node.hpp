#ifndef plan_measurer_node_hpp
#define plan_measurer_node_hpp

#include "movel_fms_utils/plan_measurer.hpp"

class PlanMeasurerNode
{
public:
  PlanMeasurerNode();
  ~PlanMeasurerNode(){};
  void setupParams();
  void setupTopics(); // and services

private:
  ros::NodeHandle nh_;
  PlanMeasurer pm_;
  ros::ServiceServer measurer_service_;
  ros::ServiceServer simple_measurer_service_;
  std::string planner_service_name_;

};

#endif