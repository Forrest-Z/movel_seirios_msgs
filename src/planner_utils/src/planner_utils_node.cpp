#include "planner_utils/planner_utils.hpp"
#include <nav_msgs/GetPlan.h>
#include <nav_msgs/Path.h>

class PlannerUtilsNode
{
private:
  ros::NodeHandle nh_;
  PlannerUtils pu_;

  ros::ServiceServer make_reachable_plan_service_;
  ros::Publisher reachable_plan_pub_;

public:
  PlannerUtilsNode()
  {
    make_reachable_plan_service_ = nh_.advertiseService("make_reachable_plan", &PlannerUtilsNode::makeReachablePlanServiceCb, this);
    reachable_plan_pub_ = nh_.advertise<nav_msgs::Path>("reachable_plan", 1);
  }

  ~PlannerUtilsNode(){};

  bool makeReachablePlanServiceCb(nav_msgs::GetPlan::Request &req, nav_msgs::GetPlan::Response &res)
  {
    std::vector<geometry_msgs::PoseStamped> plan;
    if (pu_.makePlanToReachable(req.start, req.goal, plan))
    {
      if (plan.size() > 0)
      {
        res.plan.header = plan[0].header;
        res.plan.poses = plan;

        reachable_plan_pub_.publish(res.plan);
        return true;
      }
      return false;
    }
    return false;
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "planner_utils");
  PlannerUtilsNode pun;

  ros::spin();
  return 0;
}