#include <movel_hasp_vendor/license.h>
#include "planner_utils/planner_utils.hpp"
#include <nav_msgs/GetPlan.h>
#include <nav_msgs/Path.h>
#include <movel_seirios_msgs/GetReachableSubplan.h>


class PlannerUtilsNode
{
private:
  ros::NodeHandle nh_;
  PlannerUtils pu_;
  // services/topics
  ros::ServiceServer make_clean_plan_service_;
  ros::ServiceServer calc_reachable_subplan_service_;
  ros::ServiceServer make_reachable_plan_service_;
  ros::Publisher clean_plan_pub_;
  ros::Publisher reachable_plan_pub_;

public:
  PlannerUtilsNode()
  {
    make_clean_plan_service_ = nh_.advertiseService("/make_clean_plan", &PlannerUtilsNode::makeCleanPlanCb, this);
    calc_reachable_subplan_service_ = nh_.advertiseService("/calc_reachable_subplan", &PlannerUtilsNode::calcReachableSubplanCb, this);
    make_reachable_plan_service_ = nh_.advertiseService("/make_reachable_plan", &PlannerUtilsNode::makeReachablePlanServiceCb, this);
    clean_plan_pub_ = nh_.advertise<nav_msgs::Path>("/clean_plan", 1);
    reachable_plan_pub_ = nh_.advertise<nav_msgs::Path>("/reachable_plan", 1);

    ros::NodeHandle nl("~");
    if (nl.hasParam("extra_safety_buffer")){
      nl.getParam("extra_safety_buffer", pu_.extra_safety_buffer_);
    }
  }


  ~PlannerUtilsNode(){};


  bool makeCleanPlanCb(nav_msgs::GetPlan::Request& req, nav_msgs::GetPlan::Response& res)
  {
    std::vector<geometry_msgs::PoseStamped> plan;
    if (pu_.makeCleanPlan(req.start, req.goal, plan)) {
      if (plan.size() > 0) {
        res.plan.header = plan[0].header;
        res.plan.poses = plan;
        clean_plan_pub_.publish(res.plan);
        return true;
      }
    }
    return false;
  }


  bool calcReachableSubplanCb(movel_seirios_msgs::GetReachableSubplan::Request& req, 
                              movel_seirios_msgs::GetReachableSubplan::Response& res)
  {
    int reachable_idx = 0;
    int blocked_idx = 0;
    if (pu_.calcReachableSubplan(req.plan.poses, req.start_from_idx, reachable_idx, blocked_idx)) {
      res.reachable_idx = reachable_idx;
      res.blocked_idx = blocked_idx;
      // pub reachable plan
      nav_msgs::Path reachable_plan = req.plan;
      if (reachable_idx < reachable_plan.poses.size()-1) {
        reachable_plan.poses.erase(reachable_plan.poses.begin()+reachable_idx, reachable_plan.poses.end());
        reachable_plan.header = reachable_plan.poses[0].header;
      }
      reachable_plan_pub_.publish(reachable_plan);
      return true;
    }
    return false;
  }


  bool makeReachablePlanServiceCb(nav_msgs::GetPlan::Request& req, nav_msgs::GetPlan::Response& res)
  {
    std::vector<geometry_msgs::PoseStamped> plan;
    if (pu_.makePlanToReachable(req.start, req.goal, plan)) {
      if (plan.size() > 0) {
        res.plan.header = plan[0].header;
        res.plan.poses = plan;
        reachable_plan_pub_.publish(res.plan);
        return true;
      }
    }
    return false;
  }

};


int main(int argc, char** argv)
{
#ifdef MOVEL_LICENSE
  MovelLicense ml(20);
  if (!ml.login())
    return 1;
#endif

  ros::init(argc, argv, "planner_utils");
  PlannerUtilsNode pun;
  
  ros::spin();
  
#ifdef MOVEL_LICENSE
  ml.logout();
#endif

  return 0;
}