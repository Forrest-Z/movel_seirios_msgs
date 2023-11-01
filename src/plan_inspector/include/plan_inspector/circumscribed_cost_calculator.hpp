#ifndef PLAN_INSPECTOR_CIRCUMSCRIBED_COST_CALCULATOR_HPP
#define PLAN_INSPECTOR_CIRCUMSCRIBED_COST_CALCULATOR_HPP

#include <memory>

#include <ros/ros.h>
#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/footprint.h>
#include <geometry_msgs/Point.h>

class CircumscribedCostCalculator
{
public:
  CircumscribedCostCalculator()
  {
  }

  bool updateParams()
  {
    ros::NodeHandle nh("/move_base/local_costmap");

    // Get the robot's footprint from move_base local_costmap
    footprint_ = costmap_2d::makeFootprintFromParams(nh);
    if (footprint_.empty())
    {
      ROS_ERROR("[CircumscribedCostCalculator] Could not retrieve move_base local_costmap footprint, aborting.");
      return false;
    }
    ROS_INFO("[CircumscribedCostCalculator] Footprint points:");
    for (auto pt : footprint_)
      ROS_INFO("- [%.5lf, %.5lf]", pt.x, pt.y);

    // Get the move_base local_costmap inflation_layer's cost scaling factor
    if (!nh.getParam("inflation_layer/cost_scaling_factor", inflation_cost_scaling_factor_))
    {
      ROS_ERROR("[CircumscribedCostCalculator] Could not retrieve move_base local_costmap inflation_layer cost_scaling_factor, aborting.");
      return false;
    }
    ROS_INFO("[CircumscribedCostCalculator] Inflation cost scaling factor: %.5lf", inflation_cost_scaling_factor_);

    // Update the circumscribed cost threshold
    costmap_2d::calculateMinAndMaxDistances(footprint_, inscribed_radius_, circumscribed_radius_);
    circumscribed_cost_threshold_ = ((costmap_2d::INSCRIBED_INFLATED_OBSTACLE - 1) * 
                                     exp(-1.0 * inflation_cost_scaling_factor_ * 
                                     (circumscribed_radius_ - inscribed_radius_)));
    ROS_INFO("[CircumscribedCostCalculator] Circumscribed radius: %.5lf, Inscribed radius: %.5lf, Cost threshold: %d", 
             circumscribed_radius_, inscribed_radius_, circumscribed_cost_threshold_);

    return true;
  }

  bool getCircumscribedCostThreshold(unsigned char& circumscribed_cost_threshold)
  {
    if (!updateParams())
      return false;

    circumscribed_cost_threshold = circumscribed_cost_threshold_;
    return true;
  }

  bool getOccGridCircumscribedCostThreshold(unsigned char& circumscribed_cost_threshold)
  {
    unsigned char costmap_threshold; 
    if (!getCircumscribedCostThreshold(costmap_threshold))
      return false;

    if (costmap_threshold == costmap_2d::LETHAL_OBSTACLE)
      circumscribed_cost_threshold = 100;
    else if (costmap_threshold == costmap_2d::INSCRIBED_INFLATED_OBSTACLE)
      circumscribed_cost_threshold = 99;
    else if (costmap_threshold == costmap_2d::FREE_SPACE)
      circumscribed_cost_threshold = 0;
    else
      circumscribed_cost_threshold = (unsigned char) (1 + (97 * (costmap_threshold - 1) / 251));
    
    return true;
  }

private:
  unsigned char circumscribed_cost_threshold_;
  double inflation_cost_scaling_factor_;
  double inscribed_radius_, circumscribed_radius_;
  std::vector<geometry_msgs::Point> footprint_;
};

#endif