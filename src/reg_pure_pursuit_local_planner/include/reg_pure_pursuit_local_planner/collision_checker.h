#ifndef FOOTPRINT_COLLISION_CHECKER_H_
#define FOOTPRINT_COLLISION_CHECKER_H_

#include <string>
#include <vector>
#include <memory>
#include <algorithm>

#include <ros/ros.h>
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Point.h"
#include <costmap_2d/costmap_2d_ros.h>

namespace reg_pure_pursuit_local_planner 
{
typedef std::vector<geometry_msgs::Point> Footprint;

/**
 * @class FootprintCollisionChecker
 * @brief Checker for collision with a footprint on a costmap
 */


class FootprintCollisionChecker
{
public:
  /**
   * @brief A constructor.
   */
  FootprintCollisionChecker();
  /**
   * @brief A constructor.
   */
  explicit FootprintCollisionChecker(costmap_2d::Costmap2D* costmap);
  /**
   * @brief Find the footprint cost in oriented footprint
   */
  double footprintCost(const Footprint footprint);
  /**
   * @brief Find the footprint cost a a post with an unoriented footprint
   */
  double footprintCostAtPose(double x, double y, double theta, const Footprint footprint);
  /**
   * @brief Get the cost for a line segment
   */
  double lineCost(int x0, int x1, int y0, int y1) const;
  /**
   * @brief Get the map coordinates from a world point
   */
  bool worldToMap(double wx, double wy, unsigned int & mx, unsigned int & my);
  /**
   * @brief Get the cost of a point
   */
  double pointCost(int x, int y) const;
  /**
  * @brief Set the current costmap object to use for collision detection
  */
  void setCostmap(costmap_2d::Costmap2D* costmap);
  /**
  * @brief Get the current costmap object
  */
  costmap_2d::Costmap2D* getCostmap()
  {
    return costmap_;
  }

protected:
  costmap_2d::Costmap2D* costmap_;
};

}  // namespace nav2_costmap_2d

#endif  // NAV2_COSTMAP_2D__FOOTPRINT_COLLISION_CHECKER_HPP_