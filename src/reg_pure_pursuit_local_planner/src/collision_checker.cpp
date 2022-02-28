#include <memory>
#include <string>
#include <vector>
#include <algorithm>

#include "reg_pure_pursuit_local_planner/collision_checker.h"

#include "costmap_2d/cost_values.h"
#include "reg_pure_pursuit_local_planner/line_iterator.hpp"

using namespace std::chrono_literals;

namespace reg_pure_pursuit_local_planner 
{

FootprintCollisionChecker::FootprintCollisionChecker()
: costmap_(nullptr)
{
}


FootprintCollisionChecker::FootprintCollisionChecker(
  costmap_2d::Costmap2D * costmap)
: costmap_(costmap)
{
}


double FootprintCollisionChecker::footprintCost(const Footprint footprint)
{
  // now we really have to lay down the footprint in the costmap_ grid
  unsigned int x0, x1, y0, y1;
  double footprint_cost = 0.0;

  // get the cell coord of the first point
  if (!worldToMap(footprint[0].x, footprint[0].y, x0, y0)) {
    return static_cast<double>(costmap_2d::LETHAL_OBSTACLE);
  }

  // cache the start to eliminate a worldToMap call
  unsigned int xstart = x0;
  unsigned int ystart = y0;

  // we need to rasterize each line in the footprint
  for (unsigned int i = 0; i < footprint.size() - 1; ++i) {

    // get the cell coord of the second point
    if (!worldToMap(footprint[i + 1].x, footprint[i + 1].y, x1, y1)) {
      return static_cast<double>(costmap_2d::LETHAL_OBSTACLE);
    }

    footprint_cost = std::max(lineCost(x0, x1, y0, y1), footprint_cost);

    // the second point is next iteration's first point
    x0 = x1;
    y0 = y1;

    // if in collision, no need to continue
    if (footprint_cost == static_cast<double>(costmap_2d::LETHAL_OBSTACLE)) {
      return footprint_cost;
    }
  }

  // we also need to connect the first point in the footprint to the last point
  // the last iteration's x1, y1 are the last footprint point's coordinates
  return std::max(lineCost(xstart, x1, ystart, y1), footprint_cost);
}


double FootprintCollisionChecker::lineCost(int x0, int x1, int y0, int y1) const
{
  double line_cost = 0.0;
  double point_cost = -1.0;

  for (reg_pure_pursuit_local_planner::LineIterator line(x0, y0, x1, y1); line.isValid(); line.advance()) {
    point_cost = pointCost(line.getX(), line.getY());   // Score the current point

    if (line_cost < point_cost) {
      line_cost = point_cost;
    }

    // if in collision, no need to continue
    if (line_cost == static_cast<double>(costmap_2d::LETHAL_OBSTACLE)) {
      return line_cost;
    }
  }

  return line_cost;
}


bool FootprintCollisionChecker::worldToMap(
  double wx, double wy, unsigned int & mx, unsigned int & my)
{
  return costmap_->worldToMap(wx, wy, mx, my);
}


double FootprintCollisionChecker::pointCost(int x, int y) const
{
  return costmap_->getCost(x, y);
}


void FootprintCollisionChecker::setCostmap(costmap_2d::Costmap2D* costmap)
{
  costmap_ = costmap;
}


double FootprintCollisionChecker::footprintCostAtPose(
  double x, double y, double theta, const Footprint footprint)
{
  double cos_th = cos(theta);
  double sin_th = sin(theta);
  Footprint oriented_footprint;
  for (unsigned int i = 0; i < footprint.size(); ++i) {
    geometry_msgs::Point new_pt;
    new_pt.x = (float) (x + (footprint[i].x * cos_th - footprint[i].y * sin_th));
    new_pt.y = (float) (y + (footprint[i].x * sin_th + footprint[i].y * cos_th));
    new_pt.z = 0;
    oriented_footprint.push_back(new_pt);
  }

  return footprintCost(oriented_footprint);
}
}