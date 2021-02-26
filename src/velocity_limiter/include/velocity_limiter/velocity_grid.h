#ifndef VELOCITY_LIMITER_VELOCITY_GRID_H
#define VELOCITY_LIMITER_VELOCITY_GRID_H

#include <array>
#include <ctime>

#include <velocity_limiter/common.h>
#include <velocity_limiter/velocity_grid_cell.h>

/**
 * Stores velocity limit values of grid points within the defined region
 */
class VelocityGrid
{
public:
  VelocityGrid();
  VelocityGrid(std::vector<Zone>& zone_list, double resolution);
  ~VelocityGrid();

  bool load(std::vector<Zone>& zone_list, double resolution);
  double computeVelocityLimit(Point position, Frontier& frontier_inner, Frontier& frontier_outer);
  bool getVelocityLimit(double x, double y, VelocityLimit& limit);
  VelocityLimit getMaxVelocityLimit();
  BoxHull getBoundary();
  void getOrigin(double& x, double& y);
  nav_msgs::OccupancyGrid toOccupancyGrid(uint8_t zone_id, std::string base_frame);

private:
  /**
   * Time taken to load one velocuty limit set.
   */
  ros::Time load_time_;
  /**
   * Maximum velocity of the regoin. Used by the function toOccupancyGrid.
   */
  VelocityLimit max_velocity_;
  /**
   * Height of the velocity grid region.
   */
  int height_;
  /**
   * Width of the velocity grid region.
   */
  int width_;
  /**
   * Boundary of the velocity grid region. Used to limit the region of point cloud.
   */
  BoxHull boundary_;
  /**
   * Resolution of the velocity grid.
   */
  double resolution_;
  /**
   * Origin of the velocity grid region.
   */
  geometry_msgs::Pose origin_;
  /**
   * Stoes the grid containing velocity limit.
   */
  std::vector<VelocityGridCell> grid_;

  void initMaxVelocity();
  void initMaxVelocity(double max_linear, double max_angular);
  BoxHull computeBoundary(std::vector<Zone>& zone_list);
  Point computeSegmentCoordinates(Point global_coordinates, Segment segment);
  Point computeGlobalCoordinates(Point local_coordinates, Segment segment);
  bool contains(Frontier& frontier, Point& position);
  Line getBorder(Frontier& frontier);
};

#endif
