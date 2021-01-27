#ifndef VELOCITY_LIMITER_VELOCITY_GRID_CELL_H
#define VELOCITY_LIMITER_VELOCITY_GRID_CELL_H

#include <velocity_limiter/common.h>

/**
 * Stores propertied of one cell in the velocity grid.
 */
struct VelocityGridCell
{
  Point position;       ///< Position of the bottom right point of the cell.
  VelocityLimit limit;  ///< The velociity limit value of the cell.
};

#endif
