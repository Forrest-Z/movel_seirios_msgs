#ifndef CORRELATIVE_SCAN_MATCHER_RANGE_ROUTINES_H
#define CORRELATIVE_SCAN_MATCHER_RANGE_ROUTINES_H

#include <correlative_scan_matcher/probability_grid.h>

namespace correlative_scan_matcher
{
/**
 * Functions for manipulating range data
 */
namespace range_routines
{
/**
 * Bresenham's line algorithm for raytracing
 *
 * @param map input map
 * @param ox starting x position
 * @param oy starting y position
 * @param oa starting orientation
 * @param max_range maximum raytrace range
 * @return raytrace range
 */
double bresenham(Map *map, double ox, double oy, double oa, double max_range);
}
} // namespace correlative_scan_matcher

#endif
