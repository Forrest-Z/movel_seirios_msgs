#include <correlative_scan_matcher/range_routines.h>

namespace correlative_scan_matcher
{
namespace range_routines
{
double bresenham(Map *map, double ox, double oy, double oa, double max_range)
{
  // Bresenham raytracing
  int x0, x1, y0, y1;
  int x, y;
  int xstep, ystep;
  char steep;
  int tmp;
  int deltax, deltay, error, deltaerr;

  x0 = MAP_GXWX(map, ox);
  y0 = MAP_GYWY(map, oy);

  x1 = MAP_GXWX(map, ox + max_range * cos(oa));
  y1 = MAP_GYWY(map, oy + max_range * sin(oa));

  if (abs(y1 - y0) > abs(x1 - x0))
    steep = 1;
  else
    steep = 0;

  if (steep)
  {
    tmp = x0;
    x0 = y0;
    y0 = tmp;

    tmp = x1;
    x1 = y1;
    y1 = tmp;
  }

  deltax = abs(x1 - x0);
  deltay = abs(y1 - y0);
  error = 0;
  deltaerr = deltay;

  x = x0;
  y = y0;

  if (x0 < x1)
    xstep = 1;
  else
    xstep = -1;
  if (y0 < y1)
    ystep = 1;
  else
    ystep = -1;

  if (steep)
  {
    // if (!MAP_VALID(map, y, x) || map->cells[MAP_INDEX(map, y, x)].occ_state > -1)
    //   return sqrt((x - x0) * (x - x0) + (y - y0) * (y - y0)) * map->scale;
    if (!MAP_VALID(map, y, x) || map->cells[MAP_INDEX(map, y, x)].state == -1) // unknown or out of bounds
      return max_range;
    if (map->cells[MAP_INDEX(map, y, x)].state == 1) // obstacle
      return sqrt((x - x0) * (x - x0) + (y - y0) * (y - y0)) * map->resolution;
  }
  else
  {
    // if (!MAP_VALID(map, x, y) || map->cells[MAP_INDEX(map, x, y)].occ_state > -1)
    //   return sqrt((x - x0) * (x - x0) + (y - y0) * (y - y0)) * map->scale;
    if (!MAP_VALID(map, x, y) || map->cells[MAP_INDEX(map, x, y)].state == -1) // unknown or out of bounds
      return max_range;
    if (map->cells[MAP_INDEX(map, x, y)].state == 1) // obstacle
      return sqrt((x - x0) * (x - x0) + (y - y0) * (y - y0)) * map->resolution;
  }

  while (x != (x1 + xstep * 1))
  {
    x += xstep;
    error += deltaerr;
    if (2 * error >= deltax)
    {
      y += ystep;
      error -= deltax;
    }

    if (steep)
    {
      // if (!MAP_VALID(map, y, x) || map->cells[MAP_INDEX(map, y, x)].occ_state > -1)
      //   return sqrt((x - x0) * (x - x0) + (y - y0) * (y - y0)) * map->scale;
      if (!MAP_VALID(map, y, x) || map->cells[MAP_INDEX(map, y, x)].state == -1) // unknown or out of bounds
        return max_range;
      if (map->cells[MAP_INDEX(map, y, x)].state == 1) // obstacle
        return sqrt((x - x0) * (x - x0) + (y - y0) * (y - y0)) * map->resolution;
    }
    else
    {
      // if (!MAP_VALID(map, x, y) || map->cells[MAP_INDEX(map, x, y)].occ_state > -1)
      //   return sqrt((x - x0) * (x - x0) + (y - y0) * (y - y0)) * map->scale;
      if (!MAP_VALID(map, x, y) || map->cells[MAP_INDEX(map, x, y)].state == -1) // unknown or out of bounds
        return max_range;
      if (map->cells[MAP_INDEX(map, x, y)].state == 1) // obstacle
        return sqrt((x - x0) * (x - x0) + (y - y0) * (y - y0)) * map->resolution;
    }
  }
  return max_range;
}
} // namespace range_routines
} // namespace correlative_scan_matcher