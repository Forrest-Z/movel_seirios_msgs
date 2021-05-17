#ifndef CORRELATIVE_SCAN_MATCHER_PROBABILITY_GRID_H
#define CORRELATIVE_SCAN_MATCHER_PROBABILITY_GRID_H

#include <vector>
#include <queue>
#include <cmath>
#include <cstring>
#include <cstddef>
#include <cstdlib>

#define MAX_PROBABILITY 255

// Convert from map coords to world coords
#define MAP_WXGX(map, i) (map->origin_x + ((i)-map->size_x / 2) * map->resolution)
#define MAP_WYGY(map, j) (map->origin_y + ((j)-map->size_y / 2) * map->resolution)

// Convert from world coords to map coords
#define MAP_GXWX(map, x) (floor((x - map->origin_x) / map->resolution + 0.5) + map->size_x / 2)
#define MAP_GYWY(map, y) (floor((y - map->origin_y) / map->resolution + 0.5) + map->size_y / 2)

// Test to see if the given map coords lie within the absolute map bounds.
#define MAP_VALID(map, i, j) ((i >= 0) && (i < map->size_x) && (j >= 0) && (j < map->size_y))

// Compute the cell index for the given map coords.
#define MAP_INDEX(map, i, j) ((i) + (j)*map->size_x)

namespace correlative_scan_matcher
{
/**
 * Stores the state and observation probability of a cell
 */
class Cell
{
public:
  /**
   * Constructor.
   * Initializes state to unknown and probability to 0.
   */
  Cell() : state(-1), probability(0){};
  /**
   * The state of the cell.
   * occupied = 1, free = 0, unknown = -1
   */
  int state;
  /**
   * The observation probability of the cell.
   * Used to form a probability grid for scan matching.
   */
  int probability;
};

/**
 * Stores map metadata and a vector of Cell objects
 */
class Map
{
public:
  int size_x, size_y;
  double origin_x, origin_y;
  double resolution;

  std::vector<Cell> cells;
};

/**
 * Stores information about a Cell for probability grid calculations.
 *
 * @see ProbabilityGrid::updateGrid
 * @see ProbabilityGrid::enqueue
 */
class CellData
{
public:
  Map *map_;
  unsigned int i_, j_;
  unsigned int src_i_, src_j_;
};

/**
 * Comparator.
 * Used for ordering CellData objects by probability in a priority queue.
 */
bool operator<(const CellData &a, const CellData &b)
{
  return a.map_->cells[MAP_INDEX(a.map_, a.i_, a.j_)].probability <
         b.map_->cells[MAP_INDEX(a.map_, b.i_, b.j_)].probability;
}

/**
 * Gaussian noise model used for modelling laser noise.
 */
double GaussianNoiseModel(double x, double sigma)
{
  return 1.0 / (sigma * sqrt(2.0 * M_PI)) * exp(-1.0 * pow(x, 2.0) / (2.0 * pow(sigma, 2.0)));
}

/**
 * Caches the distances between cells to reduce repeated calculations
 */
class CachedDistanceMap
{
public:
  CachedDistanceMap(double scale, double sigma) : distances_(NULL),
                                                  scale_(scale), sigma_(sigma)
  {
    max_dist_ = 3 * sigma; // approximate "width" of a normal distribution (68–95–99.7 rule)
    cell_radius_ = max_dist_ / scale;
    peak_ = GaussianNoiseModel(0, sigma);
    distances_ = new double *[cell_radius_ + 2];
    for (int i = 0; i <= cell_radius_ + 1; i++)
    {
      distances_[i] = new double[cell_radius_ + 2];
      for (int j = 0; j <= cell_radius_ + 1; j++)
      {
        distances_[i][j] = sqrt(i * i + j * j);
      }
    }
  }
  ~CachedDistanceMap()
  {
    if (distances_)
    {
      for (int i = 0; i <= cell_radius_ + 1; i++)
        delete[] distances_[i];
      delete[] distances_;
    }
  }
  double peak_;
  double **distances_;
  double scale_;
  double sigma_;
  double max_dist_;
  int cell_radius_;
};

/**
 * ProbabilityGrid derives from Map and introduces functions for probability grid calculations
 */
class ProbabilityGrid : public Map
{
  void enqueue(int i, int j, int src_i, int src_j, std::priority_queue<CellData> &cell_queue, CachedDistanceMap *cdm, unsigned char *marked);

public:
  void updateGrid(double sigma, int oi, int oj, int size_i, int size_j);
  void addRay(double ox, double oy, double oa, double range);
};

CachedDistanceMap *getDistanceMap(double scale, double sigma);
} // namespace correlative_scan_matcher

#endif
