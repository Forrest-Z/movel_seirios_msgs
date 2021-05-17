#include <correlative_scan_matcher/probability_grid.h>

namespace correlative_scan_matcher
{
/**
 * Updates a given area in a probability grid.
 *
 * @param sigma Gaussian noise model standard deviation
 * @param oi column index of the update region center
 * @param oj row index of the update region center
 * @param size_i column size of the update region
 * @param size_j row size of the update region
 */
void ProbabilityGrid::updateGrid(double sigma, int oi, int oj, int size_i, int size_j)
{
  unsigned char *marked;
  std::priority_queue<CellData> cell_queue;

  marked = new unsigned char[size_x * size_y];
  memset(marked, 0, sizeof(unsigned char) * (size_x * size_y));

  CachedDistanceMap *cdm = getDistanceMap(resolution, sigma);

  CellData cell;
  cell.map_ = this;
  for (int i = std::max(0, oi - (size_i / 2)); i < std::min(size_x, oi + (size_i / 2)); i++)
  {
    cell.src_i_ = cell.i_ = i;
    for (int j = std::max(0, oj - (size_j / 2)); j < std::min(size_y, oj + (size_j / 2)); j++)
    {
      // if the cell is occupied, mark the cell for probability grid expansion
      if (cells[MAP_INDEX(this, i, j)].state == 1)
      {
        cell.src_j_ = cell.j_ = j;
        cells[MAP_INDEX(this, i, j)].probability = MAX_PROBABILITY;
        marked[MAP_INDEX(this, i, j)] = 1;
        cell_queue.push(cell);
      }
    }
  }

  // expand the probability grid from the occupied cells
  while (!cell_queue.empty())
  {
    CellData current = cell_queue.top();
    if (current.i_ > 0)
      enqueue(current.i_ - 1, current.j_, current.src_i_,
              current.src_j_, cell_queue, cdm, marked);
    if (current.j_ > 0)
      enqueue(current.i_, current.j_ - 1, current.src_i_,
              current.src_j_, cell_queue, cdm, marked);
    if ((int)current.i_ < size_x - 1)
      enqueue(current.i_ + 1, current.j_, current.src_i_,
              current.src_j_, cell_queue, cdm, marked);
    if ((int)current.j_ < size_y - 1)
      enqueue(current.i_, current.j_ + 1, current.src_i_,
              current.src_j_, cell_queue, cdm, marked);

    cell_queue.pop();
  }

  delete[] marked;
}

/**
 * Adds a raycasted range into the probability grid
 *
 * @param ox x origin of the ray
 * @param oy y origin of the ray
 * @param oa orientation of the ray
 * @param range length of the ray
 */
void ProbabilityGrid::addRay(double ox, double oy, double oa, double range)
{
  // get cell that corresponds to end of ray
  int ei = MAP_GXWX(this, ox + range * cos(oa));
  int ej = MAP_GYWY(this, oy + range * sin(oa));

  // add end of the ray as an obstacle in the grid
  if (MAP_VALID(this, ei, ej))
    cells[MAP_INDEX(this, ei, ej)].state = 1;
}

/**
 * Retrieves the cached distance map.
 *
 * @param scale resolution of the cached distance map
 * @param sigma Gaussian noise model standard deviation
 * @return The cached distance map.
 */
CachedDistanceMap *getDistanceMap(double scale, double sigma)
{
  static CachedDistanceMap *cdm;

  // if there is no cached distance map, or if any of the parameters are changed,
  // generate a new cached distance map
  if (!cdm || (cdm->scale_ != scale) || (cdm->sigma_ != sigma))
  {
    if (cdm)
      delete cdm;
    cdm = new CachedDistanceMap(scale, sigma);
  }
  return cdm;
}

/**
 * Expands the probability grid.
 * Occupied cells are given the highest probability, and cells up to a certain distance around
 * the occupied cells are given decreasing probability based on the Gaussian model.
 *
 * @param i column index of the current cell
 * @param j row index of the current cell
 * @param src_i column index of the occupied cell this cell is expanded from
 * @param src_j row index of the occupied cell this cell is expanded from
 * @param cell_queue the queue of cells to be expanded
 * @param cdm the cached distance map
 * @param marked cells that have been marked for expansion
 */
void ProbabilityGrid::enqueue(int i, int j, int src_i, int src_j,
                              std::priority_queue<CellData> &cell_queue, CachedDistanceMap *cdm,
                              unsigned char *marked)
{
  if (marked[MAP_INDEX(this, i, j)])
    return;

  int di = std::abs(i - src_i);
  int dj = std::abs(j - src_j);
  double distance = cdm->distances_[di][dj];

  // this cell is out of the original occupied cell's range,
  // so we can stop expanding
  if (distance > cdm->cell_radius_)
    return;

  //cells[MAP_INDEX(this, i, j)].probability = MAX_PROBABILITY * (1 - pow((distance * resolution)/ cdm->max_dist_, 2.0));
  cells[MAP_INDEX(this, i, j)].probability = MAX_PROBABILITY * (GaussianNoiseModel(distance * resolution, cdm->sigma_) / cdm->peak_);

  // mark this cell for expansion
  CellData cell;
  cell.map_ = this;
  cell.i_ = i;
  cell.j_ = j;
  cell.src_i_ = src_i;
  cell.src_j_ = src_j;

  cell_queue.push(cell);
  marked[MAP_INDEX(this, i, j)] = 1;
}
} // namespace correlative_scan_matcher
