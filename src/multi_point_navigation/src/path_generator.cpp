#include <multi_point_navigation/path_generator.h>

namespace multi_point_navigation
{
PathGenerator::PathGenerator()
{
}

PathGenerator::~PathGenerator()
{
}

void PathGenerator::setConfig(std::shared_ptr<PathGeneratorConfig> config_ptr)
{
  config_ptr_ = config_ptr;
}

bool PathGenerator::generatePath(const std::vector<Point> input_points, Path& path)
{
  path.major_points_idxs.clear();
  path.points.clear();

  std::vector<Point> tmp_path_points;

  std::vector<int> tmp_major_pts_idxs = { 0 };

  for (int i = 0; i < input_points.size() - 1; i++)
  {
    Point start_pt = input_points[i];
    Point end_pt = input_points[i + 1];

    if (start_pt.x == end_pt.x && start_pt.y == end_pt.y)
    {
      ROS_ERROR(
          "[PathGenerator] Got 2 consecutive major points with same coordinates (%.2f, %.2f). This is not allowed",
          start_pt.x, start_pt.y);
      return false;
    }

    tmp_major_pts_idxs.push_back(tmp_major_pts_idxs.back());

    std::vector<Point> minor_pts;
    generatePathFromLineSegment(start_pt, end_pt, minor_pts);
    tmp_path_points.insert(tmp_path_points.end(), minor_pts.begin(), minor_pts.end());

    tmp_major_pts_idxs.back() = tmp_major_pts_idxs.back() + minor_pts.size();
  }
  tmp_path_points.push_back(input_points.back());

  Path tmp_path = { .points = tmp_path_points, .major_points_idxs = tmp_major_pts_idxs };
  path = tmp_path;

  return true;
}

void PathGenerator::smoothenPath(const Path& input, Path& output)
{
  Path tmp_input = input;
  output.points.clear();

  std::vector<std::pair<int, int>> major_pts_to_curve;
  getPointsToCurve(tmp_input, major_pts_to_curve);

  int input_point_idx_pointer = 0;
  for (int i = 0; i < major_pts_to_curve.size(); ++i)
  {
    // add points unaffected by curve generation to output
    int curved_point_idx = major_pts_to_curve[i].first;
    int bypass_degree = major_pts_to_curve[i].second;
    int furthest_unaffected_point_idx = curved_point_idx - bypass_degree - 1;
    for (int j = input_point_idx_pointer; j <= furthest_unaffected_point_idx; ++j)
    {
      output.points.push_back(tmp_input.points[j]);
      input_point_idx_pointer++;
    }

    std::vector<Point> bezier_control_points, bezier_curve;
    bezier_control_points.push_back(tmp_input.points[furthest_unaffected_point_idx + 1]);
    bezier_control_points.push_back(tmp_input.points[curved_point_idx]);
    bezier_control_points.push_back(tmp_input.points[curved_point_idx + bypass_degree]);
    constructQuadraticBezier(bezier_control_points, bypass_degree, bezier_curve);
    output.points.insert(output.points.end(), bezier_curve.begin(), bezier_curve.end());

    input_point_idx_pointer = input_point_idx_pointer + (2 * bypass_degree) + 1;
  }
  for (int i = input_point_idx_pointer; i < tmp_input.points.size(); ++i)
  {
    output.points.push_back(tmp_input.points[i]);
    input_point_idx_pointer++;
  }
}

int PathGenerator::getIndexNearestPointAheadOnPath(const Path& path, const Point& point)
{
  if (path.points.size() <= 1)
    return 0;
  
  int nearest_idx = -1;
  double nearest_dist = std::numeric_limits<double>::infinity();
  for (int i = 0; i < path.points.size(); ++i)
  {
    double path_point_dist = getDistance(path.points[i], point);
    if (path_point_dist < nearest_dist)
    {
      nearest_idx = i;
      nearest_dist = path_point_dist;
    }
  }

  if (nearest_idx == path.points.size() - 1)
    return nearest_idx;

  int next_nearest_idx = nearest_idx + 1;
  /**
   * get angle formed by query point, nearest path point, next nearest path point using dot product
   * a (dot) b = |a| |b| cos(theta), where
   * a: vector from nearest path point to query point, and
   * b: vector from nearest path point to next nearest path point
   */
  double a_x = point.x - path.points[nearest_idx].x;
  double a_y = point.y - path.points[nearest_idx].y;
  double b_x = path.points[next_nearest_idx].x - path.points[nearest_idx].x;
  double b_y = path.points[next_nearest_idx].y - path.points[nearest_idx].y;
  double a_dot_b = a_x * b_x + a_y * b_y;
  double magnitude_a = getDistance(path.points[nearest_idx], point);
  double magnitude_b = getDistance(path.points[nearest_idx], path.points[next_nearest_idx]);
  double theta = std::acos(a_dot_b / (magnitude_a * magnitude_b));

  /**
   * if theta is > (90 deg - tolerance), nearest path point is in front of query point.
   * if theta is <= (90 deg - tolerance), nearest path point is behind query point (nearest point ahead is next nearest point)
   */
  double tolerance = 5.0;  // deg
  tolerance = tolerance / 180.0 * M_PI;

  bool is_nearest_point_in_front = (theta > ((M_PI / 2) - tolerance));
  if (is_nearest_point_in_front)
    return nearest_idx;
  else
    return next_nearest_idx;
}

void PathGenerator::generatePathFromLineSegment(const Point& start_point, const Point& end_point,
                                                std::vector<Point>& path)
{
  path.clear();
  float point_gen_dist = config_ptr_->point_generation_distance;

  double segment_dx = end_point.x - start_point.x;
  double segment_dy = end_point.y - start_point.y;
  double segment_length = std::sqrt(pow(segment_dx, 2) + pow(segment_dy, 2));
  float n_generated_points = segment_length / point_gen_dist;
  if ((n_generated_points - int(n_generated_points)) * point_gen_dist < min_distance_between_points_)
    n_generated_points--;

  if (segment_dx != 0)  // slope of line segment from start_point to end_point is not inf
  {
    double slope = segment_dy / segment_dx;

    for (int j = 0; j <= int(n_generated_points); ++j)
    {
      // formulae: x = x0 + (d * cos(theta)), y = m*(x-x0) + y0
      double path_pt_x = start_point.x + ((point_gen_dist * j) * (segment_dx / segment_length));
      double path_pt_y = slope * (path_pt_x - start_point.x) + start_point.y;

      Point path_pt = { .x = path_pt_x, .y = path_pt_y };
      path.push_back(path_pt);
    }
  }
  else  // slope of line segment from start_point to end_point is inf (line is perfectly vertical)
  {
    for (int j = 0; j < int(n_generated_points); ++j)
    {
      /**
       * Utilizing C++ guaranteed implicit conversion of bool to int (T -> 1, F -> 0) to  get signum(segment_dy).
       * Signum function: returns 1 if >0, -1 if <0, 0 if ==0).
       */
      int signum_dy = (double(0) < segment_dy) - (segment_dy < double(0));

      double path_pt_x = start_point.x;
      double path_pt_y = start_point.y + (signum_dy * (point_gen_dist * j));

      Point path_pt = { .x = path_pt_x, .y = path_pt_y };
      path.push_back(path_pt);
    }
  }
}

void PathGenerator::getPointsToCurve(const Path& path, std::vector<std::pair<int, int>>& points_to_curve_idxs)
{
  points_to_curve_idxs.clear();

  int max_bypass_degree = config_ptr_->max_bypass_degree;
  std::vector<Point> path_points = path.points;
  std::vector<int> major_points_idxs = path.major_points_idxs;

  if (path_points.size() < 2)
    return;

  for (int i = 1; i < major_points_idxs.size() - 1; ++i)
  {
    Point prev_major = path_points[major_points_idxs[i - 1]];
    Point current_major = path_points[major_points_idxs[i]];
    Point next_major = path_points[major_points_idxs[i + 1]];

    // collinearity check: if area of triangle == 0, then collinear; area of triangle obtained by shoelace method
    float area = ((prev_major.x * current_major.y - current_major.x * prev_major.x) +
                  (current_major.x * next_major.y - next_major.x * current_major.y) +
                  (next_major.x * prev_major.y - prev_major.x * next_major.y)) /
                 2;
    bool is_collinear = (area == 0);

    if (!is_collinear)
    {
      int n_minor_pts_before = major_points_idxs[i] - major_points_idxs[i - 1] - 1;
      int n_minor_pts_after = major_points_idxs[i + 1] - major_points_idxs[i] - 1;
      // check if enough points exist between major points to accomodate bypass degree
      if (n_minor_pts_before >= (max_bypass_degree * 2) && n_minor_pts_after >= (max_bypass_degree * 2))
      {
        std::pair<int, int> point_to_curve_idx = { major_points_idxs[i], max_bypass_degree };
        points_to_curve_idxs.push_back(point_to_curve_idx);
      }
      else if (n_minor_pts_before <= n_minor_pts_after && n_minor_pts_before >= 2)
      {
        int point_bypass_degree = std::max(int(1), int(n_minor_pts_before / 2));
        std::pair<int, int> point_to_curve_idx = { major_points_idxs[i], point_bypass_degree };
        points_to_curve_idxs.push_back(point_to_curve_idx);
      }
      else if (n_minor_pts_before > n_minor_pts_after && n_minor_pts_after >= 2)
      {
        int point_bypass_degree = std::max(int(1), int(n_minor_pts_after / 2));
        std::pair<int, int> point_to_curve_idx = { major_points_idxs[i], point_bypass_degree };
        points_to_curve_idxs.push_back(point_to_curve_idx);
      }
    }
  }
}

void PathGenerator::constructQuadraticBezier(const std::vector<Point>& input, int bypass_degree,
                                             std::vector<Point>& output)
{
  // iterative implementation of quadratic Bezier curve B(t) given control points P_i = input[i]
  for (int i = 0; i <= bypass_degree; ++i)
  {
    float t = i / bypass_degree;
    // decomposing 2nd order parametric function B(t) into a series of linear functions for readability
    Point first_point = getDividingPoint(input[0], input[1], t);
    Point second_point = getDividingPoint(input[1], input[2], t);
    Point bezier = getDividingPoint(first_point, second_point, t);
    output.push_back(bezier);
  }
}

inline Point PathGenerator::getDividingPoint(const Point& a, const Point& b, float ratio)
{
  // get point P if P lies on line segment AB and satisfies |AP|/|AB| = ratio
  Point p;
  p.x = ratio * b.x + (1 - ratio) * a.x;
  p.y = ratio * b.y + (1 - ratio) * a.y;
  return p;
}

inline double PathGenerator::getDistance(const Point& p1, const Point& p2)
{
  return std::sqrt(pow((p2.x - p1.x), 2) + pow((p2.y - p1.y), 2));
}

}  // namespace multi_point_navigation