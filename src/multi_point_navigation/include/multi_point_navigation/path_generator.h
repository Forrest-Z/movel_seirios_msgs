#ifndef MULTI_POINT_NAVIGATION_PATH_GENERATION_H
#define MULTI_POINT_NAVIGATION_PATH_GENERATION_H

#include <cmath>
#include <ros/ros.h>

namespace multi_point_navigation
{
struct Point
{
  float x;
  float y;
};

struct Path
{
  std::vector<Point> points;
  std::vector<int> major_points_idxs;
};

struct PathGeneratorConfig
{
  float point_generation_distance = 0.5;
  int max_bypass_degree = 1.0;
};

class PathGenerator
{
private:
  std::shared_ptr<PathGeneratorConfig> config_ptr_;
  const float min_distance_between_points_ = 0.1;

public:
  PathGenerator();
  ~PathGenerator();

  void setConfig(std::shared_ptr<PathGeneratorConfig> config_ptr);

  bool generatePath(const std::vector<Point> input_points, Path& path);

  void smoothenPath(const Path& input, Path& output);

private:
  void generatePathFromLineSegment(const Point& start_point, const Point& end_point,
                                   std::vector<Point>& minor_pt_container);

  void getPointsToCurve(const Path& path, std::vector<std::pair<int, int>>& points_to_curve_idxs);

  void constructQuadraticBezier(const std::vector<Point>& input, int bypass_degree, std::vector<Point>& output);

  Point getDividingPoint(const Point& p1, const Point& p2, float ratio);
};

}  // namespace multi_point_navigation

#endif