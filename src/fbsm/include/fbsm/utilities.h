#ifndef LINE_EXTRACTION_UTILITIES_H
#define LINE_EXTRACTION_UTILITIES_H

#include <vector>
#include <cmath>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <Eigen/Dense>
#include <tf/transform_listener.h>

namespace line_extraction
{
struct CachedData
{
  std::vector<unsigned int> indices;
  std::vector<double> bearings;
  std::vector<double> cos_bearings;
  std::vector<double> sin_bearings;
};

struct RangeData
{
  std::vector<double> ranges;
  std::vector<double> xs;
  std::vector<double> ys;
};

struct Params
{
  double bearing_var;
  double range_var;
  double least_sq_angle_thresh;
  double least_sq_radius_thresh;
  double max_line_gap;
  double min_line_length;
  double min_range;
  double max_range;
  double min_split_dist;
  double outlier_dist;
  unsigned int min_line_points;
};

struct PointParams
{
  std::vector<double> a;
  std::vector<double> ap;
  std::vector<double> app;
  std::vector<double> b;
  std::vector<double> bp;
  std::vector<double> bpp;
  std::vector<double> c;
  std::vector<double> s;
};

struct Keypoint
{
  boost::array<double, 2> point;
  std::vector<double> histogram;
  int match;
  double score;
};

struct Transform_vector
{
  double tx;
  double ty;
  double yaw;
};

struct is_near
{
  bool operator()(Transform_vector first, Transform_vector second)
  {
    return ((fabs(first.tx - second.tx) < 0.01) and (fabs(first.ty - second.ty) < 0.01));
  }
};

bool is_near_yaw(double first, double second)
{
  return ((fabs(first - second) < 0.01) and (fabs(first - second) < 0.01));
}

double pi_to_pi(double angle)
{
  angle = fmod(angle, 2 * M_PI);
  if (angle >= M_PI)
    angle -= 2 * M_PI;
  return angle;
}

bool isBetween(boost::array<double, 2> a, boost::array<double, 2> b, boost::array<double, 2> c)
{
  double epsilon = 0.0001;
  double crossproduct, dotproduct, squaredlengthba;

  crossproduct = (c[1] - a[1]) * (b[0] - a[0]) - (c[0] - a[0]) * (b[1] - a[1]);

  if (std::abs(crossproduct) > epsilon)
  {
    return false;
  }

  dotproduct = (c[0] - a[0]) * (b[0] - a[0]) + (c[1] - a[1]) * (b[1] - a[1]);
  if (dotproduct < 0)
  {
    return false;
  }

  squaredlengthba = (b[0] - a[0]) * (b[0] - a[0]) + (b[1] - a[1]) * (b[1] - a[1]);
  if (dotproduct > squaredlengthba)
  {
    return false;
  }
  return true;
}

void scanToPoints(const std::vector<double>& x, const std::vector<double>& y, std::vector<Keypoint>& points)
{
  Keypoint p;
  double inf = std::numeric_limits<double>::infinity();
  points.clear();
  for (int i = 0; i < x.size(); ++i)
  {
    // clean inf points from the scan
    if (x[i] < inf and x[i] > -inf and y[i] < inf and y[i] > -inf)
    {
      p.point = { { x[i], y[i] } };
      points.push_back(p);
    }
  }
}

void matrixAsTransfrom(const Eigen::Matrix4f& out_mat, tf::Transform& bt)
{
  double mv[12];

  mv[0] = out_mat(0, 0);
  mv[4] = out_mat(0, 1);
  mv[8] = out_mat(0, 2);
  mv[1] = out_mat(1, 0);
  mv[5] = out_mat(1, 1);
  mv[9] = out_mat(1, 2);
  mv[2] = out_mat(2, 0);
  mv[6] = out_mat(2, 1);
  mv[10] = out_mat(2, 2);

  tf::Matrix3x3 basis;
  basis.setFromOpenGLSubMatrix(mv);
  tf::Vector3 origin(out_mat(0, 3), out_mat(1, 3), out_mat(2, 3));

  ROS_DEBUG("origin %f %f %f", origin.x(), origin.y(), origin.z());

  bt = tf::Transform(basis, origin);
}

double constrainAngle(double x)
{
  x = fmod(x + M_PI, 2 * M_PI);
  if (x < 0)
    x += 2 * M_PI;
  return x - M_PI;
}

}  // namespace line_extraction
#endif
