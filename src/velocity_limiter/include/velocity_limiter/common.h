#ifndef VELOCITY_LIMITER_COMMON_H
#define VELOCITY_LIMITER_COMMON_H

#include <string.h>
#include <vector>
#include <limits>
#include <math.h>
#include <stdint.h>

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/Polygon.h>
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/PointStamped.h>

typedef boost::geometry::model::d2::point_xy<double> Point;
typedef boost::geometry::model::segment<Point> Segment;
typedef boost::geometry::model::linestring<Point> Line;
typedef boost::geometry::model::polygon<Point> Polygon;

/**
 * Rectangular outer boundary of the shape of a frontier.
 */
struct BoxHull
{
  BoxHull()
  {
    top_left.x(0.0);
    top_left.y(0.0);
    bottom_right.x(0.0);
    bottom_right.y(0.0);
  }
  Point top_left;
  Point bottom_right;
};

/**
 * Define generic velocity limits.
 */
struct VelocityLimitGeneric
{
  VelocityLimitGeneric()
  {
    positive.x = DBL_MAX;
    positive.y = DBL_MAX;
    positive.z = DBL_MAX;
    negative.x = DBL_MAX;
    negative.y = DBL_MAX;
    negative.z = DBL_MAX;
  }
  // VelocityLimitGeneric(double positive_x, double positive_y, double positive_z, double negative_x, double negative_y,
  // double negative_z)
  //{
  //  positive.x = velocity_max;
  //  positive.y = velocity_max;
  //  positive.z = velocity_max;
  //  negative.x = velocity_max;
  //  negative.y = velocity_max;
  //  negative.z = velocity_max;
  //}
  geometry_msgs::Vector3 positive;
  geometry_msgs::Vector3 negative;
};

/**
 * Stores geometry limits.
 */
struct VelocityLimit
{
  // VelocityLimit(double linear_positive_x, double linear_negative_x, double angular_positive_z, double
  // angular_negative_z)
  //{
  //  VelocityLimitGeneric linear_tmp(linear_positive_x, DBL_MAX, DBL_MAX, linear_negative_x, DBL_MAX, DBL_MAX);
  //  linear = linear_tmp;
  //  VelocityLimitGeneric angular_tmp(DBL_MAX, DBL_MAX, angular_positive_z, DBL_MAX, DBL_MAX, angular_negative_z);
  //  angular = angular_tmp;
  //}
  VelocityLimitGeneric linear;
  VelocityLimitGeneric angular;
};

enum ZoneId
{
  LINEAR_POSITIVE_X,
  LINEAR_NEGATIVE_X,
  LINEAR_POSITIVE_Y,
  LINEAR_NEGATIVE_Y,
  LINEAR_POSITIVE_Z,
  LINEAR_NEGATIVE_Z,
  ANGULAR_POSITIVE_X,
  ANGULAR_NEGATIVE_X,
  ANGULAR_POSITIVE_Y,
  ANGULAR_NEGATIVE_Y,
  ANGULAR_POSITIVE_Z,
  ANGULAR_NEGATIVE_Z
};

/**
 * Stores shape of a velocity limit region.
 */
struct Shape
{
  std::vector<double> x_list;  ///< x coordinates
  std::vector<double> y_list;  ///< x coordinates
  Polygon polygon;             ///< The polygon formed by the coordinates
  BoxHull hull;                ///< The rectangle surrounding the polygon
};

/**
 * Stores frontier of a velocity limit region.
 */
struct Frontier
{
  int inclusion;  ///< Order of the frontiers. A frontier with larger inclusion number should contain the one with
                  /// smaller inclusion number entirely.
  double value;   ///< The velocity limit value of the frontier. If an obstacle is positioned right on the frontier, the
                  /// velocity limit would be this value.
  Shape shape;
};

/**
 * Stores the zone of a velocity limit region, containing different frontiers.
 */
struct Zone
{
  ZoneId id;              ///< The unique id of the zone
  std::string type;       ///< Type of the velocity that the zone is applied to, allowed: "linear" and "angular"
  std::string direction;  ///< Direction of the velocity that the zone is applied to, allowed: "positive" and
                          ///"negative".
  std::string dimension;  ///< Dimension of the velocity that the zone is applied to, allowed: "x" for linear and "z"
                          /// for angular.
  std::vector<Frontier> frontier_list;
};

/**
 * Stores a set of velocity limit regions, containing zones.
 */
struct Set
{
  std::vector<Zone> zone_list;
};

bool checkType(XmlRpc::XmlRpcValue value, XmlRpc::XmlRpcValue::Type type, std::string name);

#endif
