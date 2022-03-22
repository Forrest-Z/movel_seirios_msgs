#ifndef PATH_DIST_UTILS_HPP
#define PATH_DIST_UTILS_HPP

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>


namespace movel_fms_utils
{

const double PI = 3.14159265359;

enum class DistMetric {
  EUCLIDEAN,
  MANHATTAN,
};


inline double eclideadDist(const geometry_msgs::Pose& a, const geometry_msgs::Pose& b)
{
  double dx = a.position.x - b.position.x;
  double dy = a.position.y - b.position.y;
  return std::sqrt(dx*dx + dy*dy);
}


inline double manhattanDist(const geometry_msgs::Pose& a, const geometry_msgs::Pose& b)
{
  double dx = a.position.x - b.position.x;
  double dy = a.position.y - b.position.y;
  return std::abs(dx) + std::abs(dy);
}


inline double cosine_a_FromSides(double a, double b, double c)
{
  // a^2 = b^2 + c^2 - 2bc.cos(alpha)   // cosine law
  // 2bc.cos(alpha) = b^2 + c^2 - a^2
  // cos(alpha) = [  b^2 + c^2 - a^2 ] / 2bc
  return ( b*b + c*c - a*a ) / ( 2*b*c );
}


inline int getNearestWaypointIdx(const std::vector<geometry_msgs::Pose>& waypoints, 
                          const geometry_msgs::Pose& robot_pose,
                          const DistMetric dist_type)
{
  // sanity check
  if (waypoints.size() == 0) 
    throw std::invalid_argument("received empty waypoints vector");
  if (waypoints.size() == 1)
    return 0;
  // start search
  // dist metric
  double (*f_dist)(const geometry_msgs::Pose&, const geometry_msgs::Pose&);
  if (dist_type == DistMetric::EUCLIDEAN) f_dist = eclideadDist;
  if (dist_type == DistMetric::MANHATTAN) f_dist = manhattanDist;
  // get nearest waypoint
  int nearest{-1};
  double d_nearest = std::numeric_limits<double>::infinity();
  for (int i = 0; i < waypoints.size(); i++) {
    double d_waypoint = f_dist(waypoints[i], robot_pose);
    if(d_waypoint < d_nearest) {
      nearest = i;
      d_nearest = d_waypoint;
    }
  }
  // check which waypoint to go to
  // nearest is last point
  if (nearest == waypoints.size()-1)
    return nearest;
  // check if robot is in front of or behind the nearest point
  // get next waypoint immediately after nearest 
  int next_nearest = nearest + 1;
  double d_next_nearest = f_dist(waypoints[next_nearest], robot_pose);
  double d_btwn_waypoints = f_dist(waypoints[nearest], waypoints[next_nearest]);
  // get the cosine of angle formed by pose,nearest,next_nearest
  double cosine = cosine_a_FromSides(d_next_nearest, d_nearest, d_btwn_waypoints);
  double angle = std::acos(cosine);
  // if the angle is <= 90 deg - tol, pose is in front of nearest (between nearest and next_nearest)
  // if the angle is > 90 deg - tol, pose is in behind nearest 
  double ang_tol = 5.0;   // degrees
  ang_tol = ang_tol / 180.0 * PI;   // radians
  bool is_behind_nearest = angle + ang_tol > PI / 2;
  if (is_behind_nearest)
    return nearest;   // go to nearest
  else
    return next_nearest;   // already past nearest, go to next nearest
}

}   // namespace movel_fms_utils

#endif