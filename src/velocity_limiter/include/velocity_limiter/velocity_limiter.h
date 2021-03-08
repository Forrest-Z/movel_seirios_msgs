#ifndef VELOCITY_LIMITER_H
#define VELOCITY_LIMITER_H
#include <geometry_msgs/Twist.h>
#include <velocity_limiter/common.h>
/**
 * Build the limit set and limit the velocity
 */
class VelocityLimiter
{
public:
  bool buildLimitSet(Set& set);
  void limitVelocity(const geometry_msgs::Twist& velocity_in, geometry_msgs::Twist& velocity_out,
                     VelocityLimit& velocity_limit);
  geometry_msgs::PolygonStamped toPolygonMsg(Frontier& frontier, std::string& base_frame);

private:
  bool checkShape(std::vector<double> x_list, std::vector<double> y_list);
  void buildShape(Shape& shape);
  void sortFrontier(std::vector<Frontier>& frontier_list);
  bool setZoneId(Zone& zone);
  bool hasValidFrontierList(Set& set);
  bool isWithin(Frontier& inner, Frontier& outer);
};

#endif
