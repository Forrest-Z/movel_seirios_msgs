#include <velocity_limiter/velocity_limiter.h>

/**
 * Build features of the limit set.
 *
 * @param set the limit set.
 * @return whether it succeeds.
 * @see buildShape
 * @see sortFrontier
 * @see setZoneId
 * @see hasValidFrontierList
 */
bool VelocityLimiter::buildLimitSet(Set& set)
{
  for (int i = 0; i < set.zone_list.size(); ++i)
  {
    for (int j = 0; j < set.zone_list[i].frontier_list.size(); ++j)
    {
      buildShape(set.zone_list[i].frontier_list[j].shape);
    }
    sortFrontier(set.zone_list[i].frontier_list);

    if (!setZoneId(set.zone_list[i]))
      return false;
  }

  if (!hasValidFrontierList(set))
  {
    return false;
  }
  return true;
}

/**
 * Add Polygon and BoxHull to the shape.
 *
 * @param shape the shape used.
 * @see checkShape
 */
void VelocityLimiter::buildShape(Shape& shape)
{
  if (!checkShape(shape.x_list, shape.y_list))
    throw;

  for (int i = 0; i < shape.x_list.size(); ++i)
  {
    Point p(shape.x_list[i], shape.y_list[i]);
    shape.polygon.outer().push_back(p);
    shape.hull.top_left.x(std::max(shape.hull.top_left.x(), shape.x_list[i]));
    shape.hull.top_left.y(std::max(shape.hull.top_left.y(), shape.y_list[i]));
    shape.hull.bottom_right.x(std::min(shape.hull.bottom_right.x(), shape.x_list[i]));
    shape.hull.bottom_right.y(std::min(shape.hull.bottom_right.y(), shape.y_list[i]));
  }
}

/**
 * Check whether the shape has valid lists of coordinates. size of x and y lists should be equal.
 * Size of list should be greater than 3. First and last element of the list should be equal.
 *
 * @param x_list the array of x coordinate.
 * @param y_list the array of y coordinate.
 */
bool VelocityLimiter::checkShape(std::vector<double> x_list, std::vector<double> y_list)
{
  if (x_list.size() != y_list.size())
  {
    ROS_FATAL("Invalid shape. X list has %lu point, Y list has %lu points.", x_list.size(), y_list.size());
    return false;
  }
  else if (x_list.size() < 4)
  {
    ROS_FATAL("Invalid shape. At least 4 points required, but %lu provided.", x_list.size());
    return false;
  }
  else if (x_list.front() != x_list.back())
  {
    ROS_FATAL("Invalid shape. First and last point of X list need to be equal, but %f and %f provided.", x_list.front(),
              x_list.back());
    return false;
  }
  else if (y_list.front() != y_list.back())
  {
    ROS_FATAL("Invalid shape. First and last point of X list need to be equal, but %f and %f provided.", y_list.front(),
              y_list.back());
    return false;
  }
  return true;
}

/**
 * Sort a list of frontier in the order of ascending frontier inclusion.
 *
 * @param frontier_list the list of frontiers.
 */
void VelocityLimiter::sortFrontier(std::vector<Frontier>& frontier_list)
{
  std::sort(frontier_list.begin(), frontier_list.end(),
            [](Frontier a, Frontier b) { return a.inclusion < b.inclusion; });
}

/**
 * Set the zone id of a zone based on its type, direction and dimension.
 *
 * @param zone the zone used.
 * @return whether it suceeds.
 */
bool VelocityLimiter::setZoneId(Zone& zone)
{
  // if (zone.id == LINEAR_POSITIVE_X || zone.id == LINEAR_NEGATIVE_X || zone.id == ANGULAR_POSITIVE_Z || zone.id ==
  // ANGULAR_NEGATIVE_Z)
  //   return true;
  std::stringstream zone_id;
  zone_id << zone.type << "_" << zone.direction << "_" << zone.dimension;
  if (zone_id.str() == "linear_positive_x")
    zone.id = LINEAR_POSITIVE_X;
  else if (zone_id.str() == "linear_negative_x")
    zone.id = LINEAR_NEGATIVE_X;
  else if (zone_id.str() == "linear_positive_y")
    zone.id = LINEAR_POSITIVE_Y;
  else if (zone_id.str() == "linear_negative_y")
    zone.id = LINEAR_NEGATIVE_Y;
  else if (zone_id.str() == "linear_positive_z")
    zone.id = LINEAR_POSITIVE_Z;
  else if (zone_id.str() == "linear_negative_z")
    zone.id = LINEAR_NEGATIVE_Z;
  else if (zone_id.str() == "angular_positive_x")
    zone.id = ANGULAR_POSITIVE_X;
  else if (zone_id.str() == "angular_negative_x")
    zone.id = ANGULAR_NEGATIVE_X;
  else if (zone_id.str() == "angular_positive_y")
    zone.id = ANGULAR_POSITIVE_Y;
  else if (zone_id.str() == "angular_negative_y")
    zone.id = ANGULAR_NEGATIVE_Y;
  else if (zone_id.str() == "angular_positive_z")
    zone.id = ANGULAR_POSITIVE_Z;
  else if (zone_id.str() == "angular_negative_z")
    zone.id = ANGULAR_NEGATIVE_Z;
  else
  {
    ROS_WARN_STREAM("Zone id " << zone_id.str() << " not allowed.");
    return false;
  }
  return true;
}

/**
 * Check whether the frontier list is valid. The list should not be empty. It must contains a frontier with 0 inclusion.
 * Larger inclusion frontier should contains smaller ones
 *
 * @param set the set being checked.
 * @return whether the frontier list is valid.
 * @see isWithin
 */
bool VelocityLimiter::hasValidFrontierList(Set& set)
{
  for (int j = 0; j < set.zone_list.size(); ++j)
  {
    if (!set.zone_list[j].frontier_list.size() > 0)
    {
      ROS_FATAL("Zone %d: Frontier list is empty", j);
      return false;
    }

    if (set.zone_list[j].frontier_list[0].inclusion != 0)
    {
      ROS_FATAL("Zone %d, Invalid frontiers inclusion: missing frontier with inclusion 0", j);
      return false;
    }

    std::map<int, int> inclusion_counter;
    if (set.zone_list[j].frontier_list.size() > 1)
    {
      for (int i = 0; i < set.zone_list[j].frontier_list.size(); ++i)
      {
        if (inclusion_counter.find(set.zone_list[j].frontier_list[i].inclusion) == inclusion_counter.end())
        {
          inclusion_counter[set.zone_list[j].frontier_list[i].inclusion] = 1;
        }
        else
        {
          ROS_FATAL("Zone %d, Invalid frontiers inclusion: multiple instances of %d (should be unique)", j,
                    set.zone_list[j].frontier_list[i].inclusion);
          return false;
        }

        if (i < set.zone_list[j].frontier_list.size() - 1 &&
            !isWithin(set.zone_list[j].frontier_list[i], set.zone_list[j].frontier_list[i + 1]))
        {
          ROS_FATAL("Zone %d, Invalid frontiers inclusion: %d not included in %d", j,
                    set.zone_list[j].frontier_list[i].inclusion, set.zone_list[j].frontier_list[i + 1].inclusion);
          return false;
        }

        // if (i > 0 && !frontier_list.front().excludeInner(frontier_list[i-1])) return false;
      }
    }
  }

  return true;
}

/**
 * Check whether one frontier is within another. Overlap of borders is allowed.
 *
 * @param inner the inner frontier.
 * @param outer the outer frontier.
 * @return whether one frontier is within another.
 */
bool VelocityLimiter::isWithin(Frontier& inner, Frontier& outer)
{
  return boost::geometry::covered_by(inner.shape.polygon, outer.shape.polygon);
}

/**
 * Apply the velocity limit
 *
 * @param velocity_in the input velocity message.
 * @param velocity_out the output velocity message.
 * @param velocity_limit the final velocity limit calculated based on point cloud.
 */
void VelocityLimiter::limitVelocity(const geometry_msgs::Twist& velocity_in, geometry_msgs::Twist& velocity_out,
                                    VelocityLimit& velocity_limit)
{
  // ROS_INFO("Vel limit x: %.2f", velocity_limit_.linear.positive.x);
  // ROS_INFO("Vel limit z: %.2f", velocity_limit_.angular.negative.z);
  velocity_out.linear.x = velocity_in.linear.x;
  velocity_out.linear.y = velocity_in.linear.y;
  velocity_out.linear.z = velocity_in.linear.z;
  velocity_out.angular.x = velocity_in.angular.x;
  velocity_out.angular.y = velocity_in.angular.y;
  velocity_out.angular.z = velocity_in.angular.z;

  if (velocity_in.linear.x > 0)
  {
    if (velocity_in.linear.x > velocity_limit.linear.positive.x)
    {
      velocity_out.linear.x = velocity_limit.linear.positive.x;
      // ROS_INFO("+ Lin capped");
    }
  }
  else
  {
    if (velocity_in.linear.x < -velocity_limit.linear.negative.x)
    {
      velocity_out.linear.x = -velocity_limit.linear.negative.x;
      // ROS_INFO("- Lin capped");
    }
  }

  if (velocity_in.linear.y > 0)
  {
    if (velocity_in.linear.y > velocity_limit.linear.positive.y)
    {
      velocity_out.linear.y = velocity_limit.linear.positive.y;
      // ROS_INFO("+ Lin capped");
    }
  }
  else
  {
    if (velocity_in.linear.y < -velocity_limit.linear.negative.y)
    {
      velocity_out.linear.y = -velocity_limit.linear.negative.y;
      // ROS_INFO("- Lin capped");
    }
  }

  if (velocity_in.linear.z > 0)
  {
    if (velocity_in.linear.z > velocity_limit.linear.positive.z)
    {
      velocity_out.linear.z = velocity_limit.linear.positive.z;
      // ROS_INFO("+ Lin capped");
    }
  }
  else
  {
    if (velocity_in.linear.z < -velocity_limit.linear.negative.z)
    {
      velocity_out.linear.z = -velocity_limit.linear.negative.z;
      // ROS_INFO("- Lin capped");
    }
  }

  if (velocity_in.angular.x > 0)
  {
    if (velocity_in.angular.x > velocity_limit.angular.positive.x)
    {
      velocity_out.angular.x = velocity_limit.angular.positive.x;
      // ROS_INFO("+ Ang capped");
    }
  }
  else
  {
    if (velocity_in.angular.x < -velocity_limit.angular.negative.x)
    {
      velocity_out.angular.x = -velocity_limit.angular.negative.x;
      // ROS_INFO("- Ang capped");
    }
  }

  if (velocity_in.angular.y > 0)
  {
    if (velocity_in.angular.y > velocity_limit.angular.positive.y)
    {
      velocity_out.angular.y = velocity_limit.angular.positive.y;
      // ROS_INFO("+ Ang capped");
    }
  }
  else
  {
    if (velocity_in.angular.y < -velocity_limit.angular.negative.y)
    {
      velocity_out.angular.y = -velocity_limit.angular.negative.y;
      // ROS_INFO("- Ang capped");
    }
  }

  if (velocity_in.angular.z > 0)
  {
    if (velocity_in.angular.z > velocity_limit.angular.positive.z)
    {
      velocity_out.angular.z = velocity_limit.angular.positive.z;
      // ROS_INFO("+ Ang capped");
    }
  }
  else
  {
    if (velocity_in.angular.z < -velocity_limit.angular.negative.z)
    {
      velocity_out.angular.z = -velocity_limit.angular.negative.z;
      // ROS_INFO("- Ang capped");
    }
  }
}
/**
 * Convert Polygon to polygon message
 *
 * @param frontier the frontier whose shape will be converted.
 * @param base_frame the base frame used in the polygon message.
 * @return the polygon message.
 */
geometry_msgs::PolygonStamped VelocityLimiter::toPolygonMsg(Frontier& frontier, std::string& base_frame)
{
  std::vector<geometry_msgs::Point32> point_list;
  for (auto& point : frontier.shape.polygon.outer())
  {
    geometry_msgs::Point32 p;
    p.x = point.x();
    p.y = point.y();
    point_list.push_back(p);
  }
  geometry_msgs::Polygon polygon;
  polygon.points = point_list;

  geometry_msgs::PolygonStamped polygon_stamped;
  polygon_stamped.header.frame_id = base_frame;
  polygon_stamped.polygon = polygon;
  return polygon_stamped;
}
