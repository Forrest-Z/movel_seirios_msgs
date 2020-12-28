#include <velocity_limiter/velocity_grid.h>

VelocityGrid::VelocityGrid()
{
  this->initMaxVelocity();
}

VelocityGrid::VelocityGrid(std::vector<Zone>& zone_list, double resolution)
{
  this->load(zone_list, resolution);
}

VelocityGrid::~VelocityGrid()
{
}

/**
 * Set the initial max velocity to be 0.
 */
void VelocityGrid::initMaxVelocity()
{
  max_velocity_.linear.positive.x = 0.0;
  max_velocity_.linear.negative.x = 0.0;
  max_velocity_.linear.positive.y = 0.0;
  max_velocity_.linear.negative.y = 0.0;
  max_velocity_.linear.positive.z = 0.0;
  max_velocity_.linear.negative.z = 0.0;
  max_velocity_.angular.positive.x = 0.0;
  max_velocity_.angular.negative.x = 0.0;
  max_velocity_.angular.positive.y = 0.0;
  max_velocity_.angular.negative.y = 0.0;
  max_velocity_.angular.positive.z = 0.0;
  max_velocity_.angular.negative.z = 0.0;
}

/**
 * Load the velocity grid
 *
 * @param zone_list the list of zones of a velocity limit set.
 * @param resolution resolution of the velocity grid.
 * @return whether it succeed.
 * @see initMaxVelocity
 * @see computeBoundary
 * @see contains
 * @see computeVelocityLimit
 */
bool VelocityGrid::load(std::vector<Zone>& zone_list, double resolution)
{
  // this->initMaxVelocity(max_linear, max_angular);
  grid_.clear();
  this->initMaxVelocity();
  clock_t begin = std::clock();
  if (resolution <= 0.0)
  {
    ROS_FATAL("Invalid resolution: need to be greater than 0");
    return false;
  }
  resolution_ = resolution;
  boundary_ = computeBoundary(zone_list);
  height_ = (boundary_.top_left.x() - boundary_.bottom_right.x()) / resolution + 1;
  width_ = (boundary_.top_left.y() - boundary_.bottom_right.y()) / resolution + 1;
  origin_.position.x = boundary_.bottom_right.x();
  origin_.position.y = boundary_.bottom_right.y();
  origin_.orientation.w = 1.0;
  ROS_WARN("origin (x, y): (%.2f, %.2f)", origin_.position.x, origin_.position.y);
  ROS_WARN("width (x, y): (%d, %d)", height_, width_);
  for (int j = 0; j < width_; j++)
  {
    for (int i = 0; i < height_; i++)
    {
      VelocityGridCell cell;
      cell.position.x(origin_.position.x + i * resolution);
      cell.position.y(origin_.position.y + j * resolution);

      for (auto& zone : zone_list)
      {
        if (contains(zone.frontier_list[0], cell.position))
        {
          switch (zone.id)
          {
            case LINEAR_POSITIVE_X:
              cell.limit.linear.positive.x = 0.0;
              break;
            case LINEAR_NEGATIVE_X:
              cell.limit.linear.negative.x = 0.0;
              break;
            case LINEAR_POSITIVE_Y:
              cell.limit.linear.positive.y = 0.0;
              break;
            case LINEAR_NEGATIVE_Y:
              cell.limit.linear.negative.y = 0.0;
              break;
            case LINEAR_POSITIVE_Z:
              cell.limit.linear.positive.z = 0.0;
              break;
            case LINEAR_NEGATIVE_Z:
              cell.limit.linear.negative.z = 0.0;
              break;
            case ANGULAR_POSITIVE_X:
              cell.limit.angular.positive.x = 0.0;
              break;
            case ANGULAR_NEGATIVE_X:
              cell.limit.angular.negative.x = 0.0;
              break;
            case ANGULAR_POSITIVE_Y:
              cell.limit.angular.positive.y = 0.0;
              break;
            case ANGULAR_NEGATIVE_Y:
              cell.limit.angular.negative.y = 0.0;
              break;
            case ANGULAR_POSITIVE_Z:
              cell.limit.angular.positive.z = 0.0;
              break;
            case ANGULAR_NEGATIVE_Z:
              cell.limit.angular.negative.z = 0.0;
              break;
            default:
              ROS_WARN("Unknown zone id");
              break;
          }
          continue;
        }

        if (!contains(zone.frontier_list.back(), cell.position))
          continue;

        Frontier frontier_inner = zone.frontier_list[0];
        Frontier frontier_outer;
        for (int i = 1; i < zone.frontier_list.size(); ++i)
        {
          frontier_outer = zone.frontier_list[i];
          if (contains(frontier_outer, cell.position))
          {
            double velocity_limit = computeVelocityLimit(cell.position, frontier_inner, frontier_outer);
            switch (zone.id)
            {
              case LINEAR_POSITIVE_X:
                max_velocity_.linear.positive.x = std::max(max_velocity_.linear.positive.x, frontier_outer.value);
                cell.limit.linear.positive.x = std::min(cell.limit.linear.positive.x, velocity_limit);
                break;
              case LINEAR_NEGATIVE_X:
                max_velocity_.linear.negative.x = std::max(max_velocity_.linear.negative.x, frontier_outer.value);
                cell.limit.linear.negative.x = std::min(cell.limit.linear.negative.x, velocity_limit);
                break;
              case LINEAR_POSITIVE_Y:
                max_velocity_.linear.positive.y = std::max(max_velocity_.linear.positive.y, frontier_outer.value);
                cell.limit.linear.positive.y = std::min(cell.limit.linear.positive.y, velocity_limit);
                break;
              case LINEAR_NEGATIVE_Y:
                max_velocity_.linear.negative.y = std::max(max_velocity_.linear.negative.y, frontier_outer.value);
                cell.limit.linear.negative.y = std::min(cell.limit.linear.negative.y, velocity_limit);
                break;
              case LINEAR_POSITIVE_Z:
                max_velocity_.linear.positive.z = std::max(max_velocity_.linear.positive.z, frontier_outer.value);
                cell.limit.linear.positive.z = std::min(cell.limit.linear.positive.z, velocity_limit);
                break;
              case LINEAR_NEGATIVE_Z:
                max_velocity_.linear.negative.z = std::max(max_velocity_.linear.negative.z, frontier_outer.value);
                cell.limit.linear.negative.z = std::min(cell.limit.linear.negative.z, velocity_limit);
                break;
              case ANGULAR_POSITIVE_X:
                max_velocity_.angular.positive.x = std::max(max_velocity_.angular.positive.x, frontier_outer.value);
                cell.limit.angular.positive.x = std::min(cell.limit.angular.positive.x, velocity_limit);
                break;
              case ANGULAR_NEGATIVE_X:
                max_velocity_.angular.negative.x = std::max(max_velocity_.angular.negative.x, frontier_outer.value);
                cell.limit.angular.negative.x = std::min(cell.limit.angular.negative.x, velocity_limit);
                break;
              case ANGULAR_POSITIVE_Y:
                max_velocity_.angular.positive.y = std::max(max_velocity_.angular.positive.y, frontier_outer.value);
                cell.limit.angular.positive.y = std::min(cell.limit.angular.positive.y, velocity_limit);
                break;
              case ANGULAR_NEGATIVE_Y:
                max_velocity_.angular.negative.y = std::max(max_velocity_.angular.negative.y, frontier_outer.value);
                cell.limit.angular.negative.y = std::min(cell.limit.angular.negative.y, velocity_limit);
                break;
              case ANGULAR_POSITIVE_Z:
                max_velocity_.angular.positive.z = std::max(max_velocity_.angular.positive.z, frontier_outer.value);
                cell.limit.angular.positive.z = std::min(cell.limit.angular.positive.z, velocity_limit);
                break;
              case ANGULAR_NEGATIVE_Z:
                max_velocity_.angular.negative.z = std::max(max_velocity_.angular.negative.z, frontier_outer.value);
                cell.limit.angular.negative.z = std::min(cell.limit.angular.negative.z, velocity_limit);
                break;
              default:
                ROS_WARN("Unknown zone id");
                break;
            }
          }

          frontier_inner = zone.frontier_list[i];
        }
      }
      grid_.push_back(cell);
    }
  }
  clock_t end = std::clock();
  load_time_ = ros::Time::now();
  ROS_WARN("Load time: %.2f", double(end - begin) / CLOCKS_PER_SEC);
  return true;
}

/**
 * Compute the local coordinates of a point, with the end of the Segment being the origin
 * and the orientation of the segment (from first to second) being the x axis.
 *
 * @param global_coordinates the coordinates of the point in global frame.
 * @param segment the segment that defines the local frame.
 * @return the coordinates of the point in local frame.
 */
Point VelocityGrid::computeSegmentCoordinates(Point global_coordinates, Segment segment)
{
  // compute orientation of current segment
  double frame_orientation = atan2((segment.second.y() - segment.first.y()), (segment.second.x() - segment.first.x()));

  // compute coordinates of current pose in the current segment frame
  Point segment_coordinates;
  segment_coordinates.x(cos(frame_orientation) * (global_coordinates.x() - segment.second.x()) +
                        sin(frame_orientation) * (global_coordinates.y() - segment.second.y()));
  segment_coordinates.y(-sin(frame_orientation) * (global_coordinates.x() - segment.second.x()) +
                        cos(frame_orientation) * (global_coordinates.y() - segment.second.y()));

  return segment_coordinates;
}

/**
 * Compute the global coordinates of a point. The end of the Segment is the origin of the local frame
 * and the orientation of the segment (from first to second) is the x axis of the local frame.
 *
 * @param local_coordinates the coordinates of the point in local frame.
 * @param segment the segment that defines the local frame.
 * @return the coordinates of the point in global frame.
 */
Point VelocityGrid::computeGlobalCoordinates(Point local_coordinates, Segment segment)
{
  double frame_orientation = atan2((segment.second.y() - segment.first.y()), (segment.second.x() - segment.first.x()));
  Point global_coordinates;
  global_coordinates.x(cos(frame_orientation) * local_coordinates.x() - sin(frame_orientation) * local_coordinates.y() +
                       segment.second.x());
  global_coordinates.y(sin(frame_orientation) * local_coordinates.x() + cos(frame_orientation) * local_coordinates.y() +
                       segment.second.y());
  return global_coordinates;
}

/**
 * Compute velocity limit value of one cell
 *
 * @param position position of the target point.
 * @param frontier_inner the inner frontier.
 * @param frontier_outer the outer frontier.
 * @return the velocity limit value.
 * @see computeSegmentCoordinates
 * @see computeGlobalCoordinates
 */
double VelocityGrid::computeVelocityLimit(Point position, Frontier& frontier_inner, Frontier& frontier_outer)
{
  std::vector<Point> inner_point_list = frontier_inner.shape.polygon.outer();

  Point segment_start = *inner_point_list.begin();
  Point segment_end;
  double shortest_distance = DBL_MAX;
  Segment closest_segment;
  for (auto point = std::next(inner_point_list.begin()); point != inner_point_list.end(); ++point)
  {
    segment_end = *point;

    Segment segment(segment_start, segment_end);
    double distance = boost::geometry::distance(position, segment);
    if (distance < shortest_distance)
    {
      shortest_distance = distance;
      closest_segment = segment;
    }

    segment_start = segment_end;
  }

  Point position_local = computeSegmentCoordinates(position, closest_segment);
  Segment segment_local;
  segment_local.first = computeSegmentCoordinates(closest_segment.first, closest_segment);
  segment_local.second = computeSegmentCoordinates(closest_segment.second, closest_segment);

  Segment gradient_segment;
  gradient_segment.second = position;
  if (position_local.x() < segment_local.first.x())
  {
    gradient_segment.first = closest_segment.first;
  }
  else if (position_local.x() > segment_local.second.x())
  {
    gradient_segment.first = closest_segment.second;
  }
  else
  {
    Point projected_point_local(position_local.x(), 0.);
    gradient_segment.first = computeGlobalCoordinates(projected_point_local, closest_segment);
  }

  double numerator = boost::geometry::distance(gradient_segment.first, gradient_segment.second);

  Segment gradient_segment_local;
  gradient_segment_local.first = computeSegmentCoordinates(gradient_segment.first, gradient_segment);
  gradient_segment_local.second = computeSegmentCoordinates(gradient_segment.second, gradient_segment);
  gradient_segment_local.second.x(1000. / 2.);

  Line gradient_semi_line;
  gradient_semi_line.push_back(gradient_segment.first);
  gradient_semi_line.push_back(computeGlobalCoordinates(gradient_segment_local.second, gradient_segment));

  std::vector<Point> intersection;
  boost::geometry::intersection(gradient_semi_line, getBorder(frontier_outer), intersection);
  if (intersection.size() == 0)
  {
    ROS_ERROR("No intersection found");
  }
  Point outer_intersection = *intersection.begin();
  for (auto point = std::next(intersection.begin()); point != intersection.end(); ++point)
  {
    Point outer_intersection_local = computeSegmentCoordinates(outer_intersection, gradient_segment);
    Point outer_intersection_candidate_local = computeSegmentCoordinates(*point, gradient_segment);
    if (outer_intersection_candidate_local.x() < outer_intersection_local.x())
    {
      outer_intersection = *point;
    }
  }
  double denominator = fabs(boost::geometry::distance(gradient_segment.first, outer_intersection));
  double velocity = numerator / denominator * (frontier_outer.value - frontier_inner.value) + frontier_inner.value;

  return velocity;
}

/**
 * Get the rectangular shape that enclose the zones.
 *
 * @return the rectangular shape that enclose the zones.
 */
BoxHull VelocityGrid::getBoundary()
{
  return this->boundary_;
}

/**
 * Get the origin of the velocity grid.
 *
 */
void VelocityGrid::getOrigin(double& x, double& y)
{
  x = origin_.position.x;
  y = origin_.position.y;
}

/**
 * Get the max velocity limit of the velocity grid.
 *
 * @return the max velocity limit of the velocity grid.
 */
VelocityLimit VelocityGrid::getMaxVelocityLimit()
{
  return max_velocity_;
}

/**
 * Get the velocity limit of the point.
 *
 * @param x x coordinate of the point.
 * @param y y coordinate of the point.
 * @param limit stores the result.
 * @return whether it succeed.
 */
bool VelocityGrid::getVelocityLimit(double x, double y, VelocityLimit& limit)
{
  int i = (x - origin_.position.x) / resolution_;
  int j = (y - origin_.position.y) / resolution_;
  int index = i + height_ * j;
  if (index >= grid_.size())
    return false;
  limit = grid_[i + height_ * j].limit;
  return true;
}

/**
 * Convert the velocity grid of a zone into an occupancy grid.
 *
 * @param zone_id id of the zone being converted.
 * @param base_frame the base frame that defines the frame_id of the message.
 * @return the occupancy grid message.
 */
nav_msgs::OccupancyGrid VelocityGrid::toOccupancyGrid(uint8_t zone_id, std::string base_frame)
{
  nav_msgs::OccupancyGrid grid_msg;
  for (auto& cell : grid_)
  {
    int cell_value = -1;
    switch (zone_id)
    {
      case LINEAR_POSITIVE_X:
        if (cell.limit.linear.positive.x < DBL_MAX)
          cell_value = (int)(cell.limit.linear.positive.x / max_velocity_.linear.positive.x * 100.);
        break;
      case LINEAR_NEGATIVE_X:
        if (cell.limit.linear.negative.x < DBL_MAX)
          cell_value = (int)(cell.limit.linear.negative.x / max_velocity_.linear.negative.x * 100.);
        break;
      case LINEAR_POSITIVE_Y:
        if (cell.limit.linear.positive.y < DBL_MAX)
          cell_value = (int)(cell.limit.linear.positive.y / max_velocity_.linear.positive.y * 100.);
        break;
      case LINEAR_NEGATIVE_Y:
        if (cell.limit.linear.negative.y < DBL_MAX)
          cell_value = (int)(cell.limit.linear.negative.y / max_velocity_.linear.negative.y * 100.);
        break;
      case LINEAR_POSITIVE_Z:
        if (cell.limit.linear.positive.z < DBL_MAX)
          cell_value = (int)(cell.limit.linear.positive.z / max_velocity_.linear.positive.z * 100.);
        break;
      case LINEAR_NEGATIVE_Z:
        if (cell.limit.linear.negative.z < DBL_MAX)
          cell_value = (int)(cell.limit.linear.negative.z / max_velocity_.linear.negative.z * 100.);
        break;
      case ANGULAR_POSITIVE_X:
        if (cell.limit.angular.positive.x < DBL_MAX)
          cell_value = (int)(cell.limit.angular.positive.x / max_velocity_.angular.positive.x * 100.);
        break;
      case ANGULAR_NEGATIVE_X:
        if (cell.limit.angular.negative.x < DBL_MAX)
          cell_value = (int)(cell.limit.angular.negative.x / max_velocity_.angular.negative.x * 100.);
        break;
      case ANGULAR_POSITIVE_Y:
        if (cell.limit.angular.positive.y < DBL_MAX)
          cell_value = (int)(cell.limit.angular.positive.y / max_velocity_.angular.positive.y * 100.);
        break;
      case ANGULAR_NEGATIVE_Y:
        if (cell.limit.angular.negative.y < DBL_MAX)
          cell_value = (int)(cell.limit.angular.negative.y / max_velocity_.angular.negative.y * 100.);
        break;
      case ANGULAR_POSITIVE_Z:
        if (cell.limit.angular.positive.z < DBL_MAX)
          cell_value = (int)(cell.limit.angular.positive.z / max_velocity_.angular.positive.z * 100.);
        break;
      case ANGULAR_NEGATIVE_Z:
        if (cell.limit.angular.negative.z < DBL_MAX)
          cell_value = (int)(cell.limit.angular.negative.z / max_velocity_.angular.negative.z * 100.);
        break;
      case 12:
        if (cell.limit.linear.positive.x < DBL_MAX)
          cell_value = (int)(cell.limit.linear.positive.x / max_velocity_.linear.positive.x * 100.);
        if (cell.limit.linear.negative.x < DBL_MAX)
          cell_value = (int)(cell.limit.linear.negative.x / max_velocity_.linear.negative.x * 100.);
        if (cell.limit.linear.positive.y < DBL_MAX)
          cell_value = (int)(cell.limit.linear.positive.y / max_velocity_.linear.positive.y * 100.);
        if (cell.limit.linear.negative.y < DBL_MAX)
          cell_value = (int)(cell.limit.linear.negative.y / max_velocity_.linear.negative.y * 100.);
        if (cell.limit.linear.positive.z < DBL_MAX)
          cell_value = (int)(cell.limit.linear.positive.z / max_velocity_.linear.positive.z * 100.);
        if (cell.limit.linear.negative.z < DBL_MAX)
          cell_value = (int)(cell.limit.linear.negative.z / max_velocity_.linear.negative.z * 100.);
        if (cell.limit.angular.positive.x < DBL_MAX)
          cell_value = (int)(cell.limit.angular.positive.x / max_velocity_.angular.positive.x * 100.);
        if (cell.limit.angular.negative.x < DBL_MAX)
          cell_value = (int)(cell.limit.angular.negative.x / max_velocity_.angular.negative.x * 100.);
        if (cell.limit.angular.positive.y < DBL_MAX)
          cell_value = (int)(cell.limit.angular.positive.y / max_velocity_.angular.positive.y * 100.);
        if (cell.limit.angular.negative.y < DBL_MAX)
          cell_value = (int)(cell.limit.angular.negative.y / max_velocity_.angular.negative.y * 100.);
        if (cell.limit.angular.positive.z < DBL_MAX)
          cell_value = (int)(cell.limit.angular.positive.z / max_velocity_.angular.positive.z * 100.);
        if (cell.limit.angular.negative.z < DBL_MAX)
          cell_value = (int)(cell.limit.angular.negative.z / max_velocity_.angular.negative.z * 100.);
        break;
      default:
        ROS_WARN("Unknown zone id");
        break;
    }
    grid_msg.data.push_back(cell_value);
  }
  grid_msg.header.frame_id = base_frame;
  grid_msg.info.height = width_;
  grid_msg.info.width = height_;
  grid_msg.info.resolution = resolution_;
  grid_msg.info.origin = origin_;
  grid_msg.info.map_load_time = load_time_;
  return grid_msg;
}

/**
 * Compute the rectangular shape enclosing the zones.
 *
 * @param zone_list the list of zone.
 * @return the rectangular shape.
 */
BoxHull VelocityGrid::computeBoundary(std::vector<Zone>& zone_list)
{
  BoxHull boundary;
  for (auto& zone : zone_list)
  {
    BoxHull zone_hull = zone.frontier_list.back().shape.hull;
    boundary.top_left.x(std::max(boundary.top_left.x(), zone_hull.top_left.x()));
    boundary.top_left.y(std::max(boundary.top_left.y(), zone_hull.top_left.y()));
    boundary.bottom_right.x(std::min(boundary.bottom_right.x(), zone_hull.bottom_right.x()));
    boundary.bottom_right.y(std::min(boundary.bottom_right.y(), zone_hull.bottom_right.y()));
  }
  return boundary;
}

/**
 * Tell whether the point is within the fontier shape.
 *
 * @param frontier the frontier.
 * @param position the point.
 * @return whether the point is within the fontier shape.
 */
bool VelocityGrid::contains(Frontier& frontier, Point& position)
{
  return boost::geometry::covered_by(position, frontier.shape.polygon);
}

/**
 * Convert Polygon to Line
 *
 * @param frontier the frontier being converted.
 * @return the Line.
 */
Line VelocityGrid::getBorder(Frontier& frontier)
{
  Line border;
  for (auto& point : frontier.shape.polygon.outer())
  {
    border.push_back(point);
  }
  return border;
}
