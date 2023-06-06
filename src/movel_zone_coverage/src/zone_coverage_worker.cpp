#include <movel_zone_coverage/zone_coverage_worker.h>
#include <chrono>
#include <thread>

ZoneCoverageWorker::ZoneCoverageWorker(std::string task_id, std::vector<geometry_msgs::Point> zone_polygon,
                                       std::shared_ptr<costmap_2d::Costmap2DROS> map,
                                       std::shared_ptr<ZoneCoverageRedisClient> redis,
                                       std::shared_ptr<ros::Publisher> percentage_pub,
                                       std::shared_ptr<ros::Publisher> cell_update_pub)
  : redis_client_(redis), tf_listener_(tf_buffer_), map_(map), coverage_percentage_publisher_(percentage_pub), coverage_cell_update_publisher_(cell_update_pub)
{
  task_id_ = task_id;
  current_status_ = WorkerStatus::INITIALIZED;

  if (!processZonePolygon(zone_polygon))
    return;
}

ZoneCoverageWorker::~ZoneCoverageWorker()
{
}

void ZoneCoverageWorker::setRobotFootprintPolygon(std::vector<geometry_msgs::Point> footprint_polygon)
{
  robot_footprint_ = footprint_polygon;
  use_footprint_radius_ = false;
}

void ZoneCoverageWorker::setRobotFootprintRadius(double footprint_radius)
{
  robot_radius_ = footprint_radius;
  use_footprint_radius_ = true;
}

std::string ZoneCoverageWorker::getTaskId()
{
  return task_id_;
}

WorkerStatus ZoneCoverageWorker::getStatus()
{
  return current_status_;
}

void ZoneCoverageWorker::startWorker()
{
  current_status_ = WorkerStatus::RUNNING;
}

void ZoneCoverageWorker::pauseWorker()
{
  current_status_ = WorkerStatus::PAUSED;
}

void ZoneCoverageWorker::stopWorker()
{
  current_status_ = WorkerStatus::STOPPED;
}

void ZoneCoverageWorker::workerThread()
{
  if (redis_client_->taskExists(task_id_))
    redis_client_->deleteTask(task_id_);

  while (current_status_ != WorkerStatus::STOPPED)
  {
    if (current_status_ != WorkerStatus::RUNNING)
    {
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
      continue;
    }

    if (!getRobotPose(current_pose_))
    {
      ROS_FATAL("[zone_coverage_worker] Failed to get robot pose");
      return;
    }

    // get current occupied cells
    std::vector<geometry_msgs::Point> current_occupied_cells;
    if (use_footprint_radius_)
    {
      geometry_msgs::Point footprint_center;
      footprint_center.x = current_pose_.position.x;
      footprint_center.y = current_pose_.position.y;
      if (!getCircleFillingCells(footprint_center, robot_radius_, current_occupied_cells))
      {
        ROS_FATAL("[zone_coverage_worker] Failed to get occupied cells of circular footprint");
        return;
      }
    }
    else
    {
      // transform footprint to world frame
      geometry_msgs::TransformStamped base_link_to_world = tf_buffer_.lookupTransform("map", "base_link", ros::Time(0));
      std::vector<geometry_msgs::Point> footprint_world;
      for (std::vector<geometry_msgs::Point>::iterator it = robot_footprint_.begin(); it != robot_footprint_.end();
           ++it)
      {
        geometry_msgs::Point footprint_vertex_world;
        tf2::doTransform(*it, footprint_vertex_world, base_link_to_world);
        footprint_world.push_back(footprint_vertex_world);
      }

      if (!getPolygonFillingCells(footprint_world, current_occupied_cells))
      {
        ROS_FATAL("[zone_coverage_worker] Failed to get occupied cells of polygonal footprint");
        return;
      }
    }

    for (std::vector<geometry_msgs::Point>::const_iterator it_current = current_occupied_cells.begin(); it_current != current_occupied_cells.end(); ++it_current)
    {
      for (std::vector<geometry_msgs::Point>::const_iterator it_visited = visited_cells.begin(); it_visited != visited_cells.end(); ++it_visited)
      {
        if (it_visited->x == it_current->x && it_visited->y == it_current->y)
          break;
        visited_cells.push_back(*it_current);
      }
    }
        
    redis_client_->updateVisitedCells(task_id_, current_occupied_cells);
    publishCurrentOccupiedCell(current_occupied_cells);

    area_coverage_percentage = zone_area_ / visited_cells.size();
    redis_client_->setCoveragePercentage(task_id_, area_coverage_percentage);
    publishCoveragePercentage(area_coverage_percentage);
  }

  redis_client_->deleteTask(task_id_);

  current_status_ = WorkerStatus::FINISHED;
}

bool ZoneCoverageWorker::processZonePolygon(const std::vector<geometry_msgs::Point>& zone_polygon_world)
{
  zone_polygon_.clear();
  for (std::vector<geometry_msgs::Point>::const_iterator it = zone_polygon_world.begin();
       it != zone_polygon_world.end(); ++it)
  {
    costmap_2d::MapLocation vertex_map;
    if (!map_->getCostmap()->worldToMap(it->x, it->y, vertex_map.x, vertex_map.y))
      return false;
    zone_polygon_.push_back(vertex_map);
  }

  int num_vertices = zone_polygon_.size();
  double area = 0;
  for (int i = 0; i < num_vertices; ++i)
  {
    int j = (i + 1) % num_vertices;
    area += (zone_polygon_[j].x + zone_polygon_[i].x) * (zone_polygon_[j].y - zone_polygon_[i].y);
  }
  zone_area_ = (unsigned int)std::round(std::abs(area) / 2.0);

  return true;
}

bool ZoneCoverageWorker::getRobotPose(geometry_msgs::Pose& pose)
{
  geometry_msgs::PoseStamped pose_stamped;
  if (!map_->getRobotPose(pose_stamped))
    return false;

  pose = pose_stamped.pose;
  return true;
}

bool ZoneCoverageWorker::getPolygonFillingCells(const std::vector<geometry_msgs::Point>& footprint,
                                                std::vector<geometry_msgs::Point>& cells_world)
{
  std::vector<costmap_2d::MapLocation> footprint_polygon_cells;
  for (const geometry_msgs::Point& p : footprint)
  {
    costmap_2d::MapLocation footprint_map_cell;
    if (!map_->getCostmap()->worldToMap(p.x, p.y, footprint_map_cell.x, footprint_map_cell.y))
      return false;
    footprint_polygon_cells.push_back(footprint_map_cell);
  }

  cells_world.clear();
  std::vector<costmap_2d::MapLocation> footprint_covered_cells;
  map_->getCostmap()->convexFillCells(footprint_polygon_cells, footprint_covered_cells);
  for (const costmap_2d::MapLocation& map_cell : footprint_covered_cells)
  {
    geometry_msgs::Point p;
    map_->getCostmap()->mapToWorld(map_cell.x, map_cell.y, p.x, p.y);
    cells_world.push_back(p);
  }

  return true;
}

bool ZoneCoverageWorker::getCircleFillingCells(const geometry_msgs::Point& center, double radius,
                                               std::vector<geometry_msgs::Point>& cells_world)
{
  unsigned int radius_map = map_->getCostmap()->cellDistance(radius);
  costmap_2d::MapLocation center_map;
  if (!map_->getCostmap()->worldToMap(center.x, center.y, center_map.x, center_map.y))
    return false;
  unsigned int top = std::min(center_map.y + radius_map, map_->getCostmap()->getSizeInCellsY());
  unsigned int bottom = std::max(int(center_map.y) - int(radius_map), 0);
  unsigned int right = std::min(center_map.x + radius_map, map_->getCostmap()->getSizeInCellsX());
  unsigned int left = std::max(int(center_map.x) - int(radius_map), 0);
  std::vector<costmap_2d::MapLocation> cells_map;

  // get filling cells with flood fill algorithm
  std::function<void(costmap_2d::MapLocation)> f_flood_fill = [&](costmap_2d::MapLocation cell) -> void {
    if (cell.x < left || cell.x > right || cell.y < bottom || cell.y > top)
      return;
    // if (std::find(cells_map.begin(), cells_map.end(), cell) != cells_map.end())
    //   return;
    for (std::vector<costmap_2d::MapLocation>::const_iterator it = cells_map.begin(); it != cells_map.end(); ++it)
      if (it->x == cell.x && it->y == cell.y)
        return;

    cells_map.push_back(cell);

    costmap_2d::MapLocation cell_right = { .x = std::min(cell.x + 1, map_->getCostmap()->getSizeInCellsX()), .y = cell.y };
    costmap_2d::MapLocation cell_left = { .x = (unsigned int) std::max(int(cell.x) - 1, 0), .y = cell.y };
    costmap_2d::MapLocation cell_above = { .x = cell.x, .y = std::min(cell.y + 1, map_->getCostmap()->getSizeInCellsY()) };
    costmap_2d::MapLocation cell_below = { .x = cell.x, .y = (unsigned int) std::max(int(cell.y) - 1, 0) };

    f_flood_fill(cell_right);
    f_flood_fill(cell_left);
    f_flood_fill(cell_above);
    f_flood_fill(cell_below);
  };

  for (unsigned int x = left; x <= right; ++x)
  {
    for (unsigned int y = bottom; y <= top; ++y)
    {
      int dx = int(x) - int(center_map.x);
      int dy = int(y) - int(center_map.y);
      if (pow(dx, 2) + pow(dy, 2) <= pow(radius_map, 2))
      {
        costmap_2d::MapLocation cell = { .x = x, .y = y };
        f_flood_fill(cell);
      }
    }
  }

  cells_world.clear();
  for (std::vector<costmap_2d::MapLocation>::iterator it = cells_map.begin(); it != cells_map.end(); ++it)
  {
    geometry_msgs::Point cell_world;
    map_->getCostmap()->mapToWorld(it->x, it->y, cell_world.x, cell_world.y);
    cells_world.push_back(cell_world);
  }

  return true;
}

void ZoneCoverageWorker::publishCurrentOccupiedCell(const std::vector<geometry_msgs::Point>& cells)
{
  movel_seirios_msgs::PointArray msg;
  msg.points = cells;
  coverage_cell_update_publisher_->publish(msg);
}

void ZoneCoverageWorker::publishCoveragePercentage(const double& percentage)
{
  std_msgs::Float64 msg;
  msg.data = percentage;
  coverage_percentage_publisher_->publish(msg);
}